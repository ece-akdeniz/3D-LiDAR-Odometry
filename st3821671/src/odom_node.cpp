#include "lidar_odometry/core.hpp"
#include "lidar_odometry/point_cloud.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <string>

namespace lidar_odometry {
class OdometryNode : public rclcpp::Node {
 public:
  OdometryNode() : Node("lidar_odometry") {
    Options o;
    o.voxel_size = declare_parameter("voxel_size", o.voxel_size);
    o.max_correspondence_distance = declare_parameter("max_correspondence_distance", o.max_correspondence_distance);
    o.huber_delta = declare_parameter("huber_delta", o.huber_delta);
    o.trim_fraction = declare_parameter("trim_fraction", o.trim_fraction);
    o.min_overlap = declare_parameter("min_overlap", o.min_overlap);
    o.max_rmse = declare_parameter("max_rmse", o.max_rmse);
    o.translation_epsilon = declare_parameter("translation_epsilon", o.translation_epsilon);
    o.rotation_epsilon = declare_parameter("rotation_epsilon", o.rotation_epsilon);
    o.max_translation_step = declare_parameter("max_translation_step", o.max_translation_step);
    o.max_rotation_step = declare_parameter("max_rotation_step", o.max_rotation_step);
    o.keyframe_translation = declare_parameter("keyframe_translation", o.keyframe_translation);
    o.keyframe_rotation = declare_parameter("keyframe_rotation", o.keyframe_rotation);
    o.max_iterations = declare_parameter("max_iterations", o.max_iterations);
    o.min_correspondences = declare_parameter("min_correspondences", o.min_correspondences);
    o.max_scan_points = declare_parameter("max_scan_points", o.max_scan_points);
    o.max_keyframes = declare_parameter("max_keyframes", o.max_keyframes);
    o.max_map_points = declare_parameter("max_map_points", o.max_map_points);
    odometry_ = std::make_unique<Odometry>(o);
    filter_.min_range = declare_parameter("min_range", filter_.min_range);
    filter_.max_range = declare_parameter("max_range", filter_.max_range);
    filter_.intensity_threshold = declare_parameter("intensity_threshold", filter_.intensity_threshold);
    const int input_limit = declare_parameter("max_input_points", 200000);
    if (input_limit < 1) throw std::invalid_argument("max_input_points must be positive");
    filter_.max_input_points = static_cast<std::size_t>(input_limit);
    filter_.validate();
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    const auto topic = declare_parameter<std::string>("points_topic", "/points");
    max_path_ = declare_parameter("max_path_poses", 5000);
    if (odom_frame_.empty() || odom_frame_[0] == '/' || max_path_ < 1)
      throw std::invalid_argument("invalid odom_frame or max_path_poses");
    if (declare_parameter("save_trajectory", true)) {
      const auto path = declare_parameter<std::string>("trajectory_path", "./estimate.txt");
      // Append preserves previous runs. Use a fresh path when evaluating each bag.
      trajectory_.open(path, std::ios::app);
      if (!trajectory_) throw std::runtime_error("cannot open trajectory file: " + path);
      trajectory_ << std::fixed << std::setprecision(9);
    } else {
      declare_parameter<std::string>("trajectory_path", "./estimate.txt");
    }
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory", 1);
    // Parameters describe startup configuration. Reject runtime edits rather than
    // reporting success while the algorithm silently retains the old values.
    parameter_callback_ = add_on_set_parameters_callback([](const auto &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto &p : parameters) {
        if (p.get_name() != "use_sim_time") {
          result.successful = false;
          result.reason = "odometry parameters are startup-only; edit YAML and restart";
          break;
        }
      }
      return result;
    });
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS().keep_last(2),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { process(*msg); });
    RCLCPP_INFO(get_logger(), "Listening on %s; publishing %s <- incoming cloud frame",
                topic.c_str(), odom_frame_.c_str());
  }

 private:
  void process(const sensor_msgs::msg::PointCloud2 &msg) {
    try {
      const auto &frame = msg.header.frame_id;
      if (frame.empty() || frame[0] == '/' || frame == odom_frame_ ||
          (!cloud_frame_.empty() && frame != cloud_frame_)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Dropping empty, invalid or changed cloud frame");
        return;
      }
      const rclcpp::Time stamp(msg.header.stamp);
      if (last_stamp_ && stamp <= *last_stamp_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Dropping duplicate/out-of-order stamp; restart node before replaying a bag");
        return;
      }
      const auto cloud = decodeCloud(msg, filter_);
      const auto update = odometry_->update(cloud);
      if (!update.accepted) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "Alignment rejected: points=%zu overlap=%.2f rmse=%.3f; holding pose, map unchanged",
            update.scan_points, update.registration.overlap, update.registration.rmse);
        // Once initialized, explicitly publish the held pose at this scan's stamp.
        // Before initialization there is no estimate to publish.
        if (!cloud_frame_.empty()) publish(msg, update.pose, false);
        last_stamp_ = stamp;
        return;
      }
      cloud_frame_ = frame;
      last_stamp_ = stamp;
      publish(msg, update.pose, true);
      RCLCPP_DEBUG(get_logger(), "scan=%zu map=%zu iterations=%d rmse=%.4f",
          update.scan_points, update.map_points, update.registration.iterations, update.registration.rmse);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Dropping cloud: %s", e.what());
    }
  }

  void publish(const sensor_msgs::msg::PointCloud2 &msg, const Pose &pose, bool accepted) {
    Eigen::Quaterniond q(pose.rotation());
    q.normalize();
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg.header;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = cloud_frame_;
    tf.transform.translation.x = pose.translation().x();
    tf.transform.translation.y = pose.translation().y();
    tf.transform.translation.z = pose.translation().z();
    tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
    tf_->sendTransform(tf);
    nav_msgs::msg::Odometry odom;
    odom.header = tf.header; odom.child_frame_id = tf.child_frame_id;
    odom.pose.pose.position.x = pose.translation().x();
    odom.pose.pose.position.y = pose.translation().y();
    odom.pose.pose.position.z = pose.translation().z();
    odom.pose.pose.orientation = tf.transform.rotation;
    // No calibrated uncertainty model: mark a conservative nominal covariance;
    // rejected alignments receive much larger uncertainty.
    for (int i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 6 + i] = accepted ? 0.1 : 1e6;
      odom.twist.covariance[i * 6 + i] = 1e6;  // Twist is not estimated.
    }
    odom_pub_->publish(odom);
    geometry_msgs::msg::PoseStamped point;
    point.header = tf.header; point.pose = odom.pose.pose;
    path_.header = tf.header;
    path_.poses.push_back(point);
    if (path_.poses.size() > static_cast<std::size_t>(max_path_)) path_.poses.erase(path_.poses.begin());
    path_pub_->publish(path_);
    if (trajectory_.is_open()) {
      // Integer seconds plus nanoseconds avoids losing timestamp precision.
      trajectory_ << msg.header.stamp.sec << '.' << std::setw(9) << std::setfill('0')
          << msg.header.stamp.nanosec << std::setfill(' ') << ' '
          << pose.translation().x() << ' ' << pose.translation().y() << ' '
          << pose.translation().z() << ' ' << q.x() << ' ' << q.y() << ' '
          << q.z() << ' ' << q.w() << '\n';
      trajectory_.flush();
      if (!trajectory_) {
        RCLCPP_ERROR(get_logger(), "Trajectory write failed; disabling file output");
        trajectory_.close();
      }
    }
  }
  Filter filter_;
  std::unique_ptr<Odometry> odometry_;
  std::string odom_frame_, cloud_frame_;
  std::optional<rclcpp::Time> last_stamp_;
  int max_path_ = 5000;
  std::ofstream trajectory_;
  nav_msgs::msg::Path path_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace lidar_odometry

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try { rclcpp::spin(std::make_shared<lidar_odometry::OdometryNode>()); }
  catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("lidar_odometry"), "Startup failed: %s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
