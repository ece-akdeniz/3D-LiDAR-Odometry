#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

namespace lidar_odometry {
using Cloud = std::vector<Eigen::Vector3d>;
using Pose = Eigen::Isometry3d;  // T_odom_cloud: cloud coordinates -> odom.

struct Options {
  double voxel_size = 0.15;
  double max_correspondence_distance = 0.8;
  double huber_delta = 0.15;
  double trim_fraction = 0.85;
  double min_overlap = 0.3;
  double max_rmse = 0.3;
  double translation_epsilon = 1e-4;
  double rotation_epsilon = 1e-4;
  double max_translation_step = 1.0;
  double max_rotation_step = 0.6;
  double keyframe_translation = 0.3;
  double keyframe_rotation = 0.15;
  int max_iterations = 40;
  int min_correspondences = 30;
  int max_scan_points = 12000;
  int max_keyframes = 12;
  int max_map_points = 60000;
  void validate() const;
};

Cloud voxelDownsample(const Cloud &cloud, double size, std::size_t limit);

// Balanced kd-tree with exact nearest-neighbor search and owned storage.
class KdTree {
 public:
  explicit KdTree(Cloud points);
  bool nearest(const Eigen::Vector3d &query, double max_squared_distance,
               Eigen::Vector3d &point, double &squared_distance) const;
 private:
  struct Node { std::size_t point; int left = -1, right = -1, axis = 0; };
  Cloud points_;
  std::vector<Node> nodes_;
  int root_ = -1;
  int build(std::vector<std::size_t> &indices, std::size_t begin,
            std::size_t end, int depth);
  void search(int node, const Eigen::Vector3d &query, double &best,
              std::size_t &index) const;
};

struct Registration {
  Pose pose = Pose::Identity();
  bool accepted = false;
  bool converged = false;
  int iterations = 0;
  std::size_t correspondences = 0;
  double overlap = 0.0;
  double rmse = std::numeric_limits<double>::infinity();
};

Registration align(const Cloud &source, const KdTree &target,
                   const Pose &initial, const Options &options);

struct Update {
  Pose pose = Pose::Identity();
  bool accepted = false;
  bool initialized = false;
  Registration registration;
  std::size_t scan_points = 0;
  std::size_t map_points = 0;
};

class Odometry {
 public:
  explicit Odometry(Options options = {});
  Update update(const Cloud &cloud);
  void reset();
 private:
  struct Keyframe { Pose pose; Cloud points; };
  Options options_;
  Pose pose_ = Pose::Identity();
  std::deque<Keyframe> keyframes_;
  Cloud map_;
  void insertKeyframe(const Cloud &scan);
};
}  // namespace lidar_odometry
