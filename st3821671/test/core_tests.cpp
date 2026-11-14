#include "lidar_odometry/core.hpp"
#include "lidar_odometry/point_cloud.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

using namespace lidar_odometry;
namespace {
void require(bool condition, const std::string &message) {
  if (!condition) throw std::runtime_error(message);
}
template<typename F> void throws(F f, const std::string &message) {
  bool raised = false;
  try { f(); } catch (const std::invalid_argument &) { raised = true; }
  require(raised, message);
}
Cloud scene() {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> xy(-4.0, 4.0), z(-1.0, 3.0);
  Cloud cloud;
  for (int i = 0; i < 1400; ++i) cloud.emplace_back(xy(rng), xy(rng), z(rng));
  return cloud;
}
Pose motion(double x, double y, double z, double yaw) {
  Pose p = Pose::Identity();
  p.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(yaw * 0.3, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw * 0.2, Eigen::Vector3d::UnitX())).toRotationMatrix();
  p.translation() = Eigen::Vector3d(x, y, z);
  return p;
}
Cloud observe(const Cloud &world, const Pose &pose) {
  Cloud c;
  for (const auto &p : world) c.push_back(pose.inverse() * p);
  return c;
}
void testTree() {
  const auto cloud = scene();
  KdTree tree(cloud);
  for (const auto &p : cloud) {
    Eigen::Vector3d q; double d;
    require(tree.nearest(p, 1, q, d) && d < 1e-20, "kd-tree self query");
  }
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> random(-7, 7);
  for (int i = 0; i < 200; ++i) {
    Eigen::Vector3d query(random(rng), random(rng), random(rng)), q;
    double expected = 1e6, d;
    for (const auto &p : cloud) expected = std::min(expected, (p - query).squaredNorm());
    require(tree.nearest(query, 1e6, q, d) && std::abs(d - expected) < 1e-10,
        "kd-tree must agree with brute force");
  }
  Eigen::Vector3d q; double d;
  require(!KdTree({}).nearest(Eigen::Vector3d::Zero(), 1, q, d), "empty tree");
  require(!tree.nearest(Eigen::Vector3d(100, 100, 100), 0.01, q, d), "distance gate");
}
void testAlignment() {
  const auto world = scene();
  KdTree tree(world);
  Options options;
  const auto identity = align(world, tree, Pose::Identity(), options);
  require(identity.accepted && identity.pose.matrix().isApprox(Pose::Identity().matrix(), 1e-8),
      "ICP identity");
  const Pose expected = motion(0.14, -0.08, 0.05, 0.04);
  const auto moving = observe(world, expected);
  const auto result = align(moving, tree, Pose::Identity(), options);
  require(result.accepted && result.pose.matrix().isApprox(expected.matrix(), 1e-5),
      "ICP must recover 6DoF source-to-target transform");
  auto contaminated = moving;
  for (int i = 0; i < 200; ++i) contaminated.emplace_back(40 + i, 60, -30);
  const auto robust = align(contaminated, tree, Pose::Identity(), options);
  require(robust.accepted && robust.pose.matrix().isApprox(expected.matrix(), 1e-5),
      "ICP must reject distant outliers");
  Cloud line;
  for (int i = 0; i < 100; ++i) line.emplace_back(i * 0.1, 0, 0);
  require(!align(line, KdTree(line), Pose::Identity(), options).accepted,
      "rank-deficient line must be rejected");
  require(!align({}, tree, Pose::Identity(), options).accepted, "empty ICP source");
  auto strict = options; strict.max_translation_step = 0.02;
  require(!align(moving, tree, Pose::Identity(), strict).accepted, "implausible jump gate");
  auto bad = options; bad.trim_fraction = 0;
  throws([&] { Odometry o(bad); }, "invalid parameters");
}
void testOdometry() {
  Options o;
  o.voxel_size = 0.05; o.max_keyframes = 3; o.max_map_points = 3000;
  o.keyframe_translation = 0.08;
  Odometry odom(o);
  const auto world = scene();
  const auto first = odom.update(world);
  require(first.accepted && first.initialized, "first scan initializes");
  Pose expected;
  for (int i = 1; i <= 20; ++i) {
    expected = motion(i * 0.045, i * 0.01, i * 0.015, i * 0.008);
    const auto update = odom.update(observe(world, expected));
    require(update.accepted, "synthetic trajectory must track every frame");
    require((update.pose.translation() - expected.translation()).norm() < 0.02,
        "synthetic trajectory translation drift");
    require(Eigen::AngleAxisd(expected.rotation().transpose() * update.pose.rotation()).angle() < 0.01,
        "synthetic trajectory rotation drift");
    require(update.map_points <= 3000 && update.scan_points <= 12000, "memory point caps");
  }
  const auto missing = odom.update({});
  require(!missing.accepted && (missing.pose.translation() - expected.translation()).norm() < 0.02,
      "empty scan holds pose");
  Cloud distant;
  for (const auto &p : world) distant.push_back(p + Eigen::Vector3d(100, 100, 100));
  const auto rejected = odom.update(distant);
  require(!rejected.accepted, "no overlap scan rejected");
  const auto recovered = odom.update(observe(world, expected));
  require(recovered.accepted, "rejected scan must not poison map");
  odom.reset();
  require(odom.update(world).initialized, "reset reinitializes");
  Cloud invalid{Eigen::Vector3d(NAN, 0, 0), Eigen::Vector3d(INFINITY, 0, 0)};
  require(voxelDownsample(invalid, 0.1, 10).empty(), "nonfinite filtering");
  require(voxelDownsample(world, 0.01, 73).size() == 73, "deterministic scan cap");
}

struct Field { std::string name; std::uint32_t offset; std::uint8_t datatype; std::uint32_t count = 1; };
struct Message {
  std::uint32_t width = 1, height = 2, point_step = 24, row_step = 32;
  bool is_bigendian = false;
  std::vector<Field> fields{{"x", 1, 7}, {"y", 5, 8}, {"z", 13, 7}, {"intensity", 17, 4}};
  std::vector<std::uint8_t> data = std::vector<std::uint8_t>(64);
};
template<typename T> void put(Message &msg, std::size_t offset, T value) {
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
  const std::uint16_t one = 1;
  const bool host_bigendian = *reinterpret_cast<const std::uint8_t *>(&one) == 0;
  if (host_bigendian != msg.is_bigendian) std::reverse(bytes.begin(), bytes.end());
  std::copy(bytes.begin(), bytes.end(), msg.data.begin() + offset);
}
void testDecoder() {
  Filter filter; filter.intensity_threshold = 10;
  for (bool big : {false, true}) {
    Message msg; msg.is_bigendian = big;
    for (std::size_t row : {0, 32}) {
      put<float>(msg, row + 1, 1.5f); put<double>(msg, row + 5, -2.0);
      put<float>(msg, row + 13, 3.0f); put<std::uint16_t>(msg, row + 17, 25);
    }
    const auto cloud = decodeCloud(msg, filter);
    require(cloud.size() == 2 && cloud[1].isApprox(Eigen::Vector3d(1.5, -2, 3)),
        "decoder must handle padding, unaligned mixed fields, and either endian");
    auto capped = filter; capped.max_input_points = 1;
    require(decodeCloud(msg, capped).size() == 1, "decoder input limit");
    put<std::uint16_t>(msg, 17, 2);
    require(decodeCloud(msg, filter).size() == 1, "intensity filtering");
    put<float>(msg, 33, NAN);
    require(decodeCloud(msg, filter).empty(), "nonfinite coordinates");
    msg.data.resize(8);
    throws([&] { decodeCloud(msg, filter); }, "truncated data");
  }
  Message missing; missing.fields.pop_back();
  throws([&] { decodeCloud(missing, filter); }, "missing intensity with threshold");
  missing.fields.erase(missing.fields.begin());
  throws([&] { decodeCloud(missing, Filter{}); }, "missing XYZ");
  Message malformed; malformed.fields[0].offset = 24;
  throws([&] { decodeCloud(malformed, filter); }, "field out of bounds");
  malformed = Message{}; malformed.row_step = 1;
  throws([&] { decodeCloud(malformed, filter); }, "invalid row stride");
  malformed = Message{}; malformed.fields[0].datatype = 99;
  throws([&] { decodeCloud(malformed, filter); }, "unsupported datatype");
}
}  // namespace
int main() {
  try {
    testTree(); std::cout << "PASS exact kd-tree\n";
    testAlignment(); std::cout << "PASS robust ICP and degeneracy/jump guards\n";
    testOdometry(); std::cout << "PASS trajectory, recovery, reset and bounded map\n";
    testDecoder(); std::cout << "PASS PointCloud2 decoding and malformed input\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n'; return 1;
  }
  return 0;
}
