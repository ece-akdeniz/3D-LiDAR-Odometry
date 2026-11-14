#include "lidar_odometry/core.hpp"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <stdexcept>
#include <tuple>

namespace lidar_odometry {
namespace {
double angle(const Pose &pose) {
  return Eigen::AngleAxisd(pose.rotation()).angle();
}
struct Match { Eigen::Vector3d source, target; double distance; };
std::vector<Match> matches(const Cloud &source, const KdTree &target,
                           const Pose &pose, double max_distance) {
  std::vector<Match> result;
  result.reserve(source.size());
  for (const auto &p : source) {
    if (!p.allFinite()) continue;
    const Eigen::Vector3d q = pose * p;
    Eigen::Vector3d nearest;
    double d;
    if (target.nearest(q, max_distance * max_distance, nearest, d))
      result.push_back({q, nearest, std::sqrt(d)});
  }
  return result;
}
void trim(std::vector<Match> &pairs, const Options &o) {
  std::sort(pairs.begin(), pairs.end(), [](const Match &a, const Match &b) {
    return a.distance < b.distance;
  });
  const auto count = std::max(static_cast<std::size_t>(o.min_correspondences),
      static_cast<std::size_t>(std::floor(pairs.size() * o.trim_fraction)));
  if (pairs.size() > count) pairs.resize(count);
}
}  // namespace

void Options::validate() const {
  for (double v : {voxel_size, max_correspondence_distance, huber_delta,
                  max_rmse, translation_epsilon, rotation_epsilon,
                  max_translation_step, max_rotation_step,
                  keyframe_translation, keyframe_rotation}) {
    if (!std::isfinite(v) || v <= 0.0)
      throw std::invalid_argument("odometry distances and thresholds must be finite and positive");
  }
  if (!std::isfinite(trim_fraction) || trim_fraction <= 0 || trim_fraction > 1 ||
      !std::isfinite(min_overlap) || min_overlap <= 0 || min_overlap > 1 ||
      max_iterations < 1 || min_correspondences < 3 ||
      max_scan_points < min_correspondences || max_keyframes < 1 ||
      max_map_points < min_correspondences)
    throw std::invalid_argument("invalid ICP ratios, counts, or map limits");
}

Cloud voxelDownsample(const Cloud &cloud, double size, std::size_t limit) {
  if (!std::isfinite(size) || size <= 0 || limit == 0)
    throw std::invalid_argument("invalid voxel size or point limit");
  struct Cell { Eigen::Vector3d sum = Eigen::Vector3d::Zero(); std::size_t n = 0; };
  std::map<std::tuple<long long, long long, long long>, Cell> cells;
  for (const auto &p : cloud) {
    if (!p.allFinite()) continue;
    const Eigen::Array3d q = (p / size).array().floor();
    // Avoid undefined floating-to-integer conversion on corrupt input.
    if ((q.abs() > 1e12).any()) continue;
    auto &cell = cells[{static_cast<long long>(q.x()),
                        static_cast<long long>(q.y()), static_cast<long long>(q.z())}];
    cell.sum += p;
    ++cell.n;
  }
  Cloud result;
  result.reserve(std::min(limit, cells.size()));
  // Uniform deterministic thinning across sorted cells, rather than keeping one side.
  std::size_t i = 0, next = 0;
  const std::size_t count = std::min(limit, cells.size());
  for (const auto &entry : cells) {
    if (result.size() < count && i == next) {
      result.push_back(entry.second.sum / static_cast<double>(entry.second.n));
      next = result.size() * cells.size() / count;
    }
    ++i;
  }
  return result;
}

KdTree::KdTree(Cloud points) : points_(std::move(points)) {
  points_.erase(std::remove_if(points_.begin(), points_.end(),
      [](const auto &p) { return !p.allFinite(); }), points_.end());
  std::vector<std::size_t> indices(points_.size());
  std::iota(indices.begin(), indices.end(), 0);
  nodes_.reserve(points_.size());
  root_ = build(indices, 0, indices.size(), 0);
}
int KdTree::build(std::vector<std::size_t> &indices, std::size_t begin,
                  std::size_t end, int depth) {
  if (begin == end) return -1;
  const auto mid = begin + (end - begin) / 2;
  const int axis = depth % 3;
  std::nth_element(indices.begin() + begin, indices.begin() + mid, indices.begin() + end,
      [&](auto a, auto b) { return points_[a][axis] < points_[b][axis]; });
  const int id = static_cast<int>(nodes_.size());
  nodes_.push_back({indices[mid], -1, -1, axis});
  const int left = build(indices, begin, mid, depth + 1);
  const int right = build(indices, mid + 1, end, depth + 1);
  nodes_[id].left = left;
  nodes_[id].right = right;
  return id;
}
void KdTree::search(int node, const Eigen::Vector3d &query, double &best,
                    std::size_t &index) const {
  if (node < 0) return;
  const auto &n = nodes_[node];
  const double distance = (points_[n.point] - query).squaredNorm();
  if (distance < best) { best = distance; index = n.point; }
  const double split = query[n.axis] - points_[n.point][n.axis];
  search(split < 0 ? n.left : n.right, query, best, index);
  if (split * split <= best)
    search(split < 0 ? n.right : n.left, query, best, index);
}
bool KdTree::nearest(const Eigen::Vector3d &query, double max_squared_distance,
                     Eigen::Vector3d &point, double &squared_distance) const {
  if (!query.allFinite() || max_squared_distance <= 0) return false;
  squared_distance = max_squared_distance;
  std::size_t index = points_.size();
  search(root_, query, squared_distance, index);
  if (index == points_.size()) return false;
  point = points_[index];
  return true;
}

Registration align(const Cloud &source, const KdTree &target,
                    const Pose &initial, const Options &o) {
  o.validate();
  Registration result;
  result.pose = initial;
  if (source.size() < static_cast<std::size_t>(o.min_correspondences) ||
      !initial.matrix().allFinite()) return result;
  for (int iteration = 0; iteration < o.max_iterations; ++iteration) {
    auto pairs = matches(source, target, result.pose, o.max_correspondence_distance);
    if (pairs.size() < static_cast<std::size_t>(o.min_correspondences)) return result;
    trim(pairs, o);
    double total = 0;
    Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = Eigen::Vector3d::Zero();
    for (const auto &m : pairs) {
      const double w = std::min(1.0, o.huber_delta / std::max(m.distance, 1e-12));
      total += w; a += w * m.source; b += w * m.target;
    }
    a /= total; b /= total;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &m : pairs) {
      const double w = std::min(1.0, o.huber_delta / std::max(m.distance, 1e-12));
      covariance += w * (m.source - a) * (m.target - b).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // A plane is usable; a line or coincident points cannot constrain full 3D rotation.
    if (svd.singularValues()[1] < 1e-9) return result;
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0 ? -1 : 1;
    Pose delta = Pose::Identity();
    delta.linear() = svd.matrixV() * correction * svd.matrixU().transpose();
    delta.translation() = b - delta.rotation() * a;
    result.pose = delta * result.pose;
    result.iterations = iteration + 1;
    if (!result.pose.matrix().allFinite()) return result;
    if (delta.translation().norm() < o.translation_epsilon && angle(delta) < o.rotation_epsilon) {
      result.converged = true;
      break;
    }
  }
  auto pairs = matches(source, target, result.pose, o.max_correspondence_distance);
  result.overlap = static_cast<double>(pairs.size()) / source.size();
  trim(pairs, o);
  result.correspondences = pairs.size();
  if (pairs.empty()) return result;
  double error = 0;
  for (const auto &m : pairs) error += m.distance * m.distance;
  result.rmse = std::sqrt(error / pairs.size());
  const Pose step = initial.inverse() * result.pose;
  result.accepted = result.converged &&
      pairs.size() >= static_cast<std::size_t>(o.min_correspondences) &&
      result.overlap >= o.min_overlap && result.rmse <= o.max_rmse &&
      step.translation().norm() <= o.max_translation_step && angle(step) <= o.max_rotation_step;
  return result;
}

Odometry::Odometry(Options options) : options_(options) { options_.validate(); }
void Odometry::reset() {
  pose_ = Pose::Identity(); keyframes_.clear(); map_.clear();
}
void Odometry::insertKeyframe(const Cloud &scan) {
  Cloud world;
  world.reserve(scan.size());
  for (const auto &p : scan) world.push_back(pose_ * p);
  keyframes_.push_back({pose_, std::move(world)});
  while (keyframes_.size() > static_cast<std::size_t>(options_.max_keyframes))
    keyframes_.pop_front();
  Cloud combined;
  for (const auto &frame : keyframes_)
    combined.insert(combined.end(), frame.points.begin(), frame.points.end());
  map_ = voxelDownsample(combined, options_.voxel_size, options_.max_map_points);
}
Update Odometry::update(const Cloud &cloud) {
  Update result;
  result.pose = pose_;
  const auto scan = voxelDownsample(cloud, options_.voxel_size, options_.max_scan_points);
  result.scan_points = scan.size(); result.map_points = map_.size();
  if (scan.size() < static_cast<std::size_t>(options_.min_correspondences)) return result;
  if (keyframes_.empty()) {
    // Reject a degenerate first scan rather than creating an unusable map.
    const auto check = align(scan, KdTree(scan), Pose::Identity(), options_);
    if (!check.accepted) return result;
    insertKeyframe(scan);
    result.initialized = result.accepted = true;
  } else {
    result.registration = align(scan, KdTree(map_), pose_, options_);
    if (result.registration.accepted) {
      pose_ = result.registration.pose;
      result.accepted = true;
      const Pose delta = keyframes_.back().pose.inverse() * pose_;
      if (delta.translation().norm() >= options_.keyframe_translation ||
          angle(delta) >= options_.keyframe_rotation) insertKeyframe(scan);
    }
  }
  result.pose = pose_; result.map_points = map_.size();
  return result;
}
}  // namespace lidar_odometry
