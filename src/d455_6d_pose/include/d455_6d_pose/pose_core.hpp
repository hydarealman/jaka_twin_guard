#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace d455_6d_pose
{

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};

  bool valid() const;
  cv::Mat matrix() const;
};

struct HsvRange
{
  cv::Scalar lower{90, 60, 40};
  cv::Scalar upper{135, 255, 255};
};

struct PoseEstimate
{
  bool valid{false};
  bool orientation_complete{false};
  cv::Matx33d rotation{cv::Matx33d::eye()};
  cv::Vec3d translation{0.0, 0.0, 0.0};
  cv::Rect bounding_box;
  std::array<cv::Point2f, 4> image_corners{};
  double confidence{0.0};
  double reprojection_error_px{std::numeric_limits<double>::infinity()};
  double depth_valid_ratio{0.0};
  double depth_residual_m{std::numeric_limits<double>::infinity()};
  std::string status;
};

double depth_at_m(const cv::Mat & depth, int x, int y, double depth_scale);
cv::Vec3d back_project(double u, double v, double depth_m, const CameraIntrinsics & camera);
std::array<cv::Point2f, 4> order_quad(const std::vector<cv::Point2f> & corners);
double rotation_error_deg(const cv::Matx33d & lhs, const cv::Matx33d & rhs);

PoseEstimate estimate_bottle_pose(
  const cv::Mat & bgr,
  const cv::Mat & aligned_depth,
  const CameraIntrinsics & camera,
  const HsvRange & hsv,
  double depth_scale,
  double min_depth_m,
  double max_depth_m,
  double min_contour_area_px,
  double min_aspect_ratio,
  int point_stride = 3);

PoseEstimate estimate_planar_target_pose(
  const cv::Mat & bgr,
  const cv::Mat & aligned_depth,
  const CameraIntrinsics & camera,
  const HsvRange & hsv,
  double target_width_m,
  double target_height_m,
  double depth_scale,
  double min_depth_m,
  double max_depth_m,
  double min_contour_area_px,
  double max_reprojection_error_px,
  double max_depth_residual_m);

PoseEstimate solve_planar_target_pose(
  const std::array<cv::Point2f, 4> & ordered_corners,
  const cv::Mat & aligned_depth,
  const CameraIntrinsics & camera,
  double target_width_m,
  double target_height_m,
  double depth_scale,
  double max_reprojection_error_px,
  double max_depth_residual_m);

}  // namespace d455_6d_pose
