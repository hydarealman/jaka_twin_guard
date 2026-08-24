#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace fruit_picking_arm::calibration
{

// Transform convention: parent_T_child maps coordinates expressed in child
// into parent. For example base_T_tool is the transform returned by
// lookupTransform("base_link", "tool_flange", ...).
struct Transform3d
{
  cv::Matx33d rotation{cv::Matx33d::eye()};
  cv::Vec3d translation{0.0, 0.0, 0.0};
};

struct EyeToHandSample
{
  Transform3d base_T_tool;
  Transform3d camera_T_target;
};

struct CalibrationMetrics
{
  double translation_rms_m{0.0};
  double translation_max_m{0.0};
  double rotation_rms_deg{0.0};
  double rotation_max_deg{0.0};
  double robot_rotation_span_deg{0.0};
};

struct EyeToHandResult
{
  Transform3d base_T_camera;
  Transform3d tool_T_target;
  CalibrationMetrics metrics;
  std::string method;
};

Transform3d compose(const Transform3d & lhs, const Transform3d & rhs);
Transform3d inverse(const Transform3d & value);
double rotation_distance_deg(const cv::Matx33d & lhs, const cv::Matx33d & rhs);

// Fixed external camera, target rigidly mounted to the robot tool.
// Internally this uses OpenCV calibrateHandEye with inverse robot poses, which
// returns base_T_camera for the eye-to-hand frame arrangement.
EyeToHandResult solve_eye_to_hand(
  const std::vector<EyeToHandSample> & samples,
  std::size_t minimum_samples = 8);

}  // namespace fruit_picking_arm::calibration
