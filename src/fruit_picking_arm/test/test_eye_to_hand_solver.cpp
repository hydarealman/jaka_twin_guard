#include "fruit_picking_arm/eye_to_hand_solver.hpp"

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace calibration = fruit_picking_arm::calibration;

namespace
{

calibration::Transform3d make_transform(
  const cv::Vec3d & axis_angle, const cv::Vec3d & translation)
{
  cv::Mat rotation;
  cv::Rodrigues(axis_angle, rotation);
  cv::Matx33d rotation_value;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation_value(row, column) = rotation.at<double>(row, column);
    }
  }
  return {rotation_value, translation};
}

std::vector<calibration::EyeToHandSample> exact_samples()
{
  const auto base_T_camera = make_transform(
    cv::Vec3d(0.22, -0.15, 0.08), cv::Vec3d(0.82, -0.31, 1.24));
  const auto tool_T_target = make_transform(
    cv::Vec3d(-0.10, 0.21, 0.07), cv::Vec3d(0.035, -0.018, 0.115));
  const std::vector<cv::Vec3d> rotations{
    {0.05, 0.10, -0.04}, {-0.25, 0.15, 0.05}, {0.32, -0.12, 0.18},
    {-0.18, -0.28, 0.14}, {0.12, 0.31, -0.22}, {-0.38, 0.06, -0.11},
    {0.26, 0.27, 0.09}, {-0.08, -0.35, -0.19}, {0.41, -0.17, 0.24},
    {-0.29, 0.24, -0.26}, {0.16, -0.09, 0.38}, {-0.34, -0.19, 0.31},
  };
  std::vector<calibration::EyeToHandSample> samples;
  for (std::size_t index = 0; index < rotations.size(); ++index) {
    const double value = static_cast<double>(index);
    const auto base_T_tool = make_transform(
      rotations[index],
      cv::Vec3d(0.35 + 0.018 * value, -0.20 + 0.011 * value,
      0.62 + 0.007 * std::sin(value)));
    // base_T_tool * tool_T_target = base_T_camera * camera_T_target
    const auto camera_T_target = calibration::compose(
      calibration::compose(calibration::inverse(base_T_camera), base_T_tool),
      tool_T_target);
    samples.push_back({base_T_tool, camera_T_target});
  }
  return samples;
}

}  // namespace

TEST(EyeToHandSolver, RecoversBaseToCameraFrameDirection)
{
  const auto expected = make_transform(
    cv::Vec3d(0.22, -0.15, 0.08), cv::Vec3d(0.82, -0.31, 1.24));
  const auto result = calibration::solve_eye_to_hand(exact_samples(), 10);
  EXPECT_LT(cv::norm(result.base_T_camera.translation - expected.translation), 1e-7);
  EXPECT_LT(
    calibration::rotation_distance_deg(
      result.base_T_camera.rotation, expected.rotation),
    1e-5);
  EXPECT_LT(result.metrics.translation_rms_m, 1e-7);
  EXPECT_LT(result.metrics.rotation_rms_deg, 1e-5);
  EXPECT_GT(result.metrics.robot_rotation_span_deg, 15.0);
}

TEST(EyeToHandSolver, RejectsTooFewSamples)
{
  auto samples = exact_samples();
  samples.resize(4);
  EXPECT_THROW(calibration::solve_eye_to_hand(samples, 8), std::runtime_error);
}

TEST(EyeToHandSolver, RejectsInsufficientRotationDiversity)
{
  std::vector<calibration::EyeToHandSample> samples;
  const auto base_T_camera = make_transform({}, {0.8, 0.0, 1.0});
  const auto tool_T_target = make_transform({}, {0.0, 0.0, 0.1});
  for (int index = 0; index < 10; ++index) {
    const auto base_T_tool = make_transform(
      cv::Vec3d(0.0, 0.0, index * 0.002),
      cv::Vec3d(0.3 + index * 0.01, 0.0, 0.6));
    const auto camera_T_target = calibration::compose(
      calibration::compose(calibration::inverse(base_T_camera), base_T_tool),
      tool_T_target);
    samples.push_back({base_T_tool, camera_T_target});
  }
  EXPECT_THROW(calibration::solve_eye_to_hand(samples, 8), std::runtime_error);
}
