#pragma once

#include "fruit_picking_arm/eye_to_hand_solver.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace fruit_picking_arm::calibration
{

inline constexpr char kNominalModelHash[] = "fruit-arm-kinematics-v1";

struct KinematicSample
{
  std::array<double, 6> joints{};
  Transform3d camera_T_board;
  double reprojection_error_px{0.0};
};

struct KinematicCalibrationOptions
{
  std::size_t minimum_training_samples{30};
  std::size_t minimum_validation_samples{10};
  double translation_sigma_m{0.003};
  double rotation_sigma_rad{0.008726646259971648};
  double robust_loss_scale{2.0};
  double maximum_joint_offset_rad{0.15};
  double maximum_link_correction_m{0.03};
  double minimum_joint_span_rad{0.15};
  double maximum_condition_number{1.0e10};
  double validation_position_p95_m{0.005};
  double validation_rotation_p95_deg{1.0};
};

struct ErrorMetrics
{
  double position_rms_m{0.0};
  double position_p95_m{0.0};
  double position_max_m{0.0};
  double rotation_rms_deg{0.0};
  double rotation_p95_deg{0.0};
  double rotation_max_deg{0.0};
};

struct KinematicCalibrationResult
{
  std::array<double, 6> joint_offsets_rad{};
  double j2_j3_length_correction_m{0.0};
  double j4_j5_length_correction_m{0.0};
  Transform3d base_T_camera;
  Transform3d tool_T_board;
  ErrorMetrics training_metrics;
  ErrorMetrics validation_metrics;
  std::size_t training_samples{0};
  std::size_t validation_samples{0};
  double jacobian_condition_number{0.0};
  bool observable{false};
  bool validated{false};
  std::string solver_summary;
};

Transform3d nominal_forward_kinematics(
  const std::array<double, 6> & joints,
  const std::array<double, 6> & joint_offsets = {},
  double j2_j3_length_correction_m = 0.0,
  double j4_j5_length_correction_m = 0.0);

std::pair<std::vector<KinematicSample>, std::vector<KinematicSample>> split_samples(
  const std::vector<KinematicSample> & samples);

KinematicCalibrationResult solve_kinematic_calibration(
  const std::vector<KinematicSample> & samples,
  const KinematicCalibrationOptions & options = {});

void write_kinematic_profile(
  const std::string & destination,
  const std::string & robot_serial,
  const KinematicCalibrationResult & result,
  const std::string & tcp_source = "nominal",
  const Transform3d & flange_T_tcp = {
    cv::Matx33d::eye(), cv::Vec3d{0.0, 0.0, -0.086}});

}  // namespace fruit_picking_arm::calibration
