#include "fruit_picking_arm/kinematic_calibration.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace fruit_picking_arm::calibration
{
namespace
{

template<typename T>
struct Pose
{
  Eigen::Matrix<T, 3, 3> rotation{Eigen::Matrix<T, 3, 3>::Identity()};
  Eigen::Matrix<T, 3, 1> translation{Eigen::Matrix<T, 3, 1>::Zero()};
};

template<typename T>
Pose<T> compose_pose(const Pose<T> & lhs, const Pose<T> & rhs)
{
  return {lhs.rotation * rhs.rotation, lhs.rotation * rhs.translation + lhs.translation};
}

template<typename T>
Pose<T> inverse_pose(const Pose<T> & value)
{
  const auto rotation = value.rotation.transpose().eval();
  return {rotation, -(rotation * value.translation)};
}

Eigen::Matrix3d rpy_rotation(double roll, double pitch, double yaw)
{
  return (
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

template<typename T>
Pose<T> fixed_pose(
  const std::array<double, 3> & xyz,
  const std::array<double, 3> & rpy)
{
  Pose<T> result;
  result.rotation = rpy_rotation(rpy[0], rpy[1], rpy[2]).template cast<T>();
  result.translation = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]).template cast<T>();
  return result;
}

template<typename T>
Eigen::Matrix<T, 3, 3> axis_rotation(const std::array<double, 3> & axis, const T & angle)
{
  const Eigen::Matrix<T, 3, 1> unit =
    Eigen::Vector3d(axis[0], axis[1], axis[2]).normalized().template cast<T>();
  const auto skew = (Eigen::Matrix<T, 3, 3>() <<
    T(0), -unit.z(), unit.y(),
    unit.z(), T(0), -unit.x(),
    -unit.y(), unit.x(), T(0)).finished();
  return Eigen::Matrix<T, 3, 3>::Identity() +
         ceres::sin(angle) * skew + (T(1) - ceres::cos(angle)) * skew * skew;
}

template<typename T>
Pose<T> revolute(
  const std::array<double, 3> & xyz,
  const std::array<double, 3> & rpy,
  const std::array<double, 3> & axis,
  const T & angle)
{
  auto result = fixed_pose<T>(xyz, rpy);
  result.rotation = result.rotation * axis_rotation(axis, angle);
  return result;
}

template<typename T>
Pose<T> forward_model(
  const std::array<double, 6> & joints,
  const T * observable_joint_offsets,
  const T * link_corrections)
{
  static constexpr std::array<std::array<double, 3>, 6> xyz{{
    {{0.0, 0.0, 0.040975}},
    {{-0.048101, 0.056781, 0.094025}},
    {{0.399412, -0.015781, 0.184581}},
    {{-0.041455, -0.032800, 0.057441}},
    {{-0.372174, -0.034450, -0.004890}},
    {{-0.026883, 0.054950, -0.018957}},
  }};
  static constexpr std::array<std::array<double, 3>, 6> rpy{{
    {{0.0, 0.0, 0.0}},
    {{0.0, -1.137899491, 0.0}},
    {{0.0, 1.011527890, 0.0}},
    {{3.141592654, 0.0, 0.0}},
    {{0.0, 0.583631552, 0.0}},
    {{0.0, 0.0, 0.0}},
  }};
  static constexpr std::array<std::array<double, 3>, 6> axes{{
    {{0.0, 0.0, 1.0}}, {{0.0, 1.0, 0.0}}, {{0.0, 1.0, 0.0}},
    {{1.0, 0.0, 0.0}}, {{0.0, 1.0, 0.0}}, {{-0.834467, 0.0, -0.551058}},
  }};

  auto j3_xyz = xyz[2];
  auto j5_xyz = xyz[4];
  const auto j3_norm = std::sqrt(
    j3_xyz[0] * j3_xyz[0] + j3_xyz[1] * j3_xyz[1] + j3_xyz[2] * j3_xyz[2]);
  const auto j5_norm = std::sqrt(
    j5_xyz[0] * j5_xyz[0] + j5_xyz[1] * j5_xyz[1] + j5_xyz[2] * j5_xyz[2]);

  Pose<T> chain = fixed_pose<T>({{0.0, 0.0, 0.0}}, {{0.0, 0.0, 3.141592654}});
  for (std::size_t index = 0; index < 6; ++index) {
    Pose<T> joint;
    if (index == 2) {
      joint = revolute<T>(
        {{j3_xyz[0], j3_xyz[1], j3_xyz[2]}}, rpy[index], axes[index],
        T(joints[index]) + observable_joint_offsets[1]);
      joint.translation += Eigen::Vector3d(
        j3_xyz[0] / j3_norm, j3_xyz[1] / j3_norm, j3_xyz[2] / j3_norm).cast<T>() *
        link_corrections[0];
    } else if (index == 4) {
      joint = revolute<T>(
        {{j5_xyz[0], j5_xyz[1], j5_xyz[2]}}, rpy[index], axes[index],
        T(joints[index]) + observable_joint_offsets[3]);
      joint.translation += Eigen::Vector3d(
        j5_xyz[0] / j5_norm, j5_xyz[1] / j5_norm, j5_xyz[2] / j5_norm).cast<T>() *
        link_corrections[1];
    } else {
      T offset(0.0);
      if (index == 1) {offset = observable_joint_offsets[0];}
      if (index == 3) {offset = observable_joint_offsets[2];}
      // J1 and J6 are gauge freedoms when both camera and board mounts are unknown.
      joint = revolute<T>(xyz[index], rpy[index], axes[index], T(joints[index]) + offset);
    }
    chain = compose_pose(chain, joint);
  }
  return compose_pose(
    chain, fixed_pose<T>({{-0.058148, 0.0, -0.033366}}, {{0.0, 1.57079632679, 0.0}}));
}

template<typename T>
Pose<T> pose_from_parameters(const T * values)
{
  Pose<T> result;
  ceres::AngleAxisToRotationMatrix(values, result.rotation.data());
  result.translation = Eigen::Map<const Eigen::Matrix<T, 3, 1>>(values + 3);
  return result;
}

Pose<double> eigen_pose(const Transform3d & value)
{
  Pose<double> result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.rotation(row, column) = value.rotation(row, column);
    }
    result.translation(row) = value.translation[row];
  }
  return result;
}

Transform3d cv_pose(const Pose<double> & value)
{
  Transform3d result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.rotation(row, column) = value.rotation(row, column);
    }
    result.translation[row] = value.translation(row);
  }
  return result;
}

std::array<double, 6> pose_parameters(const Transform3d & value)
{
  cv::Vec3d angle_axis;
  cv::Rodrigues(cv::Mat(value.rotation), angle_axis);
  return {{
    angle_axis[0], angle_axis[1], angle_axis[2],
    value.translation[0], value.translation[1], value.translation[2]}};
}

struct ObservationResidual
{
  ObservationResidual(
    const KinematicSample & sample,
    double translation_sigma,
    double rotation_sigma)
  : sample_(sample),
    observed_(eigen_pose(sample.camera_T_board)),
    translation_sigma_(translation_sigma),
    rotation_sigma_(rotation_sigma) {}

  template<typename T>
  bool operator()(
    const T * observable_offsets,
    const T * link_corrections,
    const T * base_camera,
    const T * tool_board,
    T * residuals) const
  {
    const auto base_T_tool = forward_model<T>(
      sample_.joints, observable_offsets, link_corrections);
    const auto predicted = compose_pose(
      compose_pose(inverse_pose(pose_from_parameters(base_camera)), base_T_tool),
      pose_from_parameters(tool_board));
    for (int index = 0; index < 3; ++index) {
      residuals[index] =
        (predicted.translation(index) - T(observed_.translation(index))) /
        T(translation_sigma_);
    }
    const Eigen::Matrix<T, 3, 3> relative =
      observed_.rotation.cast<T>().transpose() * predicted.rotation;
    T angle_axis[3];
    ceres::RotationMatrixToAngleAxis(relative.data(), angle_axis);
    for (int index = 0; index < 3; ++index) {
      residuals[index + 3] = angle_axis[index] / T(rotation_sigma_);
    }
    return true;
  }

  KinematicSample sample_;
  Pose<double> observed_;
  double translation_sigma_;
  double rotation_sigma_;
};

Pose<double> predict_camera_T_board(
  const KinematicSample & sample,
  const double * offsets,
  const double * links,
  const double * base_camera,
  const double * tool_board)
{
  return compose_pose(
    compose_pose(
      inverse_pose(pose_from_parameters(base_camera)),
      forward_model<double>(sample.joints, offsets, links)),
    pose_from_parameters(tool_board));
}

double percentile95(std::vector<double> values)
{
  if (values.empty()) {return 0.0;}
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::ceil(0.95 * static_cast<double>(values.size()))) - 1U;
  return values[std::min(index, values.size() - 1U)];
}

double rotation_error_deg(
  const Eigen::Matrix3d & expected,
  const Eigen::Matrix3d & actual)
{
  const auto cosine = std::clamp(
    ((expected.transpose() * actual).trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine) * 180.0 / M_PI;
}

ErrorMetrics metrics_for(
  const std::vector<KinematicSample> & samples,
  const double * offsets,
  const double * links,
  const double * base_camera,
  const double * tool_board)
{
  ErrorMetrics metrics;
  if (samples.empty()) {return metrics;}
  std::vector<double> positions;
  std::vector<double> rotations;
  double position_squared = 0.0;
  double rotation_squared = 0.0;
  for (const auto & sample : samples) {
    const auto predicted = predict_camera_T_board(
      sample, offsets, links, base_camera, tool_board);
    const auto observed = eigen_pose(sample.camera_T_board);
    const auto position = (predicted.translation - observed.translation).norm();
    const auto rotation = rotation_error_deg(observed.rotation, predicted.rotation);
    positions.push_back(position);
    rotations.push_back(rotation);
    position_squared += position * position;
    rotation_squared += rotation * rotation;
    metrics.position_max_m = std::max(metrics.position_max_m, position);
    metrics.rotation_max_deg = std::max(metrics.rotation_max_deg, rotation);
  }
  metrics.position_rms_m = std::sqrt(position_squared / samples.size());
  metrics.rotation_rms_deg = std::sqrt(rotation_squared / samples.size());
  metrics.position_p95_m = percentile95(positions);
  metrics.rotation_p95_deg = percentile95(rotations);
  return metrics;
}

void check_pose_coverage(
  const std::vector<KinematicSample> & samples,
  double minimum_span)
{
  for (std::size_t joint = 1; joint <= 4; ++joint) {
    auto limits = std::minmax_element(
      samples.begin(), samples.end(),
      [joint](const auto & lhs, const auto & rhs) {
        return lhs.joints[joint] < rhs.joints[joint];
      });
    if (limits.second->joints[joint] - limits.first->joints[joint] < minimum_span) {
      throw std::runtime_error(
              "joint_" + std::to_string(joint + 1) +
              " span is too small for observable calibration");
    }
  }
}

double jacobian_condition_number(ceres::Problem & problem)
{
  ceres::Problem::EvaluateOptions options;
  ceres::CRSMatrix sparse;
  double cost = 0.0;
  std::vector<double> residuals;
  if (!problem.Evaluate(options, &cost, &residuals, nullptr, &sparse) ||
    sparse.num_cols == 0 || sparse.num_rows < sparse.num_cols)
  {
    return std::numeric_limits<double>::infinity();
  }
  Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(sparse.num_rows, sparse.num_cols);
  for (int row = 0; row < sparse.num_rows; ++row) {
    for (int entry = sparse.rows[row]; entry < sparse.rows[row + 1]; ++entry) {
      dense(row, sparse.cols[entry]) = sparse.values[entry];
    }
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(dense, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.singularValues().size() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  const auto largest = svd.singularValues()(0);
  const auto smallest = svd.singularValues()(svd.singularValues().size() - 1);
  if (smallest <= largest * 1.0e-12) {
    return std::numeric_limits<double>::infinity();
  }
  return largest / smallest;
}

YAML::Node transform_node(const Transform3d & transform)
{
  const auto parameters = pose_parameters(transform);
  YAML::Node node;
  node["angle_axis"] = std::vector<double>(parameters.begin(), parameters.begin() + 3);
  node["translation_m"] = std::vector<double>(parameters.begin() + 3, parameters.end());
  return node;
}

YAML::Node metrics_node(const ErrorMetrics & metrics)
{
  YAML::Node node;
  node["position_rms_m"] = metrics.position_rms_m;
  node["position_p95_m"] = metrics.position_p95_m;
  node["position_max_m"] = metrics.position_max_m;
  node["rotation_rms_deg"] = metrics.rotation_rms_deg;
  node["rotation_p95_deg"] = metrics.rotation_p95_deg;
  node["rotation_max_deg"] = metrics.rotation_max_deg;
  return node;
}

}  // namespace

Transform3d nominal_forward_kinematics(
  const std::array<double, 6> & joints,
  const std::array<double, 6> & joint_offsets,
  double j2_j3_length_correction_m,
  double j4_j5_length_correction_m)
{
  const std::array<double, 4> observable{{
    joint_offsets[1], joint_offsets[2], joint_offsets[3], joint_offsets[4]}};
  const std::array<double, 2> links{{
    j2_j3_length_correction_m, j4_j5_length_correction_m}};
  return cv_pose(forward_model<double>(joints, observable.data(), links.data()));
}

std::pair<std::vector<KinematicSample>, std::vector<KinematicSample>> split_samples(
  const std::vector<KinematicSample> & samples)
{
  std::vector<KinematicSample> training;
  std::vector<KinematicSample> validation;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (index % 4U == 3U) {
      validation.push_back(samples[index]);
    } else {
      training.push_back(samples[index]);
    }
  }
  return {training, validation};
}

KinematicCalibrationResult solve_kinematic_calibration(
  const std::vector<KinematicSample> & samples,
  const KinematicCalibrationOptions & options)
{
  const auto [training, validation] = split_samples(samples);
  if (training.size() < options.minimum_training_samples ||
    validation.size() < options.minimum_validation_samples)
  {
    throw std::runtime_error(
            "kinematic calibration requires at least " +
            std::to_string(options.minimum_training_samples) + " training and " +
            std::to_string(options.minimum_validation_samples) + " validation samples");
  }
  check_pose_coverage(training, options.minimum_joint_span_rad);

  std::vector<EyeToHandSample> initial_samples;
  initial_samples.reserve(training.size());
  for (const auto & sample : training) {
    initial_samples.push_back({nominal_forward_kinematics(sample.joints), sample.camera_T_board});
  }
  const auto initial = solve_eye_to_hand(initial_samples, options.minimum_training_samples);
  auto base_camera = pose_parameters(initial.base_T_camera);
  auto tool_board = pose_parameters(initial.tool_T_target);
  std::array<double, 4> offsets{};
  std::array<double, 2> links{};

  ceres::Problem problem;
  for (const auto & sample : training) {
    auto * cost = new ceres::AutoDiffCostFunction<ObservationResidual, 6, 4, 2, 6, 6>(
      new ObservationResidual(
        sample, options.translation_sigma_m, options.rotation_sigma_rad));
    problem.AddResidualBlock(
      cost, new ceres::HuberLoss(options.robust_loss_scale),
      offsets.data(), links.data(), base_camera.data(), tool_board.data());
  }
  for (int index = 0; index < 4; ++index) {
    problem.SetParameterLowerBound(offsets.data(), index, -options.maximum_joint_offset_rad);
    problem.SetParameterUpperBound(offsets.data(), index, options.maximum_joint_offset_rad);
  }
  for (int index = 0; index < 2; ++index) {
    problem.SetParameterLowerBound(links.data(), index, -options.maximum_link_correction_m);
    problem.SetParameterUpperBound(links.data(), index, options.maximum_link_correction_m);
  }

  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.max_num_iterations = 200;
  solver_options.function_tolerance = 1.0e-12;
  solver_options.gradient_tolerance = 1.0e-12;
  solver_options.parameter_tolerance = 1.0e-12;
  solver_options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;

  // Stage 1: refine camera and board mount from nominal kinematics.
  problem.SetParameterBlockConstant(offsets.data());
  problem.SetParameterBlockConstant(links.data());
  ceres::Solve(solver_options, &problem, &summary);
  if (!summary.IsSolutionUsable()) {
    throw std::runtime_error("extrinsic initialization failed: " + summary.BriefReport());
  }
  // Stage 2: identify observable J2-J5 encoder offsets.
  problem.SetParameterBlockVariable(offsets.data());
  ceres::Solve(solver_options, &problem, &summary);
  if (!summary.IsSolutionUsable()) {
    throw std::runtime_error("joint-offset calibration failed: " + summary.BriefReport());
  }
  // Stage 3: release the two dominant link-length corrections and jointly refine.
  problem.SetParameterBlockVariable(links.data());
  ceres::Solve(solver_options, &problem, &summary);
  if (!summary.IsSolutionUsable()) {
    throw std::runtime_error("joint/link calibration failed: " + summary.BriefReport());
  }

  KinematicCalibrationResult result;
  result.joint_offsets_rad = {{0.0, offsets[0], offsets[1], offsets[2], offsets[3], 0.0}};
  result.j2_j3_length_correction_m = links[0];
  result.j4_j5_length_correction_m = links[1];
  result.base_T_camera = cv_pose(pose_from_parameters(base_camera.data()));
  result.tool_T_board = cv_pose(pose_from_parameters(tool_board.data()));
  result.training_metrics = metrics_for(
    training, offsets.data(), links.data(), base_camera.data(), tool_board.data());
  result.validation_metrics = metrics_for(
    validation, offsets.data(), links.data(), base_camera.data(), tool_board.data());
  result.training_samples = training.size();
  result.validation_samples = validation.size();
  result.jacobian_condition_number = jacobian_condition_number(problem);
  result.observable = std::isfinite(result.jacobian_condition_number) &&
    result.jacobian_condition_number <= options.maximum_condition_number;
  result.validated = result.observable &&
    result.validation_metrics.position_p95_m <= options.validation_position_p95_m &&
    result.validation_metrics.rotation_p95_deg <= options.validation_rotation_p95_deg;
  result.solver_summary = summary.BriefReport();
  return result;
}

void write_kinematic_profile(
  const std::string & destination,
  const std::string & robot_serial,
  const KinematicCalibrationResult & result,
  const std::string & tcp_source,
  const Transform3d & flange_T_tcp)
{
  if (robot_serial.empty()) {
    throw std::invalid_argument("robot_serial must not be empty");
  }
  YAML::Node root;
  root["schema_version"] = 1;
  root["robot_serial"] = robot_serial;
  root["nominal_model_hash"] = kNominalModelHash;
  root["validated"] = result.validated;
  root["observable"] = result.observable;
  root["joint_offsets_rad"] = std::vector<double>(
    result.joint_offsets_rad.begin(), result.joint_offsets_rad.end());
  root["j2_j3_length_correction_m"] = result.j2_j3_length_correction_m;
  root["j4_j5_length_correction_m"] = result.j4_j5_length_correction_m;
  root["base_T_camera"] = transform_node(result.base_T_camera);
  root["tool_T_board"] = transform_node(result.tool_T_board);
  root["tcp_source"] = tcp_source;
  root["flange_T_tcp"] = transform_node(flange_T_tcp);
  root["training_samples"] = result.training_samples;
  root["validation_samples"] = result.validation_samples;
  root["jacobian_condition_number"] = result.jacobian_condition_number;
  root["training_metrics"] = metrics_node(result.training_metrics);
  root["validation_metrics"] = metrics_node(result.validation_metrics);
  root["solver_summary"] = result.solver_summary;
  root["gauge_constraints"] =
    "J1 and J6 offsets fixed at mechanical zero; external camera and board mount are unknown";

  const auto path = std::filesystem::path(destination);
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  const auto temporary = path.string() + ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("cannot open calibration profile: " + temporary);
    }
    output << std::setprecision(15) << root;
    if (!output.good()) {
      throw std::runtime_error("failed to write calibration profile: " + temporary);
    }
  }
  std::filesystem::rename(temporary, path);
}

}  // namespace fruit_picking_arm::calibration
