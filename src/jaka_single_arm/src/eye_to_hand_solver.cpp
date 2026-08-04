#include "jaka_single_arm/eye_to_hand_solver.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace jaka_single_arm::calibration
{
namespace
{

cv::Mat to_mat(const cv::Matx33d & value)
{
  return cv::Mat(value).clone();
}

cv::Mat to_column(const cv::Vec3d & value)
{
  return (cv::Mat_<double>(3, 1) << value[0], value[1], value[2]);
}

cv::Matx33d to_rotation(const cv::Mat & value)
{
  cv::Mat converted;
  value.convertTo(converted, CV_64F);
  if (converted.rows != 3 || converted.cols != 3) {
    throw std::runtime_error("calibration solver returned an invalid rotation");
  }
  cv::Matx33d output;
  std::memcpy(output.val, converted.ptr<double>(), 9 * sizeof(double));
  return output;
}

cv::Vec3d to_translation(const cv::Mat & value)
{
  cv::Mat converted;
  value.reshape(1, 3).convertTo(converted, CV_64F);
  return {converted.at<double>(0), converted.at<double>(1), converted.at<double>(2)};
}

bool finite(const Transform3d & value)
{
  return cv::checkRange(cv::Mat(value.rotation)) &&
         cv::checkRange(cv::Mat(value.translation));
}

cv::Matx33d mean_rotation(const std::vector<Transform3d> & values)
{
  cv::Mat accumulated = cv::Mat::zeros(3, 3, CV_64F);
  for (const auto & value : values) {
    accumulated += cv::Mat(value.rotation);
  }
  cv::SVD svd(accumulated, cv::SVD::FULL_UV);
  cv::Mat correction = cv::Mat::eye(3, 3, CV_64F);
  if (cv::determinant(svd.u * svd.vt) < 0.0) {
    correction.at<double>(2, 2) = -1.0;
  }
  return to_rotation(svd.u * correction * svd.vt);
}

CalibrationMetrics evaluate(
  const std::vector<EyeToHandSample> & samples,
  const Transform3d & base_T_camera,
  Transform3d & tool_T_target)
{
  std::vector<Transform3d> estimates;
  estimates.reserve(samples.size());
  for (const auto & sample : samples) {
    estimates.push_back(compose(
      compose(inverse(sample.base_T_tool), base_T_camera),
      sample.camera_T_target));
  }
  cv::Vec3d mean_translation{0.0, 0.0, 0.0};
  for (const auto & estimate : estimates) {
    mean_translation += estimate.translation;
  }
  mean_translation *= 1.0 / static_cast<double>(estimates.size());
  tool_T_target = {mean_rotation(estimates), mean_translation};

  CalibrationMetrics metrics;
  double translation_squared = 0.0;
  double rotation_squared = 0.0;
  for (const auto & estimate : estimates) {
    const auto translation_error = cv::norm(
      estimate.translation - mean_translation);
    const auto rotation_error = rotation_distance_deg(
      estimate.rotation, tool_T_target.rotation);
    translation_squared += translation_error * translation_error;
    rotation_squared += rotation_error * rotation_error;
    metrics.translation_max_m = std::max(
      metrics.translation_max_m, translation_error);
    metrics.rotation_max_deg = std::max(metrics.rotation_max_deg, rotation_error);
  }
  metrics.translation_rms_m = std::sqrt(
    translation_squared / static_cast<double>(estimates.size()));
  metrics.rotation_rms_deg = std::sqrt(
    rotation_squared / static_cast<double>(estimates.size()));

  for (std::size_t first = 0; first < samples.size(); ++first) {
    for (std::size_t second = first + 1; second < samples.size(); ++second) {
      metrics.robot_rotation_span_deg = std::max(
        metrics.robot_rotation_span_deg,
        rotation_distance_deg(
          samples[first].base_T_tool.rotation,
          samples[second].base_T_tool.rotation));
    }
  }
  return metrics;
}

struct Method
{
  const char * name;
  cv::HandEyeCalibrationMethod value;
};

}  // namespace

Transform3d compose(const Transform3d & lhs, const Transform3d & rhs)
{
  return {
    lhs.rotation * rhs.rotation,
    lhs.rotation * rhs.translation + lhs.translation};
}

Transform3d inverse(const Transform3d & value)
{
  const auto transposed = value.rotation.t();
  return {transposed, -(transposed * value.translation)};
}

double rotation_distance_deg(const cv::Matx33d & lhs, const cv::Matx33d & rhs)
{
  const auto relative = lhs.t() * rhs;
  const auto cosine = std::clamp((cv::trace(cv::Mat(relative))[0] - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine) * 180.0 / CV_PI;
}

EyeToHandResult solve_eye_to_hand(
  const std::vector<EyeToHandSample> & samples,
  std::size_t minimum_samples)
{
  if (samples.size() < minimum_samples) {
    throw std::runtime_error(
            "eye-to-hand calibration requires at least " +
            std::to_string(minimum_samples) + " samples");
  }
  for (const auto & sample : samples) {
    if (!finite(sample.base_T_tool) || !finite(sample.camera_T_target)) {
      throw std::runtime_error("calibration samples contain non-finite transforms");
    }
  }

  std::vector<cv::Mat> robot_inverse_rotations;
  std::vector<cv::Mat> robot_inverse_translations;
  std::vector<cv::Mat> target_to_camera_rotations;
  std::vector<cv::Mat> target_to_camera_translations;
  for (const auto & sample : samples) {
    // OpenCV's eye-in-hand API becomes eye-to-hand when the robot poses are
    // inverted. Synthetic ground-truth tests protect this frame convention.
    const auto tool_T_base = inverse(sample.base_T_tool);
    robot_inverse_rotations.push_back(to_mat(tool_T_base.rotation));
    robot_inverse_translations.push_back(to_column(tool_T_base.translation));
    target_to_camera_rotations.push_back(to_mat(sample.camera_T_target.rotation));
    target_to_camera_translations.push_back(to_column(sample.camera_T_target.translation));
  }

  static constexpr std::array<Method, 5> methods{{
    {"TSAI", cv::CALIB_HAND_EYE_TSAI},
    {"PARK", cv::CALIB_HAND_EYE_PARK},
    {"HORAUD", cv::CALIB_HAND_EYE_HORAUD},
    {"ANDREFF", cv::CALIB_HAND_EYE_ANDREFF},
    {"DANIILIDIS", cv::CALIB_HAND_EYE_DANIILIDIS},
  }};

  EyeToHandResult best;
  double best_score = std::numeric_limits<double>::infinity();
  std::string errors;
  for (const auto & method : methods) {
    try {
      cv::Mat rotation;
      cv::Mat translation;
      cv::calibrateHandEye(
        robot_inverse_rotations, robot_inverse_translations,
        target_to_camera_rotations, target_to_camera_translations,
        rotation, translation, method.value);
      const Transform3d base_T_camera{
        to_rotation(rotation), to_translation(translation)};
      if (!finite(base_T_camera) || std::abs(cv::determinant(cv::Mat(base_T_camera.rotation)) - 1.0) > 1e-3) {
        throw std::runtime_error("non-rigid solution");
      }
      Transform3d tool_T_target;
      const auto metrics = evaluate(
        samples, base_T_camera, tool_T_target);
      // Treat one degree of rotation scatter like 10 mm of translation
      // scatter when selecting among OpenCV's independent solvers.
      const auto score = metrics.translation_rms_m +
        metrics.rotation_rms_deg * 0.01;
      if (score < best_score) {
        best_score = score;
        best = {base_T_camera, tool_T_target, metrics, method.name};
      }
    } catch (const cv::Exception & error) {
      errors += std::string(method.name) + ": " + error.what() + "; ";
    } catch (const std::exception & error) {
      errors += std::string(method.name) + ": " + error.what() + "; ";
    }
  }
  if (!std::isfinite(best_score)) {
    throw std::runtime_error("all OpenCV hand-eye solvers failed: " + errors);
  }
  if (best.metrics.robot_rotation_span_deg < 15.0) {
    throw std::runtime_error(
            "robot pose rotation span is below 15 degrees; collect more diverse poses");
  }
  return best;
}

}  // namespace jaka_single_arm::calibration
