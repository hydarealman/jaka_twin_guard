#pragma once

#include "fruit_picking_arm/eye_to_hand_solver.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>

#include <optional>
#include <string>

namespace fruit_picking_arm::calibration
{

struct CalibrationTargetSettings
{
  std::string target_type{"chessboard"};
  std::string dictionary{"DICT_5X5_250"};
  int charuco_squares_x{7};
  int charuco_squares_y{5};
  int chessboard_corners_x{9};
  int chessboard_corners_y{6};
  double square_length_m{0.030};
  double marker_length_m{0.022};
  int minimum_detected_corners{8};
};

struct CalibrationTargetDetection
{
  Transform3d camera_T_target;
  int corner_count{0};
  double reprojection_error_px{0.0};
  cv::Mat annotated_image;
};

class CalibrationTargetDetector
{
public:
  explicit CalibrationTargetDetector(CalibrationTargetSettings settings);

  std::optional<CalibrationTargetDetection> detect(
    const cv::Mat & image,
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion) const;

private:
  CalibrationTargetSettings settings_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;
};

}  // namespace fruit_picking_arm::calibration
