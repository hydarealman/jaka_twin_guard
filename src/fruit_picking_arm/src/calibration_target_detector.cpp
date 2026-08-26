#include "fruit_picking_arm/calibration_target_detector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace fruit_picking_arm::calibration
{
namespace
{

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_from_name(const std::string & name)
{
  if (name == "DICT_4X4_50") {return cv::aruco::DICT_4X4_50;}
  if (name == "DICT_4X4_100") {return cv::aruco::DICT_4X4_100;}
  if (name == "DICT_5X5_50") {return cv::aruco::DICT_5X5_50;}
  if (name == "DICT_5X5_100") {return cv::aruco::DICT_5X5_100;}
  if (name == "DICT_5X5_250") {return cv::aruco::DICT_5X5_250;}
  if (name == "DICT_6X6_250") {return cv::aruco::DICT_6X6_250;}
  throw std::invalid_argument("unsupported ArUco dictionary: " + name);
}

Transform3d transform_from_pose(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  Transform3d result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.rotation(row, column) = rotation.at<double>(row, column);
    }
  }
  result.translation = tvec;
  return result;
}

double reprojection_error(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  const cv::Vec3d & rvec,
  const cv::Vec3d & tvec,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_points, rvec, tvec, camera_matrix, distortion, projected);
  double squared = 0.0;
  for (std::size_t index = 0; index < projected.size(); ++index) {
    const auto error = projected[index] - image_points[index];
    squared += error.dot(error);
  }
  return projected.empty() ? 0.0 : std::sqrt(squared / projected.size());
}

}  // namespace

CalibrationTargetDetector::CalibrationTargetDetector(CalibrationTargetSettings settings)
: settings_(std::move(settings))
{
  if (settings_.target_type != "chessboard" && settings_.target_type != "charuco") {
    throw std::invalid_argument("target_type must be 'chessboard' or 'charuco'");
  }
  if (settings_.square_length_m <= 0.0 || settings_.chessboard_corners_x < 2 ||
    settings_.chessboard_corners_y < 2)
  {
    throw std::invalid_argument("invalid calibration target dimensions");
  }
  dictionary_ = cv::aruco::getPredefinedDictionary(
    dictionary_from_name(settings_.dictionary));
  charuco_board_ = cv::aruco::CharucoBoard::create(
    settings_.charuco_squares_x, settings_.charuco_squares_y,
    static_cast<float>(settings_.square_length_m),
    static_cast<float>(settings_.marker_length_m), dictionary_);
}

std::optional<CalibrationTargetDetection> CalibrationTargetDetector::detect(
  const cv::Mat & image,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion) const
{
  if (image.empty() || camera_matrix.empty()) {return std::nullopt;}
  cv::Mat annotated = image.clone();
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point3f> object_points;

  if (settings_.target_type == "chessboard") {
    const cv::Size pattern(
      settings_.chessboard_corners_x, settings_.chessboard_corners_y);
    cv::Mat grayscale;
    if (image.channels() == 1) {
      grayscale = image;
    } else {
      cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
    }
    if (!cv::findChessboardCorners(
        grayscale, pattern, image_points,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE))
    {
      return std::nullopt;
    }
    cv::cornerSubPix(
      grayscale, image_points, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));
    for (int row = 0; row < pattern.height; ++row) {
      for (int column = 0; column < pattern.width; ++column) {
        object_points.emplace_back(
          static_cast<float>(column * settings_.square_length_m),
          static_cast<float>(row * settings_.square_length_m), 0.0F);
      }
    }
    cv::drawChessboardCorners(annotated, pattern, image_points, true);
    if (!cv::solvePnP(
        object_points, image_points, camera_matrix, distortion,
        rvec, tvec, false, cv::SOLVEPNP_IPPE))
    {
      return std::nullopt;
    }
  } else {
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids);
    if (marker_ids.empty()) {return std::nullopt;}
    cv::Mat charuco_corners;
    cv::Mat charuco_ids;
    cv::aruco::interpolateCornersCharuco(
      marker_corners, marker_ids, image, charuco_board_,
      charuco_corners, charuco_ids, camera_matrix, distortion);
    if (charuco_ids.total() < static_cast<std::size_t>(settings_.minimum_detected_corners)) {
      return std::nullopt;
    }
    if (!cv::aruco::estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, charuco_board_, camera_matrix,
        distortion, rvec, tvec))
    {
      return std::nullopt;
    }
    cv::aruco::drawDetectedMarkers(annotated, marker_corners, marker_ids);
    cv::aruco::drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids);
    image_points.reserve(charuco_ids.total());
    object_points.reserve(charuco_ids.total());
    for (int index = 0; index < charuco_ids.rows; ++index) {
      const auto id = charuco_ids.at<int>(index);
      image_points.push_back(charuco_corners.at<cv::Point2f>(index));
      object_points.push_back(charuco_board_->chessboardCorners[id]);
    }
  }

  cv::drawFrameAxes(
    annotated, camera_matrix, distortion, rvec, tvec,
    static_cast<float>(settings_.square_length_m * 2.0));
  return CalibrationTargetDetection{
    transform_from_pose(rvec, tvec), static_cast<int>(image_points.size()),
    reprojection_error(
      object_points, image_points, rvec, tvec, camera_matrix, distortion),
    annotated};
}

}  // namespace fruit_picking_arm::calibration
