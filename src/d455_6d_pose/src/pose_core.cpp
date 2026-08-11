#include "d455_6d_pose/pose_core.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace d455_6d_pose
{
namespace
{

cv::Mat segment_hsv(const cv::Mat & bgr, const HsvRange & range)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat mask;
  cv::inRange(hsv, range.lower, range.upper, mask);
  const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  return mask;
}

std::optional<std::vector<cv::Point>> largest_contour(
  const cv::Mat & mask, double minimum_area)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  auto best = contours.end();
  double best_area = minimum_area;
  for (auto iterator = contours.begin(); iterator != contours.end(); ++iterator) {
    const auto area = std::abs(cv::contourArea(*iterator));
    if (area > best_area) {
      best_area = area;
      best = iterator;
    }
  }
  if (best == contours.end()) {return std::nullopt;}
  return *best;
}

std::vector<double> valid_depths(
  const cv::Mat & depth, const cv::Mat & mask, const cv::Rect & roi,
  double depth_scale, double minimum, double maximum)
{
  std::vector<double> values;
  const auto clipped = roi & cv::Rect(0, 0, depth.cols, depth.rows);
  for (int y = clipped.y; y < clipped.y + clipped.height; ++y) {
    for (int x = clipped.x; x < clipped.x + clipped.width; ++x) {
      if (!mask.empty() && mask.at<std::uint8_t>(y, x) == 0) {continue;}
      const auto value = depth_at_m(depth, x, y, depth_scale);
      if (std::isfinite(value) && value >= minimum && value <= maximum) {
        values.push_back(value);
      }
    }
  }
  return values;
}

double median(std::vector<double> values)
{
  if (values.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

cv::Matx33d rotation_from_rvec(const cv::Mat & rvec)
{
  cv::Mat matrix;
  cv::Rodrigues(rvec, matrix);
  cv::Mat converted;
  matrix.convertTo(converted, CV_64F);
  cv::Matx33d output;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      output(row, column) = converted.at<double>(row, column);
    }
  }
  return output;
}

cv::Vec3d vector_from_mat(const cv::Mat & value)
{
  cv::Mat converted;
  value.reshape(1, 3).convertTo(converted, CV_64F);
  return {converted.at<double>(0), converted.at<double>(1), converted.at<double>(2)};
}

cv::Matx33d basis_from_axis(cv::Vec3d axis)
{
  const auto length = cv::norm(axis);
  if (length < 1e-9) {throw std::runtime_error("bottle axis is degenerate");}
  axis *= 1.0 / length;
  // Keep the reported bottle +Z axis generally pointing toward image top.
  if (axis[1] > 0.0) {axis = -axis;}
  cv::Vec3d reference{1.0, 0.0, 0.0};
  auto x_axis = reference - axis * reference.dot(axis);
  if (cv::norm(x_axis) < 1e-6) {
    reference = {0.0, 1.0, 0.0};
    x_axis = reference - axis * reference.dot(axis);
  }
  x_axis *= 1.0 / cv::norm(x_axis);
  auto y_axis = axis.cross(x_axis);
  y_axis *= 1.0 / cv::norm(y_axis);
  return {
    x_axis[0], y_axis[0], axis[0],
    x_axis[1], y_axis[1], axis[1],
    x_axis[2], y_axis[2], axis[2]};
}

}  // namespace

bool CameraIntrinsics::valid() const
{
  return fx > 0.0 && fy > 0.0 && width > 0 && height > 0;
}

cv::Mat CameraIntrinsics::matrix() const
{
  return (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
}

double depth_at_m(const cv::Mat & depth, int x, int y, double depth_scale)
{
  if (depth.empty() || x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (depth.type() == CV_16UC1) {
    const auto raw = depth.at<std::uint16_t>(y, x);
    return raw == 0 ? std::numeric_limits<double>::quiet_NaN() : raw * depth_scale;
  }
  if (depth.type() == CV_32FC1) {
    return static_cast<double>(depth.at<float>(y, x));
  }
  if (depth.type() == CV_64FC1) {
    return depth.at<double>(y, x);
  }
  return std::numeric_limits<double>::quiet_NaN();
}

cv::Vec3d back_project(
  double u, double v, double depth_m, const CameraIntrinsics & camera)
{
  if (!camera.valid() || !std::isfinite(depth_m) || depth_m <= 0.0) {
    throw std::invalid_argument("invalid back-projection input");
  }
  return {
    (u - camera.cx) * depth_m / camera.fx,
    (v - camera.cy) * depth_m / camera.fy,
    depth_m};
}

std::array<cv::Point2f, 4> order_quad(const std::vector<cv::Point2f> & corners)
{
  if (corners.size() != 4) {throw std::invalid_argument("quadrilateral requires four corners");}
  cv::Point2f centre{0.0F, 0.0F};
  for (const auto & corner : corners) {centre += corner;}
  centre *= 0.25F;
  std::vector<cv::Point2f> circular = corners;
  std::sort(circular.begin(), circular.end(), [&centre](const auto & lhs, const auto & rhs) {
      return std::atan2(lhs.y - centre.y, lhs.x - centre.x) <
             std::atan2(rhs.y - centre.y, rhs.x - centre.x);
    });
  const auto top_left = std::min_element(
    circular.begin(), circular.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.x + lhs.y < rhs.x + rhs.y;
    });
  std::rotate(circular.begin(), top_left, circular.end());
  // In image coordinates (+Y down), TL->TR->BR has a positive cross product.
  const auto first = circular[1] - circular[0];
  const auto second = circular[2] - circular[1];
  if (first.x * second.y - first.y * second.x < 0.0F) {
    std::reverse(circular.begin() + 1, circular.end());
  }
  std::array<cv::Point2f, 4> ordered;
  std::copy(circular.begin(), circular.end(), ordered.begin());
  return ordered;
}

double rotation_error_deg(const cv::Matx33d & lhs, const cv::Matx33d & rhs)
{
  const auto relative = lhs.t() * rhs;
  const auto cosine = std::clamp((cv::trace(cv::Mat(relative))[0] - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cosine) * 180.0 / CV_PI;
}

PoseEstimate estimate_bottle_pose(
  const cv::Mat & bgr, const cv::Mat & aligned_depth,
  const CameraIntrinsics & camera, const HsvRange & hsv,
  double depth_scale, double min_depth_m, double max_depth_m,
  double min_contour_area_px, double min_aspect_ratio, int point_stride)
{
  PoseEstimate result;
  result.status = "no bottle contour";
  if (bgr.empty() || aligned_depth.empty() || !camera.valid() ||
    bgr.size() != aligned_depth.size())
  {
    result.status = "invalid or unaligned RGB-D input";
    return result;
  }
  const auto mask = segment_hsv(bgr, hsv);
  const auto contour = largest_contour(mask, min_contour_area_px);
  if (!contour) {return result;}
  const auto rectangle = cv::minAreaRect(*contour);
  const auto short_side = std::max(1.0F, std::min(rectangle.size.width, rectangle.size.height));
  const auto long_side = std::max(rectangle.size.width, rectangle.size.height);
  if (long_side / short_side < min_aspect_ratio) {
    result.status = "candidate does not have bottle aspect ratio";
    return result;
  }
  result.bounding_box = cv::boundingRect(*contour);
  cv::Mat contour_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
  std::vector<std::vector<cv::Point>> contours{*contour};
  cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);

  std::vector<cv::Vec3d> points;
  int candidate_pixels = 0;
  int valid_pixels = 0;
  point_stride = std::max(1, point_stride);
  const auto roi = result.bounding_box & cv::Rect(0, 0, bgr.cols, bgr.rows);
  for (int y = roi.y; y < roi.y + roi.height; y += point_stride) {
    for (int x = roi.x; x < roi.x + roi.width; x += point_stride) {
      if (contour_mask.at<std::uint8_t>(y, x) == 0) {continue;}
      ++candidate_pixels;
      const auto depth = depth_at_m(aligned_depth, x, y, depth_scale);
      if (!std::isfinite(depth) || depth < min_depth_m || depth > max_depth_m) {continue;}
      ++valid_pixels;
      points.push_back(back_project(x, y, depth, camera));
    }
  }
  result.depth_valid_ratio = candidate_pixels > 0 ?
    static_cast<double>(valid_pixels) / candidate_pixels : 0.0;
  if (points.size() < 20) {
    result.status = "insufficient valid bottle depth";
    return result;
  }

  cv::Vec3d centroid{0.0, 0.0, 0.0};
  for (const auto & point : points) {centroid += point;}
  centroid *= 1.0 / static_cast<double>(points.size());
  cv::Mat samples(static_cast<int>(points.size()), 3, CV_64F);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto centered = points[index] - centroid;
    samples.at<double>(static_cast<int>(index), 0) = centered[0];
    samples.at<double>(static_cast<int>(index), 1) = centered[1];
    samples.at<double>(static_cast<int>(index), 2) = centered[2];
  }
  cv::PCA pca(samples, cv::Mat(), cv::PCA::DATA_AS_ROW);
  const cv::Vec3d axis(
    pca.eigenvectors.at<double>(0, 0),
    pca.eigenvectors.at<double>(0, 1),
    pca.eigenvectors.at<double>(0, 2));
  result.rotation = basis_from_axis(axis);
  result.translation = centroid;
  result.orientation_complete = false;
  result.confidence = std::clamp(
    0.5 * result.depth_valid_ratio +
    0.5 * std::min(1.0, cv::contourArea(*contour) / (bgr.total() * 0.15)),
    0.0, 1.0);
  result.valid = true;
  result.status = "bottle axis pose valid; rotation about bottle axis is unobservable";
  return result;
}

PoseEstimate solve_planar_target_pose(
  const std::array<cv::Point2f, 4> & corners, const cv::Mat & aligned_depth,
  const CameraIntrinsics & camera, double width, double height,
  double depth_scale, double max_reprojection_error_px, double max_depth_residual_m)
{
  PoseEstimate result;
  result.image_corners = corners;
  result.status = "PnP failed";
  if (!camera.valid() || width <= 0.0 || height <= 0.0) {
    result.status = "invalid camera or target dimensions";
    return result;
  }
  const std::vector<cv::Point3f> object_points{
    {-static_cast<float>(width / 2.0),  static_cast<float>(height / 2.0), 0.0F},
    { static_cast<float>(width / 2.0),  static_cast<float>(height / 2.0), 0.0F},
    { static_cast<float>(width / 2.0), -static_cast<float>(height / 2.0), 0.0F},
    {-static_cast<float>(width / 2.0), -static_cast<float>(height / 2.0), 0.0F}};
  const std::vector<cv::Point2f> image_points(corners.begin(), corners.end());

  result.bounding_box = cv::boundingRect(image_points);
  double measured_depth_m = std::numeric_limits<double>::quiet_NaN();
  if (!aligned_depth.empty()) {
    cv::Mat polygon_mask = cv::Mat::zeros(aligned_depth.size(), CV_8UC1);
    std::vector<cv::Point> polygon;
    for (const auto & corner : corners) {polygon.emplace_back(cvRound(corner.x), cvRound(corner.y));}
    cv::fillConvexPoly(polygon_mask, polygon, cv::Scalar(255));
    const auto values = valid_depths(
      aligned_depth, polygon_mask, result.bounding_box, depth_scale, 0.05, 10.0);
    const auto area = std::max(1.0, std::abs(cv::contourArea(polygon)));
    result.depth_valid_ratio = std::min(1.0, static_cast<double>(values.size()) / area);
    measured_depth_m = median(values);
  }

  std::vector<cv::Mat> rotation_vectors;
  std::vector<cv::Mat> translation_vectors;
  if (cv::solvePnPGeneric(
      object_points, image_points, camera.matrix(), cv::Mat(),
      rotation_vectors, translation_vectors, false, cv::SOLVEPNP_IPPE) <= 0)
  {
    return result;
  }

  double best_score = std::numeric_limits<double>::infinity();
  bool found_front_facing_solution = false;
  for (std::size_t solution = 0; solution < rotation_vectors.size(); ++solution) {
    const auto rotation = rotation_from_rvec(rotation_vectors[solution]);
    const auto translation = vector_from_mat(translation_vectors[solution]);
    if (translation[2] <= 0.0) {continue;}

    // Object +Z is defined as the outward face normal. A visible front face must point
    // generally back toward the camera, so its normal has a negative dot product with t.
    const cv::Vec3d outward_normal{
      rotation(0, 2), rotation(1, 2), rotation(2, 2)};
    if (outward_normal.dot(translation) >= 0.0) {continue;}

    std::vector<cv::Point2f> projected;
    cv::projectPoints(
      object_points, rotation_vectors[solution], translation_vectors[solution],
      camera.matrix(), cv::Mat(), projected);
    double squared = 0.0;
    for (std::size_t index = 0; index < projected.size(); ++index) {
      const auto delta = projected[index] - image_points[index];
      squared += delta.dot(delta);
    }
    const auto reprojection_error = std::sqrt(squared / projected.size());
    const auto depth_residual = std::isfinite(measured_depth_m) ?
      std::abs(measured_depth_m - translation[2]) :
      std::numeric_limits<double>::infinity();
    // One centimetre of depth disagreement counts roughly as one pixel for ambiguity
    // selection. Acceptance still uses the independent configured limits below.
    const auto score = reprojection_error +
      (std::isfinite(depth_residual) ? depth_residual * 100.0 : 0.0);
    if (score < best_score) {
      best_score = score;
      result.rotation = rotation;
      result.translation = translation;
      result.reprojection_error_px = reprojection_error;
      result.depth_residual_m = depth_residual;
      found_front_facing_solution = true;
    }
  }
  if (!found_front_facing_solution) {
    result.status = "PnP produced only behind-camera or back-face solutions";
    return result;
  }
  if (result.reprojection_error_px > max_reprojection_error_px) {
    result.status = "reprojection error exceeds limit";
    return result;
  }
  if (std::isfinite(result.depth_residual_m) &&
    result.depth_residual_m > max_depth_residual_m)
  {
    result.status = "PnP/depth disagreement exceeds limit";
    return result;
  }
  result.valid = true;
  result.orientation_complete = true;
  const auto reprojection_score = std::exp(-result.reprojection_error_px / 3.0);
  const auto depth_score = std::isfinite(result.depth_residual_m) ?
    std::exp(-result.depth_residual_m / 0.03) : 0.5;
  result.confidence = std::clamp(0.65 * reprojection_score + 0.35 * depth_score, 0.0, 1.0);
  result.status = "planar target pose valid";
  return result;
}

PoseEstimate estimate_planar_target_pose(
  const cv::Mat & bgr, const cv::Mat & aligned_depth,
  const CameraIntrinsics & camera, const HsvRange & hsv,
  double width, double height, double depth_scale,
  double min_depth_m, double max_depth_m, double min_contour_area_px,
  double max_reprojection_error_px, double max_depth_residual_m)
{
  PoseEstimate result;
  result.status = "no planar target contour";
  if (bgr.empty() || !camera.valid()) {return result;}
  const auto mask = segment_hsv(bgr, hsv);
  const auto contour = largest_contour(mask, min_contour_area_px);
  if (!contour) {return result;}
  std::vector<cv::Point> approximation;
  const auto perimeter = cv::arcLength(*contour, true);
  cv::approxPolyDP(*contour, approximation, 0.025 * perimeter, true);
  std::vector<cv::Point2f> corners;
  if (approximation.size() == 4 && cv::isContourConvex(approximation)) {
    for (const auto & point : approximation) {corners.emplace_back(point);}
  } else {
    cv::Point2f rectangle_corners[4];
    cv::minAreaRect(*contour).points(rectangle_corners);
    corners.assign(rectangle_corners, rectangle_corners + 4);
  }
  result = solve_planar_target_pose(
    order_quad(corners), aligned_depth, camera, width, height,
    depth_scale, max_reprojection_error_px, max_depth_residual_m);
  if (result.valid &&
    (result.translation[2] < min_depth_m || result.translation[2] > max_depth_m))
  {
    result.valid = false;
    result.status = "planar target depth is outside configured range";
  }
  return result;
}

}  // namespace d455_6d_pose
