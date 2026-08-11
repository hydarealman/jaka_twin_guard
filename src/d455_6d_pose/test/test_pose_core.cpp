#include "d455_6d_pose/pose_core.hpp"

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <vector>

namespace pose = d455_6d_pose;

TEST(PoseCore, ConvertsAndBackProjectsDepth)
{
  cv::Mat depth = cv::Mat::zeros(4, 5, CV_16UC1);
  depth.at<std::uint16_t>(2, 3) = 1250;
  EXPECT_NEAR(pose::depth_at_m(depth, 3, 2, 0.001), 1.25, 1e-12);
  pose::CameraIntrinsics camera{500.0, 500.0, 2.0, 2.0, 5, 4};
  const auto point = pose::back_project(3.0, 2.0, 1.25, camera);
  EXPECT_NEAR(point[0], 0.0025, 1e-12);
  EXPECT_NEAR(point[1], 0.0, 1e-12);
  EXPECT_NEAR(point[2], 1.25, 1e-12);
}

TEST(PoseCore, OrdersQuadrilateralCorners)
{
  const std::vector<cv::Point2f> shuffled{
    {400.0F, 300.0F}, {100.0F, 100.0F},
    {100.0F, 300.0F}, {400.0F, 100.0F}};
  const auto ordered = pose::order_quad(shuffled);
  EXPECT_EQ(ordered[0], cv::Point2f(100.0F, 100.0F));
  EXPECT_EQ(ordered[1], cv::Point2f(400.0F, 100.0F));
  EXPECT_EQ(ordered[2], cv::Point2f(400.0F, 300.0F));
  EXPECT_EQ(ordered[3], cv::Point2f(100.0F, 300.0F));
}

TEST(PoseCore, OrdersRotatedQuadrilateralWithoutDuplicateCorners)
{
  const std::vector<cv::Point2f> shuffled{
    {320.0F, 80.0F}, {500.0F, 240.0F},
    {140.0F, 240.0F}, {320.0F, 400.0F}};
  const auto ordered = pose::order_quad(shuffled);
  for (std::size_t first = 0; first < ordered.size(); ++first) {
    for (std::size_t second = first + 1; second < ordered.size(); ++second) {
      EXPECT_NE(ordered[first], ordered[second]);
    }
  }
  const auto edge_a = ordered[1] - ordered[0];
  const auto edge_b = ordered[2] - ordered[1];
  EXPECT_GT(edge_a.x * edge_b.y - edge_a.y * edge_b.x, 0.0F);
}

TEST(PoseCore, RecoversSyntheticPlanarPose)
{
  pose::CameraIntrinsics camera{620.0, 615.0, 320.0, 240.0, 640, 480};
  constexpr double width = 0.23;
  constexpr double height = 0.127;
  const std::vector<cv::Point3f> object_points{
    {-static_cast<float>(width / 2.0),  static_cast<float>(height / 2.0), 0.0F},
    { static_cast<float>(width / 2.0),  static_cast<float>(height / 2.0), 0.0F},
    { static_cast<float>(width / 2.0), -static_cast<float>(height / 2.0), 0.0F},
    {-static_cast<float>(width / 2.0), -static_cast<float>(height / 2.0), 0.0F}};
  cv::Mat delta_rotation_mat;
  cv::Rodrigues(cv::Vec3d{0.08, -0.12, 0.04}, delta_rotation_mat);
  cv::Matx33d delta_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      delta_rotation(row, column) = delta_rotation_mat.at<double>(row, column);
    }
  }
  const cv::Matx33d expected_rotation =
    delta_rotation * cv::Matx33d(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
  cv::Mat expected_rotation_mat(expected_rotation);
  cv::Vec3d expected_rvec;
  cv::Rodrigues(expected_rotation_mat, expected_rvec);
  const cv::Vec3d expected_translation{0.025, -0.018, 0.82};
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_points, expected_rvec, expected_translation,
    camera.matrix(), cv::Mat(), projected);
  std::array<cv::Point2f, 4> corners;
  std::copy(projected.begin(), projected.end(), corners.begin());
  cv::Mat depth(480, 640, CV_16UC1, cv::Scalar(820));
  const auto result = pose::solve_planar_target_pose(
    corners, depth, camera, width, height, 0.001, 1.0, 0.05);
  ASSERT_TRUE(result.valid) << result.status;
  EXPECT_TRUE(result.orientation_complete);
  EXPECT_LT(cv::norm(result.translation - expected_translation), 1e-4);
  EXPECT_LT(pose::rotation_error_deg(result.rotation, expected_rotation), 0.05);
  const cv::Vec3d outward_normal{
    result.rotation(0, 2), result.rotation(1, 2), result.rotation(2, 2)};
  EXPECT_LT(outward_normal.dot(result.translation), 0.0);
  EXPECT_LT(result.reprojection_error_px, 0.1);
}

TEST(PoseCore, BottleReportsAxialSymmetry)
{
  pose::CameraIntrinsics camera{600.0, 600.0, 320.0, 240.0, 640, 480};
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::rectangle(image, cv::Rect(280, 100, 80, 300), cv::Scalar(255, 0, 0), cv::FILLED);
  cv::Mat depth = cv::Mat::zeros(480, 640, CV_16UC1);
  cv::rectangle(depth, cv::Rect(280, 100, 80, 300), cv::Scalar(1000), cv::FILLED);
  const auto result = pose::estimate_bottle_pose(
    image, depth, camera, pose::HsvRange{}, 0.001,
    0.2, 2.0, 1000.0, 1.3, 3);
  ASSERT_TRUE(result.valid) << result.status;
  EXPECT_FALSE(result.orientation_complete);
  EXPECT_NEAR(result.translation[2], 1.0, 1e-6);
  EXPECT_LT(result.rotation(1, 2), -0.9);
  EXPECT_GT(result.depth_valid_ratio, 0.95);
}
