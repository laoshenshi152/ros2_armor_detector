#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <array>
#include <vector>
#include <opencv2/core.hpp>
#include "armor.hpp"

namespace armor_detector
{

class PnPSolver
{
public:
  PnPSolver() = default;

  PnPSolver(const std::array<double, 9> & camera_matrix,
            const std::vector<double> & dist_coeffs);

  // 使用给定装甲板进行解算
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // 计算装甲中心到图像中心的像素距离
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;

  static constexpr double SMALL_ARMOR_WIDTH = 135.0;   // mm
  static constexpr double SMALL_ARMOR_HEIGHT = 55.0;   // mm
  static constexpr double LARGE_ARMOR_WIDTH = 230.0;   // mm
  static constexpr double LARGE_ARMOR_HEIGHT = 55.0;   // mm
};

}  // namespace armor_detector

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
