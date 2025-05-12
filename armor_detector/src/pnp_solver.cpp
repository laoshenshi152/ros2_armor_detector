#include "pnp_solver.hpp"
#include <opencv2/calib3d.hpp>

namespace armor_detector
{

PnPSolver::PnPSolver(const std::array<double, 9> & camera_matrix,
                     const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // 单位：米（将毫米转化）
  constexpr double s_half_w = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double s_half_h = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double l_half_w = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double l_half_h = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // 四个角点（从左下开始顺时针）
  small_armor_points_ = {
    {0,  s_half_w, -s_half_h},
    {0,  s_half_w,  s_half_h},
    {0, -s_half_w,  s_half_h},
    {0, -s_half_w, -s_half_h}
  };

  large_armor_points_ = {
    {0,  l_half_w, -l_half_h},
    {0,  l_half_w,  l_half_h},
    {0, -l_half_w,  l_half_h},
    {0, -l_half_w, -l_half_h}
  };
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_points;

  // 左下 -> 左上 -> 右上 -> 右下
  image_points.emplace_back(armor.left_light.bottom);
  image_points.emplace_back(armor.left_light.top);
  image_points.emplace_back(armor.right_light.top);
  image_points.emplace_back(armor.right_light.bottom);

  const auto & object_points =
    (armor.type == ArmorType::SMALL) ? small_armor_points_ : large_armor_points_;

  return cv::solvePnP(
    object_points, image_points, camera_matrix_, dist_coeffs_,
    rvec, tvec, false, cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
  float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace armor_detector
