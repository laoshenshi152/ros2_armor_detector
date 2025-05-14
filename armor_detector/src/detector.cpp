#include "detector.hpp"
#include <cmath>

namespace armor_detector
{

std::vector<Armor> ArmorDetector::detect(const cv::Mat & input)
{
  std::vector<Light> lights = findLights(input);
  std::vector<Armor> armors = matchArmors(lights);
  return armors;
}

std::vector<Light> ArmorDetector::findLights(const cv::Mat & img)
{
  std::vector<Light> lights;
  cv::Mat channels[3], gray, binary, blur;
  cv::split(img, channels);

  // 使用红色通道（channels[2]）
  gray = channels[2];

  // 二值化 + 高斯模糊
  cv::threshold(gray, binary, 220, 255, cv::THRESH_BINARY);
  cv::GaussianBlur(binary, blur, cv::Size(5, 5), 0);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(blur, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    cv::RotatedRect rrect = cv::minAreaRect(contour);
    float angle = rrect.angle;
    if (rrect.size.width > rrect.size.height)
      std::swap(rrect.size.width, rrect.size.height);

    float ratio = rrect.size.height / rrect.size.width;

    if (ratio > 1.2 && rrect.size.height > 10) {
      cv::Point2f pts[4];
      rrect.points(pts);
      cv::Point2f top = (pts[1] + pts[2]) / 2;
      cv::Point2f bottom = (pts[3] + pts[0]) / 2;

      cv::Rect box = cv::boundingRect(contour);
      int area = cv::contourArea(contour);

      Light light(box, top, bottom, area, angle);
      lights.push_back(light);
    }
  }

  return lights;
}

//这里仿照rm视觉代码写了识别大小装甲板的函数
std::vector<Armor> ArmorDetector::matchArmors(const std::vector<Light> & lights)
{
  std::vector<Armor> armors;
  int n = lights.size();

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const auto & l1 = lights[i];
      const auto & l2 = lights[j];

      if (!isValidArmorPair(l1, l2)) continue;

      Armor armor(l1, l2);

      // 新增：判别装甲板类型（LARGE or SMALL）
      float center_dist = cv::norm(l1.center - l2.center);
      float avg_length = (l1.length + l2.length) / 2;
      float ratio = center_dist / avg_length;

      if (ratio > 3.2f) {
        armor.type = ArmorType::LARGE;
      } else {
        armor.type = ArmorType::SMALL;
      }

      armors.push_back(armor);
    }
  }

  return armors;
}


bool ArmorDetector::isValidArmorPair(const Light & l1, const Light & l2)
{
  float angle_diff = std::abs(l1.tilt_angle - l2.tilt_angle);
  if (angle_diff > 10.0f) return false;

  float length_diff_ratio = std::abs(l1.length - l2.length) / std::max(l1.length, l2.length);
  if (length_diff_ratio > 0.3f) return false;

  float y_diff = std::abs(l1.center.y - l2.center.y);
  float x_diff = std::abs(l1.center.x - l2.center.x);
  if (x_diff == 0 || y_diff / x_diff > 0.5f) return false;

  float center_dist = cv::norm(l1.center - l2.center);
  float avg_length = (l1.length + l2.length) / 2;
  if (center_dist / avg_length > 5.0f || center_dist / avg_length < 1.0f) return false;

  return true;
}

bool ArmorDetector::isColorMatch(const cv::Mat &, int)
{
  // 简化版本：颜色不做判断
  return true;
}

void drawArmors(cv::Mat & image, const std::vector<armor_detector::Armor> & armors)
{
  for (const auto & armor : armors)
  {
    const auto & l1 = armor.left_light;
    const auto & l2 = armor.right_light;

    // 画灯条轮廓
    cv::rectangle(image, l1, cv::Scalar(255, 0, 0), 2);
    cv::rectangle(image, l2, cv::Scalar(255, 0, 0), 2);

    // 画装甲板中心
    cv::circle(image, armor.center, 4, cv::Scalar(0, 255, 0), -1);

    // 画连接线框
    // cv::line(image, l1.center, l2.center, cv::Scalar(0, 255, 255), 1);
  }
}



}  // namespace armor_detector
