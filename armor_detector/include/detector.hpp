#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include "armor.hpp"

namespace armor_detector
{

class ArmorDetector
{
public:
  // 主接口：输入图像，输出检测到的装甲板
  std::vector<Armor> detect(const cv::Mat & input);

private:
  // 子功能：从图像中提取灯条
  std::vector<Light> findLights(const cv::Mat & img);

  // 子功能：从灯条配对装甲板（并判别大小类型）
  std::vector<Armor> matchArmors(const std::vector<Light> & lights);

  // 判断两个灯条是否构成合法装甲板对
  bool isValidArmorPair(const Light & l1, const Light & l2);

  // 判断颜色是否匹配（如需启用颜色判断逻辑）
  bool isColorMatch(const cv::Mat & roi, int target_color);

  // 绘制装甲板（用于调试可视化）
  void drawArmors(cv::Mat & image, const std::vector<armor_detector::Armor> & armors);
};

}  // namespace armor_detector

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
