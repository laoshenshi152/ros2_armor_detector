//代码大部分参考自TARSGO JLURoboVisionN

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>
#include <algorithm>
#include <string>

namespace armor_detector
{
// const int RED = 0;
// const int BLUE = 1;

enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Light : public cv::Rect
{
  Light() = default;
  explicit Light(cv::Rect box, cv::Point2f top, cv::Point2f bottom, int area, float tilt_angle)
  : cv::Rect(box), top(top), bottom(bottom), tilt_angle(tilt_angle)
  {
    length = cv::norm(top - bottom);
    width = area / length;
    center = (top + bottom) / 2;
  }

  int color;
  cv::Point2f top, bottom;
  cv::Point2f center;
  double length;
  double width;
  float tilt_angle;
};
/*
Light 继承自 cv::Rect，表示图像中的一个灯光区域（例如一个矩形框）。

它包含：

color：灯光的颜色（如红色或蓝色）。

top 和 bottom：分别表示灯光区域上部和下部的坐标。

center：灯光区域的中心。

length 和 width：灯光的长度和宽度。

tilt_angle：灯光区域的倾斜角度。
*/


struct Armor
{
  Armor() = default;
  Armor(const Light & l1, const Light & l2)
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
};
/*
Armor 表示装甲，它由两个 Light 组成：left_light 和 right_light。

通过传入两个 Light 对象，Armor 会确定哪个是左灯光，哪个是右灯光，并计算装甲的中心点。

center 是装甲的中心点。

number_img 是装甲的编号图像，number 是装甲的编号，confidence 是识别的置信度，classification_result 是分类结果
*/

}  // namespace armor_detector

#endif  // ARMOR_DETECTOR__ARMOR_HPP_