// src/armor_detector_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "detector.hpp"
#include "tracker.hpp" 

using std::placeholders::_1;

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode() : Node("armor_detector_node")
  {
    // 创建订阅器和发布器 订阅的是video_publisher 发布了debug_image的图像
    image_sub_ = image_transport::create_subscription(
      this, "input_image", std::bind(&ArmorDetectorNode::imageCallback, this, _1), "raw");

    image_pub_ = image_transport::create_publisher(this, "debug_image");

    RCLCPP_INFO(this->get_logger(), "ArmorDetector Node 已启动");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat frame;

    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }

    // 调用装甲检测器进行装甲板检测
    std::vector<armor_detector::Armor> armors = detector_.detect(frame);

    // 创建图像的debug副本并可视化装甲板检测结果
    cv::Mat debug_frame = frame.clone();
    armor_detector::drawArmors(debug_frame, armors);

    // 更新装甲板跟踪器
    if (!armors.empty()) {
      if (!armor_tracker_.isTracking()) {
        // 如果跟踪器尚未初始化，则初始化
        armor_tracker_.init(armors[0], msg->header.stamp);
      } else {
        // 否则更新跟踪器
        armor_tracker_.update(armors, msg->header.stamp);
      }
    }

    // 为了显示，额外预测一次下一帧的位置（不会改变实际状态，仅用于可视化）
    armor_tracker_.predictOnce();  // 推进一步用于显示

    // 获取并可视化跟踪的装甲板位置
    armor_detector::Armor tracked_armor = armor_tracker_.getTrackedArmor();
    // 绘制跟踪的装甲板（如果有的话）
    if (armor_tracker_.isTracking()) {
    Eigen::VectorXd pred_state = armor_tracker_.getPredictedState();
    cv::Point predicted_point(pred_state(0), pred_state(1));
    cv::circle(debug_frame, predicted_point, 6, CV_RGB(0, 0, 255), 2);  // 蓝色圆圈表示预测位置
    cv::putText(debug_frame, "Pred", predicted_point, cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 0, 255), 1);
    }

    // 发布带有检测和跟踪结果的图像
    auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_frame).toImageMsg();
    image_pub_.publish(debug_msg);
  }

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  armor_detector::ArmorDetector detector_;
  armor_tracker::ArmorTracker armor_tracker_;  // 添加 ArmorTracker 成员变量
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
