#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node
{
public:
  VideoPublisher() : Node("video_publisher")
  {
    image_pub_ = image_transport::create_publisher(this, "input_image");
    cap_.open("src/样例3.mp4");

    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开视频文件！");
      return;
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33), std::bind(&VideoPublisher::publish_frame, this)); // ~30 FPS
  }

private:
  void publish_frame()
  {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_INFO(this->get_logger(), "视频播放完毕");
      rclcpp::shutdown();
      return;
    }

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    image_pub_.publish(msg);
  }

  image_transport::Publisher image_pub_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoPublisher>());
  rclcpp::shutdown();
  return 0;
}
