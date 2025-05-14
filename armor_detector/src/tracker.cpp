#include "tracker.hpp"
#include "armor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>

namespace armor_tracker
{

ArmorTracker::ArmorTracker(): 
    ekf_([](const Eigen::VectorXd &x) -> Eigen::VectorXd {
      // 状态转移函数 f(x)
      Eigen::VectorXd x_pred(4);
      x_pred(0) = x(0) + x(2);  //x(0)和x(1)是装甲板的位置(x 和 y)，   x(2)和x(3)是装甲板的速度(vx 和 vy)。
      x_pred(1) = x(1) + x(3);  //位置通过速度来更新（x + vx 和 y + vy）
      x_pred(2) = x(2);
      x_pred(3) = x(3);         //速度则保持不变,这样的情况适合匀速运动
      return x_pred;
    },
    [](const Eigen::VectorXd &x) -> Eigen::VectorXd {
      // 观测函数 h(x)
      // 假设我们得到的观测值为装甲板图像坐标
      Eigen::VectorXd z(2);
      // 使用 PnP 计算图像坐标，这里简化为x, y投影
      z(0) = x(0);  // 投影的x坐标
      z(1) = x(1);  // 投影的y坐标
      return z;
    },
    [](const Eigen::VectorXd &x) -> Eigen::MatrixXd {
      // 过程方程的雅可比矩阵 (J_f):对于简单的线性状态转移函数（位置通过速度更新）
      Eigen::MatrixXd J_f(4, 4);
      J_f.setZero();
      J_f(0, 0) = 1;
      J_f(0, 2) = 1;
      J_f(1, 1) = 1;
      J_f(1, 3) = 1;
      return J_f;
    },
    [](const Eigen::VectorXd &x) -> Eigen::MatrixXd {
      // 观测方程的雅可比矩阵 (J_h):由于观测函数仅涉及到位置的直接输出，因此 J_h 只有 x 和 y 对应的部分有非零元素，其他元素为零
      Eigen::MatrixXd J_h(2, 4);
      J_h.setZero();
      J_h(0, 0) = 1;
      J_h(1, 1) = 1;
      return J_h;
    },
    [](const Eigen::VectorXd &x) -> Eigen::MatrixXd {
      // 过程（速度）噪声矩阵 Q  设置了较低的 x 和 y 位置噪声，较高的 x 和 y 速度噪声。这样可以适应目标在相对静止时的噪声特性。
      Eigen::MatrixXd Q(4, 4);
      Q.setIdentity();
      Q(0, 0) = 1e-2;  // x位置噪声
      Q(1, 1) = 1e-2;  // y位置噪声
      Q(2, 2) = 1e-1;  // x速度噪声
      Q(3, 3) = 1e-1;  // y速度噪声
      return Q;
    },
    [](const Eigen::VectorXd &z) -> Eigen::MatrixXd {
      // 观测（坐标）噪声矩阵 R  设置了 x 和 y 坐标的观测噪声，这样可以应对图像坐标测量误差。
      Eigen::MatrixXd R(2, 2);
      R.setIdentity();
      R(0, 0) = 1e-2;  // x坐标噪声
      R(1, 1) = 1e-2;  // y坐标噪声
      return R;
    },
    Eigen::MatrixXd::Identity(4, 4)) {}

void ArmorTracker::init(const armor_detector::Armor &armor, const rclcpp::Time &stamp)
{
  if (!is_initialized_) {
    tracked_armor_ = armor;
    Eigen::VectorXd initial_state(4);
    initial_state << tracked_armor_.center.x, tracked_armor_.center.y, 0, 0;  // x, y, vx, vy
    ekf_.setState(initial_state);
    last_update_time_ = stamp;
    is_initialized_ = true;
  }
}

void ArmorTracker::update(const std::vector<armor_detector::Armor> &armors, const rclcpp::Time &stamp)
{//在每次调用 update 时，首先进行状态的预测（ekf_.predict()），然后基于新的观测值（装甲板的中心点）更新滤波器的状态（ekf_.update(z)）
  if (!is_initialized_ || armors.empty()) {
    return;
  }

  // 选择最合适的装甲板
  tracked_armor_ = selectBestArmor(armors);

  // 获取观测值：当前检测到的装甲板中心点
  Eigen::VectorXd z(2);
  z << tracked_armor_.center.x, tracked_armor_.center.y;

  // 预测状态
  ekf_.predict();

  // 用新的观测更新EKF
  ekf_.update(z);

  last_update_time_ = stamp;
}

bool ArmorTracker::isTracking() const
{
  return is_initialized_;
}

Eigen::VectorXd ArmorTracker::getPredictedState() const
{
  return ekf_.getPredictedState();
}

armor_detector::Armor ArmorTracker::getTrackedArmor() const
{
  return tracked_armor_;
}

armor_detector::Armor ArmorTracker::selectBestArmor(const std::vector<armor_detector::Armor> &armors)
{
  // 选择最合适的装甲板进行跟踪
  // 这里可以根据一些指标（如位置、置信度等）来选择最佳装甲板
  return armors.front();  // 简单选择第一个装甲板
}

void ArmorTracker::predictOnce() 
{
  ekf_.predict();
}

}  // namespace armor_tracker
