#include "tracker.hpp"
#include "armor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include "pnp_solver.hpp"

namespace armor_tracker
{

ArmorTracker::ArmorTracker(): 
    ekf_([](const Eigen::VectorXd &x) -> Eigen::VectorXd {
      // 状态转移函数 f(x)
      Eigen::VectorXd x_pred(4);
      x_pred(0) = x(0) + x(2);  // x position = x + vx
      x_pred(1) = x(1) + x(3);  // y position = y + vy
      x_pred(2) = x(2);         // vx remains constant
      x_pred(3) = x(3);         // vy remains constant
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
      // 雅可比矩阵 J_f
      Eigen::MatrixXd J_f(4, 4);
      J_f.setZero();
      J_f(0, 0) = 1;
      J_f(0, 2) = 1;
      J_f(1, 1) = 1;
      J_f(1, 3) = 1;
      return J_f;
    },
    [](const Eigen::VectorXd &x) -> Eigen::MatrixXd {
      // 雅可比矩阵 J_h
      Eigen::MatrixXd J_h(2, 4);
      J_h.setZero();
      J_h(0, 0) = 1;
      J_h(1, 1) = 1;
      return J_h;
    },
    [](const Eigen::VectorXd &x) -> Eigen::MatrixXd {
      // 过程噪声矩阵 Q
      Eigen::MatrixXd Q(4, 4);
      Q.setIdentity();
      Q(0, 0) = 1e-2;  // x位置噪声
      Q(1, 1) = 1e-2;  // y位置噪声
      Q(2, 2) = 1e-1;  // x速度噪声
      Q(3, 3) = 1e-1;  // y速度噪声
      return Q;
    },
    [](const Eigen::VectorXd &z) -> Eigen::MatrixXd {
      // 观测噪声矩阵 R
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
{
  if (!is_initialized_) {
    return;
  }

  // 预测下一个状态
  Eigen::VectorXd predicted_state = ekf_.predict();

  // 通过PnP获取新的观测值（假设我们已经有了装甲板的图像坐标）
  Eigen::VectorXd z = ekf_.getMeasurement(predicted_state);

  // 使用观测值更新EKF
  ekf_.update(z);

  // 选择最合适的装甲板进行跟踪
  tracked_armor_ = selectBestArmor(armors);

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

}  // namespace armor_tracker
