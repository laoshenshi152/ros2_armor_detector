#ifndef ARMOR_TRACKER__ARMOR_TRACKER_HPP_
#define ARMOR_TRACKER__ARMOR_TRACKER_HPP_

#include "armor.hpp"
#include "kalmanfilter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

namespace armor_tracker
{

class ArmorTracker
{
public:
  ArmorTracker();

  void init(const armor_detector::Armor & armor, const rclcpp::Time & stamp);
  void update(const std::vector<armor_detector::Armor> & armors, const rclcpp::Time & stamp);
  bool isTracking() const;

  Eigen::VectorXd getPredictedState() const;
  armor_detector::Armor getTrackedArmor() const;
  void predictOnce();

private:
  armor_tracker::ExtendedKalmanFilter ekf_;
  armor_detector::Armor tracked_armor_;
  rclcpp::Time last_update_time_;
  bool is_initialized_ = false;

  armor_detector::Armor selectBestArmor(const std::vector<armor_detector::Armor> & armors);
};

}  // namespace armor_tracker

#endif  // ARMOR_TRACKER__ARMOR_TRACKER_HPP_
