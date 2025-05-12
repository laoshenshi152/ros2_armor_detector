// Copyright 2022 Chen Jun

#include "kalmanfilter.hpp"

namespace armor_tracker
{
  ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0): 
  f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post), Q = update_Q(x_post);

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  H = jacobian_h(x_pri), R = update_R(z);

  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  x_post = x_pri + K * (z - h(x_pri));
  P_post = (I - K * H) * P_pri;

//约束条件 适用于实际情况，降低干扰
  // if(tracked_id == "outpost"){
  //   x_post(1) = 0;
  //   x_post(3) = 0;
  //   x_post(5) = 0;
  // }

  return x_post;
}

// 新增方法：获取观测值
Eigen::VectorXd ExtendedKalmanFilter::getMeasurement(const Eigen::VectorXd & predicted_state) const
{
  return h(predicted_state);
}

}  // namespace armor_tracker