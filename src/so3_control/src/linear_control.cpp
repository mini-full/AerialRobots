#include <linear_control.h>
#include <iostream>
#include <ros/ros.h>

LinearControl::LinearControl()
  : mass_(0.49)
  , g_(9.81)
{
}

void
LinearControl::setMass(const double mass)
{
  mass_ = mass;
}

void
LinearControl::setGravity(const double g)
{
  g_ = g;
}

void
LinearControl::calculateControl(const Desired_State_t &des,
                        const Odom_Data_t &odom, 
                        const Imu_Data_t &imu,
                        Controller_Output_t &u,
                        Gain gain)
{
  // Calculate the error
  
  
  Eigen::Vector3d e_p = des.p - odom.p;
  Eigen::Vector3d e_v = des.v - odom.v;
  // Eigen::Vector3d e_a = des.a - imu.a;

  // Calculate the control
  Eigen::Vector3d p_c;
  p_c(0) = des.a(0) + gain.Kp0 * e_p(0) + gain.Kv0 * e_v(0);
  p_c(1) = des.a(1) + gain.Kp1 * e_p(1) + gain.Kv1 * e_v(1);
  p_c(2) = des.a(2) + gain.Kp2 * e_p(2) + gain.Kv2 * e_v(2);
  u.thrust = mass_ * (g_ + p_c(2));

  Eigen::Quaterniond q_c;
  
  // turn imu data into rpy
  Eigen::Vector3d rpy = imu.q.matrix().eulerAngles(2,1,0);

  double roll = (p_c(0) * sin(rpy(2)) - p_c(1) * cos(rpy(2))) / g_;
  double pitch = (p_c(0) * cos(rpy(2)) + p_c(1) * sin(rpy(2))) / g_;

  std::cout << roll << "roll" << std::endl;
  std::cout << pitch << "pitch" << std:: endl;

  Eigen::Vector3d eulerAngle(roll, pitch, 0);
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));

  u.q = yawAngle*pitchAngle*rollAngle;
 
}
