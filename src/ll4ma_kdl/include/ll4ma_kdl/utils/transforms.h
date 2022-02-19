#include "debug.h"
#include<iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
// ros
#include<ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

namespace ll4ma_kdl
{
class transformClass
{
public:
  /// Converts quaterion to Euler RPY
  Eigen::VectorXd get_RPY_q(const double &x, const double &y,const double &z, const double &w) const;
  /// Converts Euler RPY to quaternion
  Eigen::VectorXd get_q_RPY(const double &R,const double &P, const double &Y) const;
};
}
