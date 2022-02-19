#include "ll4ma_kdl/utils/transforms.h"

namespace ll4ma_kdl
{
Eigen::VectorXd transformClass::get_q_RPY(const double &R,const  double &P,const double &Y) const
{
  //
  tf::Matrix3x3 R_;
  R_.setRPY(R,P,Y);
  tf::Quaternion q;
  R_.getRotation(q);
  Eigen::VectorXd q_;
  q_.resize(4);
  q_[0]=q.x();
  q_[1]=q.y();
  q_[2]=q.z();
  q_[3]=q.w();
  return q_;
  
}
Eigen::VectorXd transformClass::get_RPY_q(const double &x, const double &y,const double &z, const double &w) const
{
  tf::Matrix3x3 R_;
  tf::Quaternion q(x,y,z,w);
  R_.setRotation(q);
  Eigen::VectorXd rpy_;
  rpy_.resize(3);
  R_.getRPY(rpy_[0],rpy_[1],rpy_[2]);
  
  return rpy_;
}

}
