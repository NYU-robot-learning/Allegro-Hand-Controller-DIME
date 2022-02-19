/*
 * Copyright (C) 2017  Balakumar Sundaralingam, LL4MA lab
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// This file tests the different functions from our manipulator_kdl library, use this as a reference
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include "manipulator_kdl/robot_kdl.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/console.h>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

using std::string;
using std::vector;
using std::cerr;
using std::endl;

// read joint state from robot:

Eigen::VectorXd js;
void js_cb(sensor_msgs::JointState msg)
{
  js=Eigen::Map<Eigen::VectorXd>(msg.position.data(),msg.position.size());
}

void ws_cb(geometry_msgs::WrenchStamped msg)
{
  
}
int main(int argc, char** argv)
{
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal))
 {
   ros::console::notifyLoggerLevelsChanged();
 }

  ros::init(argc,argv,"kdl_test_node");
  ros::NodeHandle n;

  ros::Subscriber js_sub=n.subscribe("/lbr4_allegro/joint_states",1,js_cb);

  // urdf file location:
  //string urdf_file = ros::package::getPath("urlg_robots_description");
  //urdf_file.append("/debug/lbr4_kdl.urdf");
  vector<string> ee_names={"palm_link"};
  string sensor_node="optoforce_sensor_link";
  vector<string> base_names={"optoforce_sensor_link"};
  vector<double> g_vec={0.0,0.0,-9.8};
  Eigen::VectorXd g_vec_;
  g_vec_.resize(3);
  g_vec_[2]=-9.8;
  
  // Create dart model:
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  loader.addPackageDirectory("ll4ma_robots_description","/home/bala/catkin_ws/src/ll4ma_robots_description");

  // load skeleton from rosparam:
  std::string urdf_st;
  n.getParam("robot_description", urdf_st);
  //cerr<<urdf_st<<endl;
  SkeletonPtr manipulator = loader.parseSkeletonString(urdf_st,"");
  manipulator->setName("lbr4");
  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);
  
  //Eigen::VectorXd j_pos;
  //j_pos.resize(7+16);
  // j_pos.setZero();
  //j_pos[1]=1.2;

  // Get dart values:
  manipulator->setGravity(g_vec_);
  ros::Rate loop_rate(100);
  
  while(ros::ok())
  {
    
    loop_rate.sleep();
    ros::spinOnce();
    if(js.size()<1)
    {
      continue;
    }

    manipulator->setPositions(js);
  

    Eigen::MatrixXd M_d=manipulator->getMassMatrix();
    BodyNode* b_node= manipulator->getBodyNode("optoforce_sensor_link");
    //Joint* b_joint=manipulator->getJoint("optoforce_sensor_joint");
    //int idx=manipulator->getIndexOf(b_node);
    dart::dynamics::Inertia n_inertia=b_node->getArticulatedInertia();
    //cerr<<n_inertia.getMass()<<endl;
    //cerr<<n_inertia.getLocalCOM()<<endl;
    Eigen::Vector3d r_c=n_inertia.getLocalCOM();
    double m=n_inertia.getMass();
    Eigen::Matrix6d mI=n_inertia.getSpatialTensor();
    Eigen::Vector3d l_gvec=dart::math::AdInvRLinear(b_node->getWorldTransform(), g_vec_).tail(3);
    cerr<<"Force:"<<endl;
    cerr<<m*l_gvec<<endl;
    Eigen::VectorXd mG_F = mI * dart::math::AdInvRLinear(b_node->getWorldTransform(), g_vec_);
    cerr<<"Torque"<<endl;
    cerr<<m*r_c.cross(l_gvec)<<endl;
    //cerr<<"Spatial"<<endl;
    //cerr<<mG_F<<endl;
    
  }

}
