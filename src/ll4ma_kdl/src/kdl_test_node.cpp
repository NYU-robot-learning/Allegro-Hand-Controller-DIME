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
#include "manipulator_kdl/robot_kdl.h"

#include <ros/ros.h>
#include <ros/package.h>

using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc,argv,"kdl_test_node");
  ros::NodeHandle n;


  // urdf file location:
  //string urdf_file = ros::package::getPath("urlg_robots_description");
  //urdf_file.append("/debug/lbr4_kdl.urdf");
  vector<string> ee_names={"lbr4_7_link"};
  vector<string> base_names={"base_link"};
  vector<double> g_vec={0.0,0.0,-9.8};
  // initialize kdl class:
  // load robot from robot_description:
  manipulator_kdl::robotKDL lbr4_("robot_description",n,base_names,ee_names,g_vec);
  cerr<<"Created KDL model"<<endl;
  Eigen::VectorXd j_pos;
  j_pos.resize(7);
  j_pos.setZero();
  int ch=0;
  bool rpy=true;
  Eigen::VectorXd ee_pose;
  lbr4_.getFK(ch,j_pos,ee_pose,rpy);
  cerr<<ee_pose<<endl;
  Eigen::VectorXd tau_g;
  lbr4_.getGtau(0,j_pos,tau_g);
  cerr<<tau_g<<endl;
  
}
