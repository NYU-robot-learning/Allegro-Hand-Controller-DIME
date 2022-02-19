// Builds a kdl tree from lbr4 urdf file for ik solving
// Author: bala@cs.utah.edu
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <urlg_msgs/JointCommands.h>
#include <std_msgs/Char.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"lbr4_kdl_parser");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  
  ros::Publisher joint_pub=n.advertise<urlg_msgs::JointCommands>("lbr_allegro/lbr4/control/joint_cmd",1);
  ros::Publisher control_pub=n.advertise<std_msgs::Char>("lbr_allegro/lbr4/control/control_type",1);
  

  KDL::Tree full_tree;
  std::string robot_desc_string;
  n.param("robot_description",robot_desc_string,std::string());

  TRAC_IK::TRAC_IK ik_solver("base_link","palm_link","/robot_description",0.005,1e-5,TRAC_IK::Speed);
  KDL::JntArray joint_seed,return_joints;
  joint_seed.resize(7);
  int rc = ik_solver.CartToJnt(joint_seed, KDL::Frame({0.2,0.2,-0.1}),return_joints);
  std::cerr<<return_joints(6);
  /*  if(!kdl_parser::treeFromString(robot_desc_string,full_tree))
    {
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
  else
    {
      ROS_INFO("Built full tree");
    }
  */
  while(ros::ok())
    {
      std_msgs::Char c_meth;
      c_meth.data='p';
      control_pub.publish(c_meth);
      urlg_msgs::JointCommands j_cmd;
      j_cmd.name={"joint0","joint1","joint2","joint3","joint4","joint5","joint6"};
      j_cmd.position={return_joints(0),return_joints(1),return_joints(2),return_joints(3),return_joints(4),return_joints(5),return_joints(6)};
      joint_pub.publish(j_cmd);
      ros::spinOnce();
      loop_rate.sleep();
    }
  return(0);
}
