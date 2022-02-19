// Builds a kdl tree from robot description for ik solving of the allegro hand(palm_link to fingers) and takes subscribers and solves the IK giving joint angles.
// Author: bala@cs.utah.edu
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <urlg_msgs/JointCommands.h>
#include <std_msgs/Char.h>
#include <urlg_robots_ik_solvers/IK_command.h>
#include <vector>
bool init_tree=false;
bool get_data=false;
using namespace std;
ros::Publisher joint_pub;
urlg_robots_ik_solvers::IK_command ik_c;
//TRAC_IK::TRAC_IK ik_solver;
void ikCallback(urlg_robots_ik_solvers::IK_command ik_temp)
{
  ik_c=ik_temp;
  get_data=true;
  
  //std::cerr<<return_joints(6);
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"allegro_ik_solver");
  ros::NodeHandle n;
  ros::Rate loop_rate(1000);
 
  ros::Subscriber ik_sub=n.subscribe("/allegro_hand_right/ik/command",1,ikCallback);
  joint_pub=n.advertise<urlg_msgs::JointCommands>("/allegro_hand_right/control/joint_cmd",1);
  ros::Publisher control_pub=n.advertise<std_msgs::Char>("/allegro_hand_right/control/control_type",1);
  //  KDL::Tree full_tree;
  std::string robot_desc_string;
  n.param("robot_description",robot_desc_string,std::string());
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

  TRAC_IK::TRAC_IK* ik_sol;
  while(ros::ok())
    {
      std_msgs::Char c_meth;
      c_meth.data='p';
      control_pub.publish(c_meth);
       if(!init_tree && get_data)
	{
	  vector<string> frames=ik_c.frame_names;
	  //	  TRAC_IK::TRAC_IK ik_solver(frames[0],frames[1],"/robot_description",0.005,1e-5,TRAC_IK::Speed);
	  TRAC_IK::TRAC_IK ik_solver("palm_link","ring_link_3",robot_desc_string,0.005,1e-5,TRAC_IK::Speed);

	  init_tree=true;
	  ik_sol=&ik_solver;

	}
      if(init_tree && get_data)
	{
	  KDL::JntArray joint_seed,return_joints;
	  //TODO Make joint size generic
	  joint_seed.resize(4);
	  vector<double> pose=ik_c.cart_pose;
	  int rc = ik_sol->CartToJnt(joint_seed, KDL::Frame({pose[0],pose[1],pose[2]}),return_joints);
	  urlg_msgs::JointCommands j_cmd;
	  j_cmd.name={"joint8","joint9","joint10","joint11"};
	  j_cmd.position={return_joints(0),return_joints(1),return_joints(2),return_joints(3)};
	  joint_pub.publish(j_cmd);
	  get_data=false;
	}

      ros::spinOnce();
      loop_rate.sleep();
    }
  return(0);
}
