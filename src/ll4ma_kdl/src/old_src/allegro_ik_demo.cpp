// Sends end effector poses to a topic which will be used by ik solvers
// Author: bala@cs.utah.edu
#include <ros/ros.h>
#include <urlg_robots_ik_solvers/IK_command.h>
#include <vector>



int main(int argc, char** argv )
{
  ros::init(argc,argv,"pose_pub_to_ik");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher ik_pub=n.advertise<urlg_robots_ik_solvers::IK_command>("ik/command",1);
  while(ros::ok())
    {
      urlg_robots_ik_solvers::IK_command ik_c;
      ik_c.frame_names.resize(2);
      ik_c.frame_names[0]="palm_link";
      ik_c.robot_desc="ring_tip";
      ik_c.cart_pose.resize(3);
      ik_c.cart_pose[0]=0.2;
      ik_c.cart_pose[1]=0.2;
      ik_c.cart_pose[2]=-0.1;
      ik_pub.publish(ik_c);
      ros::spinOnce();
      loop_rate.sleep();
    }
  return(0);
}
  
