//
// Created by felixd on 10/1/15.
//

#ifndef PROJECT_ALLEGRO_NODE_COMMON_H
#define PROJECT_ALLEGRO_NODE_COMMON_H

// Defines DOF_JOINTS.
#include "allegro_hand_driver/AllegroHandDrv.h"
using namespace allegro;

#include <string>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdio.h>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <chrono>
#include <Eigen/Dense>

// Forward declaration.
class AllegroHandDrv;

#define ALLEGRO_CONTROL_TIME_INTERVAL 0.003

// Topic names: current & desired JointState, named grasp to command.
const std::string JOINT_STATE_TOPIC = "allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "allegroHand/joint_cmd";
const std::string COMMANDED_JOINT_STATE_TOPIC = "allegroHand/commanded_joint_states";
const std::string GRAV_COMP_TOPIC = "allegroHand/grav_comp_torques";
const std::string LIB_CMD_TOPIC = "allegroHand/lib_cmd";
const std::string GRAV_ROT_TOPIC = "/j2n6s300_driver/hand_gravity_vector";

class AllegroNode {
 public:

  AllegroNode(bool sim = false);

  virtual ~AllegroNode();

  void publishData();

  void desiredStateCallback(const sensor_msgs::JointState &desired);

  virtual void updateController();

  // This is the main method that must be implemented by the various
  // controller nodes.
  virtual void computeDesiredTorque() {
    ROS_ERROR("Called virtual function!");
  };

  ros::Timer startTimerCallback();

  void timerCallback(const ros::TimerEvent &event);

 protected:

  double current_position[DOF_JOINTS] = {0.0};
  double previous_position[DOF_JOINTS] = {0.0};

  double current_position_filtered[DOF_JOINTS] = {0.0};
  double previous_position_filtered[DOF_JOINTS] = {0.0};

  double current_velocity[DOF_JOINTS] = {0.0};
  double previous_velocity[DOF_JOINTS] = {0.0};
  double current_velocity_filtered[DOF_JOINTS] = {0.0};

  double desired_torque[DOF_JOINTS] = {0.0};

  std::string whichHand;  // Right or left hand.

  // ROS stuff
  ros::NodeHandle nh;
  ros::Publisher joint_state_pub;
  ros::Publisher commanded_joint_state_pub;
  ros::Publisher grav_comp_torques_pub;
  ros::Subscriber joint_cmd_sub;
  ros::Subscriber grav_rot_sub;

  // Store the current and desired joint states.
  sensor_msgs::JointState current_joint_state;
  sensor_msgs::JointState desired_joint_state;
  sensor_msgs::JointState commanded_joint_states;
  sensor_msgs::JointState grav_comp_torques;
  std_msgs::Float64MultiArray frame_rotation_angles;
 

  // ROS Time
  ros::Time tstart;
  ros::Time tnow;
  double dt;
  double sample_rate;
  double write_rate;
  std::chrono::time_point<std::chrono::system_clock> begin_time, end_time,write_time;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> dt_clocks;

  // CAN device
  allegro::AllegroHandDrv *canDevice;
  boost::mutex *mutex;

  // Flags
  int lEmergencyStop = 0;
  long frame = 0;
};

#endif //PROJECT_ALLEGRO_NODE_COMMON_H