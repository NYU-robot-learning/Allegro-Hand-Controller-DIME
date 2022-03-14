using namespace std;

#include "allegro_node_pd.h"
#include <stdio.h>
#include <vector>

#include "std_msgs/Float64MultiArray.h"

#include "ros/ros.h"

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

// Default parameters (if rosparams are not correctly set)
double home_pose[DOF_JOINTS];
Eigen::VectorXd tau_g = Eigen::VectorXd::Zero(DOF_JOINTS);

Eigen::VectorXd current_position_eigen((int)DOF_JOINTS);
Eigen::VectorXd desired_position_eigen((int)DOF_JOINTS);
Eigen::VectorXd current_velocity_eigen((int)DOF_JOINTS);
Eigen::VectorXd tau_pos = Eigen::VectorXd::Zero(DOF_JOINTS);

vector<double> K_p(16);
vector<double> K_d(16);

// Initial limits.
double max_delta_q = 0.15;
double max_tau_des = 0.65;

// Loop rate < 333Hz is recommended since states are received at 333Hz.
double loop_rate = 300.0;

// Initialiing the gravity vector
std::vector<double> g_vec(3);

// Rosparam names
// K_p values
std::string pGainParams[DOF_JOINTS] = {
                "~gains_pd/p/j00", "~gains_pd/p/j01", "~gains_pd/p/j02",
                "~gains_pd/p/j03",
                "~gains_pd/p/j10", "~gains_pd/p/j11", "~gains_pd/p/j12",
                "~gains_pd/p/j13",
                "~gains_pd/p/j20", "~gains_pd/p/j21", "~gains_pd/p/j22",
                "~gains_pd/p/j23",
                "~gains_pd/p/j30", "~gains_pd/p/j31", "~gains_pd/p/j32",
                "~gains_pd/p/j33"
        };

// K_d values
std::string dGainParams[DOF_JOINTS] = {
                "~gains_pd/d/j00", "~gains_pd/d/j01", "~gains_pd/d/j02",
                "~gains_pd/d/j03",
                "~gains_pd/d/j10", "~gains_pd/d/j11", "~gains_pd/d/j12",
                "~gains_pd/d/j13",
                "~gains_pd/d/j20", "~gains_pd/d/j21", "~gains_pd/d/j22",
                "~gains_pd/d/j23",
                "~gains_pd/d/j30", "~gains_pd/d/j31", "~gains_pd/d/j32",
                "~gains_pd/d/j33"
        };

// Home position 
std::string initialPosition[DOF_JOINTS] = {
                "~initial_position/j00", "~initial_position/j01",
                "~initial_position/j02",
                "~initial_position/j03",
                "~initial_position/j10", "~initial_position/j11",
                "~initial_position/j12",
                "~initial_position/j13",
                "~initial_position/j20", "~initial_position/j21",
                "~initial_position/j22",
                "~initial_position/j23",
                "~initial_position/j30", "~initial_position/j31",
                "~initial_position/j32",
                "~initial_position/j33"
        };

// Constructor subscribes to topics.
AllegroNodePD::AllegroNodePD() 
  : AllegroNode() {
  control_hand_ = false;
  gravity_comp_ = false;

  initController(whichHand);

  lib_cmd_sub = nh.subscribe(LIB_CMD_TOPIC, 1, &AllegroNodePD::libCmdCallback, this);

  joint_cmd_sub = nh.subscribe(DESIRED_STATE_TOPIC, 1, &AllegroNodePD::setJointCallback, this);
                
  grav_rot_sub = nh.subscribe(GRAV_ROT_TOPIC, 1, //300, // queue size
                                &AllegroNodePD::handGravityVectorCallback, this);
  
  grav_comp_torques.position.resize(0);
  grav_comp_torques.velocity.resize(0);
  grav_comp_torques.effort.resize(DOF_JOINTS);
  grav_comp_torques.name.resize(DOF_JOINTS);
                
  commanded_joint_states.position.resize(DOF_JOINTS);
  commanded_joint_states.velocity.resize(DOF_JOINTS);
  commanded_joint_states.effort.resize(DOF_JOINTS);
  commanded_joint_states.name.resize(DOF_JOINTS);
    
  commanded_joint_state_pub = nh.advertise<sensor_msgs::JointState>(COMMANDED_JOINT_STATE_TOPIC, 3);
  grav_comp_torques_pub = nh.advertise<sensor_msgs::JointState>(GRAV_COMP_TOPIC, 3);

  kdl_comp = new allegroKDL(g_vec, loop_rate);
  kdl_comp->load_gains(K_p, K_d, max_tau_des, max_delta_q);
}

AllegroNodePD::~AllegroNodePD() {
  ROS_INFO("PD controller node is shutting down");
}

// Called when an external (string) message is received
void AllegroNodePD::libCmdCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());

  const std::string lib_cmd = msg->data.c_str();

  // Compare the message received to an expected input
  if (lib_cmd.compare("pdControl") == 0) {
    ROS_INFO("Using PD control!");
    control_hand_ = true;
    gravity_comp_ = false;
  }

  else if (lib_cmd.compare("home") == 0) {
    ROS_INFO("Moving home!");
    // Set the home position as the desired joint states.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    control_hand_ = true;
    gravity_comp_ = false;
    mutex->unlock();
  }

  // Setting the hand at gravity comp mode
  else if (lib_cmd.compare("gravcomp") == 0) {
    ROS_INFO("Using grav comp!");
    gravity_comp_ = true;
    control_hand_ = true;
  }
}

void AllegroNodePD::setJointCallback(const sensor_msgs::JointState &msg) {
  ROS_WARN_COND(!control_hand_, "Setting control_hand_ to True because of "
                "received JointState message");
  control_hand_ = true;
}

void AllegroNodePD::handGravityVectorCallback(const std_msgs::Float64MultiArray &msg) {
  g_vec[0] = msg.data[0];
  g_vec[1] = msg.data[1];
  g_vec[2] = msg.data[2];
}

void AllegroNodePD::computeDesiredTorque() {
  // NOTE: here we just compute and set the desired_torque class member
  // variable.
  for (int iterator=0; iterator < DOF_JOINTS; iterator++) {
    current_position_eigen[iterator] = current_position_filtered[iterator];
    desired_position_eigen[iterator] = desired_joint_state.position[iterator];
    current_velocity_eigen[iterator] = current_velocity_filtered[iterator];
  }

  // Obtaining the gravity compensation torques using the dynamic gravity vector
  kdl_comp->update_G(g_vec);

  kdl_comp->get_G(current_position_eigen, tau_g);

  // No control: set torques to zero.
  if (!control_hand_) {
    //ROS_INFO_THROTTLE(1.0, "Hand control is false");
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_torque[i] = 0.0;
    }
    return;
  }

  // Getting PD Torques
  kdl_comp->get_PD(desired_position_eigen, current_position_eigen, current_velocity_eigen, tau_pos);

  // Sanity/defensive check: if *both* position and torques are set in the
  // message, do nothing.
  if (desired_joint_state.position.size() > 0 &&
      desired_joint_state.effort.size() > 0) {
    ROS_WARN("Error: both positions and torques are specified in the desired "
                     "state. You cannot control both at the same time.");
    return;
  }

  // Updating the torques in other conditions
  {
    mutex->lock();

    if (desired_joint_state.position.size() == DOF_JOINTS) {
      // Control joint positions: compute the desired torques (PD control).
      commanded_joint_states.position.resize(DOF_JOINTS);

      for (int i = 0; i < DOF_JOINTS; i++) {
        // Not applying additional torques if in gravity compensation mode.
        if(gravity_comp_) {
          desired_torque[i] = tau_g[i];
        } else {
          desired_torque[i] = tau_g[i] + tau_pos[i];

        }
        // Clamping max torques.
        if (desired_torque[i] > max_tau_des) desired_torque[i] = max_tau_des;
        else if (desired_torque[i] < -max_tau_des) desired_torque[i] = -max_tau_des;
        commanded_joint_states.effort[i] = desired_torque[i];
        commanded_joint_states.position[i] = desired_joint_state.position[i];
      }
    } else if (desired_joint_state.effort.size() > 0) {
      // Control joint torques: set desired torques as the value stored in the
      // desired_joint_state message.
      commanded_joint_states.position.resize(0);
      for (int i = 0; i < DOF_JOINTS; i++) {
        desired_torque[i] = desired_joint_state.effort[i] + tau_g[i];
        
        // Clamping max torques.
        if (desired_torque[i] > max_tau_des) desired_torque[i] = max_tau_des;
        else if (desired_torque[i] < -max_tau_des) desired_torque[i] = -max_tau_des;
        commanded_joint_states.effort[i] = desired_torque[i];
      }
    }

    for (int i = 0; i < DOF_JOINTS; i++) {
        grav_comp_torques.header.stamp = tnow;
        commanded_joint_states.header.stamp = tnow;
        grav_comp_torques.effort[i] = tau_g[i];
    }

    // Torque publishers
    grav_comp_torques_pub.publish(grav_comp_torques);
    commanded_joint_state_pub.publish(commanded_joint_states);
    
    mutex->unlock();
  }
}

void AllegroNodePD::initController(const std::string &whichHand) {
  // set gains_pd via gains_pd.yaml or to default values
  if (ros::param::has("~gains_pd")) {
    ROS_INFO("CTRL: PD gains loaded from param server.");

    // Loading PD gain values
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(pGainParams[i], K_p[i]);
      ros::param::get(dGainParams[i], K_d[i]);
    }
  }
  else {
    // gains will be loaded every control iteration
    ROS_WARN("CTRL: PD gains not loaded");
    ROS_WARN("Check launch file is loading /parameters/gains_pd.yaml");
    ROS_WARN("Loading default PD gains...");
  }

  // set initial position via initial_position.yaml or to default values
  if (ros::param::has("~initial_position")) {
    ROS_INFO("CTRL: Initial Pose loaded from param server.");
    double tmp;    
    mutex->lock();    
    desired_joint_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(initialPosition[i], tmp);
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(tmp);
      home_pose[i] = tmp;
    }
    mutex->unlock();
  }
  else {
    ROS_WARN("CTRL: Initial position not loaded.");
    ROS_WARN("Check launch file is loading /parameters/initial_position.yaml");
    ROS_WARN("Loading Home position instead...");

    // Home position
    mutex->lock();
    desired_joint_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    mutex->unlock();
  }
  control_hand_ = false;
  gravity_comp_ = false;

  printf("*************************************\n");
  printf("      Joint PD Control Method        \n");
  printf("-------------------------------------\n");
  printf("  Only 'H', 'Space', 'Z' works. \n");
  printf("*************************************\n");
}

void AllegroNodePD::doIt(bool polling) {
  // Main spin loop, uses the publisher/subscribers.
  if (polling) {
    ROS_INFO("Polling = true.");
    ros::Rate rate(loop_rate);

    while (ros::ok()) {
      updateController();
      ros::spinOnce();
      rate.sleep();
    }
  } else {
    ROS_INFO("Polling = false.");

    // Timer callback (not recommended).
    ros::Timer timer = startTimerCallback();
    ros::spin();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "allegro_hand_core_pd");
  AllegroNodePD allegroNode;

  bool polling = false;
  if (argv[1] == std::string("true")) {
    polling = true;
  }
  allegroNode.doIt(polling);
}