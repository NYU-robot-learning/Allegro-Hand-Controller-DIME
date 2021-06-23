using namespace std;

#include "allegro_node_pd.h"
#include "kdl_controller.h"
#include <stdio.h>
#include <vector>

#include "ros/ros.h"

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

// Default parameters (if rosparams are not correctly set)
double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

vector<double> K_p = {
  1.0, 2.0, 2.0, 2.0,
  1.0, 2.0, 2.0, 2.0,
  1.0, 2.0, 1.7, 1.5,
  2.0, 1.5, 2.0, 2.0
};
vector<double> K_d = {
  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.12,
  0.1, 0.15, 0.12, 0.12,
  0.15, 0.12, 0.15, 0.15
};

// PD on position and velocity
// Currently unused. Future TODO to integrate velocity PD.
vector<double> traj_K_p = {
  1.0, 0.8, 1.0, 1.0,
  1.0, 0.8, 1.0, 1.0,
  1.0, 0.8, 1.0, 1.0,
  1.0, 1.0, 0.8, 1.0  
};
vector<double> traj_K_d = {
  0.001, 0.001, 0.001, 0.001,
  0.001, 0.001, 0.001, 0.001,
  0.001, 0.001, 0.001, 0.001,
  0.001, 0.001, 0.001, 0.001
};
vector<double> traj_K_i = {
  50.0,50.0,50.0,50.0,
  50.0,50.0,50.0,50.0,
  50.0,50.0,50.0,50.0,
  50.0,50.0,50.0,50.0,
};
vector<double> vel_K_d = {
  0.001, 0.001, 0.001, 0.001,
  0.001, 0.001, 0.001, 0.001,
  0.001, 0.001, 0.001, 0.001,
  0.001, 0.001, 0.001, 0.001    
};

// Initial limits.
double max_q_vel=0.01;
double max_delta_q=0.15;
double max_tau_des=0.65;

// Loop rate < 333Hz is recommended since states are received at 333Hz.
double loop_rate = 300.0;

// TODO: Convert this to a ROS subscriber when robot is attached.
std::vector<double> g_vec={0.0,0.0,-9.8};

// Rosparam names
// K_p values
std::string pGainParams[DOF_JOINTS] =
        {
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
std::string dGainParams[DOF_JOINTS] =
        {
                "~gains_pd/d/j00", "~gains_pd/d/j01", "~gains_pd/d/j02",
                "~gains_pd/d/j03",
                "~gains_pd/d/j10", "~gains_pd/d/j11", "~gains_pd/d/j12",
                "~gains_pd/d/j13",
                "~gains_pd/d/j20", "~gains_pd/d/j21", "~gains_pd/d/j22",
                "~gains_pd/d/j23",
                "~gains_pd/d/j30", "~gains_pd/d/j31", "~gains_pd/d/j32",
                "~gains_pd/d/j33"
        };

// Traj_K_d values 
std::string pTrajGainParams[DOF_JOINTS] =
        {
                "~gains_pd/traj_p/j00", "~gains_pd/traj_p/j01", "~gains_pd/traj_p/j02",
                "~gains_pd/traj_p/j03",
                "~gains_pd/traj_p/j10", "~gains_pd/traj_p/j11", "~gains_pd/traj_p/j12",
                "~gains_pd/traj_p/j13",
                "~gains_pd/traj_p/j20", "~gains_pd/traj_p/j21", "~gains_pd/traj_p/j22",
                "~gains_pd/traj_p/j23",
                "~gains_pd/traj_p/j30", "~gains_pd/traj_p/j31", "~gains_pd/traj_p/j32",
                "~gains_pd/traj_p/j33"
        };

// Traj_K_p values
std::string dTrajGainParams[DOF_JOINTS] =
        {
                "~gains_pd/traj_d/j00", "~gains_pd/traj_d/j01", "~gains_pd/traj_d/j02",
                "~gains_pd/traj_d/j03",
                "~gains_pd/traj_d/j10", "~gains_pd/traj_d/j11", "~gains_pd/traj_d/j12",
                "~gains_pd/traj_d/j13",
                "~gains_pd/traj_d/j20", "~gains_pd/traj_d/j21", "~gains_pd/traj_d/j22",
                "~gains_pd/traj_d/j23",
                "~gains_pd/traj_d/j30", "~gains_pd/traj_d/j31", "~gains_pd/traj_d/j32",
                "~gains_pd/traj_d/j33"
        };

// Traj_K_i values
std::string iTrajGainParams[DOF_JOINTS] =
        {
                "~gains_pd/traj_i/j00", "~gains_pd/traj_i/j01", "~gains_pd/traj_i/j02",
                "~gains_pd/traj_i/j03",
                "~gains_pd/traj_i/j10", "~gains_pd/traj_i/j11", "~gains_pd/traj_i/j12",
                "~gains_pd/traj_i/j13",
                "~gains_pd/traj_i/j20", "~gains_pd/traj_i/j21", "~gains_pd/traj_i/j22",
                "~gains_pd/traj_i/j23",
                "~gains_pd/traj_i/j30", "~gains_pd/traj_i/j31", "~gains_pd/traj_i/j32",
                "~gains_pd/traj_i/j33"
        };

// Vel_d values
std::string dVelGainParams[DOF_JOINTS] =
        {
                "~gains_pd/vel_d/j00", "~gains_pd/vel_d/j01", "~gains_pd/vel_d/j02",
                "~gains_pd/vel_d/j03",
                "~gains_pd/vel_d/j10", "~gains_pd/vel_d/j11", "~gains_pd/vel_d/j12",
                "~gains_pd/vel_d/j13",
                "~gains_pd/vel_d/j20", "~gains_pd/vel_d/j21", "~gains_pd/vel_d/j22",
                "~gains_pd/vel_d/j23",
                "~gains_pd/vel_d/j30", "~gains_pd/vel_d/j31", "~gains_pd/vel_d/j32",
                "~gains_pd/vel_d/j33"
        };

std::string maxJointVelocity = "~gains_pd/max_q_vel";
std::string maxJointDelta = "~gains_pd/max_delta_q";
std::string maxDesiredJointTorque = "~gains_pd/max_tau_des";

// Home position 
std::string initialPosition[DOF_JOINTS] =
        {
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

std::string loopRate = "~shared/parameters/loop_rate";

std::string gravityVector[3] = {
        "~shared/parameters/g_vector/x",
        "~shared/parameters/g_vector/y",
        "~shared/parameters/g_vector/z"
};


// Constructor subscribes to topics.
AllegroNodePD::AllegroNodePD()
        : AllegroNode() {
  control_hand_ = false;

  initController(whichHand);

  lib_cmd_sub = nh.subscribe(
          LIB_CMD_TOPIC, 1, &AllegroNodePD::libCmdCallback, this);

  joint_cmd_sub = nh.subscribe(
          DESIRED_STATE_TOPIC, 1, &AllegroNodePD::setJointCallback, this);
                
  
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
    control_hand_ = true;
  }

  else if (lib_cmd.compare("home") == 0) {
    // Set the home position as the desired joint states.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    control_hand_ = true;
    mutex->unlock();
  }
  else if (lib_cmd.compare("off") == 0) {
    control_hand_ = false;
  }

  else if (lib_cmd.compare("save") == 0) {
    // Set the current position as the desired joint states.
    mutex->lock();
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = current_position[i];
    mutex->unlock();
  }
}

void AllegroNodePD::setJointCallback(const sensor_msgs::JointState &msg) {
  ROS_WARN_COND(!control_hand_, "Setting control_hand_ to True because of "
                "received JointState message");
  control_hand_ = true;
}

void AllegroNodePD::computeDesiredTorque() {
  // NOTE: here we just compute and set the desired_torque class member
  // variable.

  allegroKDL kdl_comp(g_vec,loop_rate);
  kdl_comp.load_gains(K_p,K_d,traj_K_p,traj_K_d,traj_K_i,vel_K_d,max_tau_des,max_delta_q,max_q_vel);
  
  Eigen::VectorXd tau_g;

  Eigen::VectorXd current_position_eigen((int)DOF_JOINTS);
  Eigen::VectorXd desired_position_eigen((int)DOF_JOINTS);
  Eigen::VectorXd current_velocity_eigen((int)DOF_JOINTS);
  Eigen::VectorXd tau_pos = Eigen::VectorXd::Zero(DOF_JOINTS);

  for (int iterator=0; iterator < DOF_JOINTS; iterator++) {
    current_position_eigen[iterator] = current_position_filtered[iterator];
  }

  for (int iterator=0; iterator < DOF_JOINTS; iterator++) {
    desired_position_eigen[iterator] = desired_joint_state.position[iterator];
  }

  for (int iterator=0; iterator < DOF_JOINTS; iterator++) {
    current_velocity_eigen[iterator] = current_velocity_filtered[iterator];
  }

  kdl_comp.get_G(current_position_eigen, tau_g);
  kdl_comp.get_PD(desired_position_eigen, current_position_eigen, current_velocity_eigen, tau_pos);

  // No control: set torques to zero.
  if (!control_hand_) {
    //ROS_INFO_THROTTLE(1.0, "Hand control is false");
    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_torque[i] = 0.0;
    }
    return;
  }

  // Sanity/defensive check: if *both* position and torques are set in the
  // message, do nothing.
  if (desired_joint_state.position.size() > 0 &&
      desired_joint_state.effort.size() > 0) {
    ROS_WARN("Error: both positions and torques are specified in the desired "
                     "state. You cannot control both at the same time.");
    return;
  }

  {
    mutex->lock();

    if (desired_joint_state.position.size() == DOF_JOINTS) {
      // Control joint positions: compute the desired torques (PD control).

      commanded_joint_states.position.resize(DOF_JOINTS);
      for (int i = 0; i < DOF_JOINTS; i++) {
        
        desired_torque[i] = tau_g[i];
        desired_torque[i] += tau_pos[i];   

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
        // grav_comp_torques.name[i] = desired_joint_state.name[i];
        // commanded_joint_states.name[i] = desired_joint_state.name[i];
        grav_comp_torques.effort[i] = tau_g[i];
    }
    grav_comp_torques_pub.publish(grav_comp_torques);
    commanded_joint_state_pub.publish(commanded_joint_states);
    mutex->unlock();
  }
}

void AllegroNodePD::initController(const std::string &whichHand) {
  // set gains_pd via gains_pd.yaml or to default values
  if (ros::param::has("~gains_pd")) {
    ROS_INFO("CTRL: PD gains loaded from param server.");
    for (int i = 0; i < DOF_JOINTS; i++) {
      ros::param::get(pGainParams[i], K_p[i]);
      ros::param::get(dGainParams[i], K_d[i]);
      ros::param::get(pTrajGainParams[i], traj_K_p[i]);
      ros::param::get(dTrajGainParams[i], traj_K_d[i]);
      ros::param::get(iTrajGainParams[i], traj_K_i[i]);
      ros::param::get(dVelGainParams[i], vel_K_d[i]);
    }

    ros::param::get(maxJointVelocity, max_q_vel);
    ros::param::get(maxJointDelta, max_delta_q);
    ros::param::get(maxDesiredJointTorque, max_tau_des);
    ros::param::get(loopRate, loop_rate);
  
    for (int i = 0; i < 3; i++) {
      ros::param::get(gravityVector[i], g_vec[i]);
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

  printf("*************************************\n");
  printf("      Joint PD Control Method        \n");
  printf("-------------------------------------\n");
  printf("  Only 'H', 'O', 'S', 'Space' works. \n");
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