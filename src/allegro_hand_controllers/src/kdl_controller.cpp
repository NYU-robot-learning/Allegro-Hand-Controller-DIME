#include "kdl_controller.h"
#include "ros/ros.h"
#include <iostream>

#define Joint 13

allegroKDL::allegroKDL(std::vector<double> g_vec, double control_rate) {
  // load urdf file
  _urdf_file = ros::package::getPath("ll4ma_robots_description");
  _urdf_file.append("/urdf/allegro_right/allegro_kdl.urdf");

  _ee_names = {"index_tip", "middle_tip", "ring_tip", "thumb_tip"};
  _base_names = {"palm_link", "palm_link", "palm_link", "palm_link"};

  _g_vec = g_vec;
  loop_rate = control_rate;

  // build kdl tree and chains:
  _allegro_kdl = new robotKDL(_urdf_file, _base_names, _ee_names, _g_vec);
  _allegro_kdl->getJointLimits(0, max_j_limits, min_j_limits);
  _allegro_kdl->getJointLimits(1, max_j_limits, min_j_limits);
  _allegro_kdl->getJointLimits(2, max_j_limits, min_j_limits);
  _allegro_kdl->getJointLimits(3, max_j_limits, min_j_limits);
}

void allegroKDL::load_gains(std::vector<double> kp, std::vector<double> kd, double max_tau, double max_delta_joint) {
  K_p = kp;
  K_d = kd;
  max_tau_des = max_tau;
  max_delta_q = max_delta_joint;
}

void allegroKDL::get_G(const int idx, const Eigen::VectorXd &q, Eigen::VectorXd &tau_g) {
  // send only joints of idx finger:
  _q_finger.resize(4);

  for (int i =0; i<4; i++) {
    _q_finger[i] = q[idx * 4 + i];
  }

  _allegro_kdl->getGtau(idx, q, tau_g);
}

void allegroKDL::get_G(const Eigen::VectorXd &q, Eigen::VectorXd &tau_g) {
  tau_g.resize(16);

  // send only joints of idx finger:
  for (int j = 0; j < 4; j++) {
    _q_finger.resize(4);
    
    for(int i = 0; i < 4; i++) {
      _q_finger[i] = q[j * 4 + i];
    }

    _allegro_kdl->getGtau(j, _q_finger, _tau_g_finger);

    for (int i =0;i<4;i++) {
	    tau_g[j * 4 + i] = _tau_g_finger[i];
	  }
  }

  // ROS_INFO("Computed gravity torque: %f", tau_g[1]);

  // increasing thumb first joint:
  tau_g[12] = 1.01 * tau_g[12];
}

void allegroKDL::get_G(const Eigen::VectorXd &q) {
   allegroKDL::get_G(q, _tau_g);
}

void allegroKDL::get_PD(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD) {
  tau_PD.resize(16);

  for (int i = 0; i < 16; i++) {
    delta_q = q_des[i]-q[i];

    if(delta_q > max_delta_q) {
      delta_q = max_delta_q;
    } else if(delta_q < -max_delta_q) {
      delta_q = -max_delta_q;
    } 

    tau_PD[i] = (K_p[i] * (delta_q) - K_d[i] * q_dot[i]);

    // limit tau within threshold:
    if(std::abs(tau_PD[i]) > max_tau_des) {
      tau_PD[i] = std::copysign(max_tau_des, tau_PD[i]);
    }
  }
}

void allegroKDL::update_G(std::vector<double> g_vec) {
  _g_vec = g_vec;
  _allegro_kdl->update_g_vec(_g_vec);
}

// Destructor
allegroKDL::~allegroKDL() {
  delete _allegro_kdl;
}
