#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <ll4ma_kdl/manipulator_kdl/robot_kdl.h>
// #include <debug.h>

using namespace manipulator_kdl;

class allegroKDL
{
 public:
  allegroKDL(std::vector<double> g_vec, double control_rate);// constructor- builds kdl chain
  ~allegroKDL();
  void load_gains(std::vector<double> kp, std::vector<double> kd, std::vector<double> traj_kp, std::vector<double> traj_kd, std::vector<double> traj_ki,std::vector<double> vel_kd, double max_tau, double max_delta,double max_qvel);
  // Dynamic functions:
  // gravity compensation
  void get_G(std::vector<double> g_vec, const Eigen::VectorXd &q);
  void get_G(std::vector<double> g_vec, const Eigen::VectorXd &q, Eigen::VectorXd &tau_g);
  void get_G(std::vector<double> g_vec, const int idx, const Eigen::VectorXd &q, Eigen::VectorXd &tau_g);
  
  std::vector<double> get_C(int idx,std::vector<double> q,std::vector<double> q_dot);
  std::vector<double> get_tau_ff(std::vector<double> u_pd, std::vector<double> q,std::vector<double> q_dd);

  void get_PD(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot);
  void get_PD(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD);

  void get_traj_PD(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD);
  void get_traj_PD(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot);

  void get_vel_PD(const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD);
  void get_vel_PD(const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q_dot);

  void get_traj_PID(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD);
  void get_traj_PID(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot);

  std::vector<double> max_j_limits,min_j_limits;

  void updateG(std::vector<double> g_vec);
  robotKDL* _allegro_kdl;

 private:
  std::vector<double> _g_vec;//={0.0,0.0,-9.8};
  std::string _urdf_file;// = ros::package::getPath("ll4ma_robots_description");
  
  std::vector<std::string> _ee_names;//={"palm_link","index_tip","middle_tip","ring_tip","thumb_tip"};
  std::vector<std::string> _base_names;//={"base_link","base_link","base_link","base_link","base_link"};
  Eigen::VectorXd _q_finger;
  Eigen::VectorXd _tau_g_finger;
  Eigen::VectorXd _tau_g;
  Eigen::VectorXd _tau_PD;
  double delta_q;

  std::vector<double> K_p;
  std::vector<double> K_d;
  std::vector<double> traj_K_p, traj_K_d,traj_K_i,vel_K_d;
  std::vector<double> q_i;
  double max_delta_q;
  double max_tau_des;
  double loop_rate;
  double max_q_vel;
};