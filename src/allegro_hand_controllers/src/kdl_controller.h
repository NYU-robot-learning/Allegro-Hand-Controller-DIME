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

  void load_gains(std::vector<double> kp, std::vector<double> kd, double max_tau, double max_delta_joint);

  void get_G(const Eigen::VectorXd &q);
  void get_G(const Eigen::VectorXd &q, Eigen::VectorXd &tau_g);
  void get_G(const int idx, const Eigen::VectorXd &q, Eigen::VectorXd &tau_g);

  void get_PD(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD);

  void update_G(std::vector<double> g_vec);

  std::vector<double> max_j_limits, min_j_limits;
  std::vector<double> _g_vec;

  robotKDL* _allegro_kdl;

 private:
  std::string _urdf_file;
  
  std::vector<std::string> _ee_names;
  std::vector<std::string> _base_names;
  Eigen::VectorXd _q_finger;
  Eigen::VectorXd _tau_g_finger;
  Eigen::VectorXd _tau_g;
  Eigen::VectorXd _tau_PD;
  double delta_q;

  std::vector<double> K_p;
  std::vector<double> K_d;
  double max_delta_q;
  double max_tau_des;
  double loop_rate;
};