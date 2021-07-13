#include "kdl_controller.h"
#include "ros/ros.h"
#include <iostream>

#define Joint 13

allegroKDL::allegroKDL(std::vector<double> g_vec, double control_rate)
{
  // load urdf file
  _urdf_file = ros::package::getPath("ll4ma_robots_description");
  _urdf_file.append("/urdf/allegro_right/allegro_kdl.urdf");
  _ee_names={"index_tip","middle_tip","ring_tip","thumb_tip"};
  _base_names={"palm_link","palm_link","palm_link","palm_link"};
  _g_vec=g_vec;
  loop_rate=control_rate;

  // build kdl tree and chains:
  _allegro_kdl = new robotKDL(_urdf_file,_base_names,_ee_names,_g_vec);
  _allegro_kdl->getJointLimits(0, max_j_limits, min_j_limits);
  _allegro_kdl->getJointLimits(1, max_j_limits, min_j_limits);
  _allegro_kdl->getJointLimits(2, max_j_limits, min_j_limits);
  _allegro_kdl->getJointLimits(3, max_j_limits, min_j_limits);
}

void allegroKDL::load_gains(std::vector<double> kp, std::vector<double> kd, std::vector<double> traj_kp, std::vector<double> traj_kd, std::vector<double> traj_ki, std::vector<double> vel_Kd, double max_tau, double max_delta,double max_qvel)
{
  K_p=kp;
  K_d=kd;
  traj_K_p=traj_kp;
  traj_K_d=traj_kd;
  max_tau_des=max_tau;
  max_delta_q=max_delta;
  max_q_vel=max_qvel;
  vel_K_d=vel_Kd;
  traj_K_i=traj_ki;
  q_i.clear();
  q_i.resize(16,0.0);
}

void allegroKDL::get_G(const int idx, const Eigen::VectorXd &q, Eigen::VectorXd &tau_g)
{
  // send only joints of idx finger:
  _q_finger.resize(4);
  for (int i =0;i<4;i++)
  {
    _q_finger[i]=q[idx*4+i];
  }
  _allegro_kdl->getGtau(idx, q, tau_g);
}

void allegroKDL::get_G(const Eigen::VectorXd &q, Eigen::VectorXd &tau_g)
{
  tau_g.resize(16);
  // send only joints of idx finger:
  for (int j=0;j<4;j++)
    {
      _q_finger.resize(4);
      for (int i =0;i<4;i++)
	{
	  _q_finger[i]=q[j*4+i];
	}
      _allegro_kdl->getGtau(j,_q_finger,_tau_g_finger);
      for (int i =0;i<4;i++)
	{
	  tau_g[j*4+i]=_tau_g_finger[i];
	}
    }
  // increasing thumb first joint:
  tau_g[12]=1.01*tau_g[12];
}

void allegroKDL::get_G(const Eigen::VectorXd &q)
{
   allegroKDL::get_G(q, _tau_g);
}

std::vector<double> allegroKDL::get_C(int idx,std::vector<double> q,std::vector<double> q_dot)
{
  std::vector<double> c_tau;
  return c_tau;
}

std::vector<double> allegroKDL::get_tau_ff(std::vector<double> u_pd,std::vector<double> q,std::vector<double> q_dd)
{
  std::vector<double> tau_ff;
  Eigen::Map<Eigen::VectorXd> tau_u(&u_pd[0],u_pd.size());
  Eigen::Map<Eigen::VectorXd> qdd(&q_dd[0],q_dd.size());
  tau_u=tau_u+qdd;
  
  tau_ff.resize(16);
  Eigen::MatrixXd H;
  for (int j=0;j<4;j++)
  {
    
    _q_finger.resize(4);
    for (int i =0;i<4;i++)
    {
      _q_finger[i]=q[j*4+i];
    }
    _allegro_kdl->getM(j,_q_finger,H);
    Eigen::VectorXd tau_H=H*tau_u.segment(j*4,4);
    for (int i=0;i<4;i++)
    {
      tau_ff[j*4+i]=tau_H[i];
    }
  }
  
  return tau_ff;
  
}

void allegroKDL::get_PD(const Eigen::VectorXd &q_des,const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD)
{
  tau_PD.resize(16);

  for (int i=0;i<16;i++)
  {
    delta_q=q_des[i]-q[i];

    if(delta_q>max_delta_q)
    {
      delta_q=max_delta_q;
    }
    else if(delta_q<-max_delta_q)
    {
      delta_q=-max_delta_q;
    } 

    tau_PD[i]=(K_p[i]*(delta_q)-K_d[i]*q_dot[i]);

    // limit tau within threshold:
    if(std::abs(tau_PD[i])>max_tau_des)
    {
      tau_PD[i]=std::copysign(max_tau_des, tau_PD[i]);
    }
  }
}

void allegroKDL::get_PD(const Eigen::VectorXd &q_des,const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot)
{
  allegroKDL::get_PD(q_des, q, q_dot, _tau_PD);
}

void allegroKDL::get_traj_PD(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD)
{
  tau_PD.resize(16);
  
  for (int i=0;i<16;i++)
    {
      delta_q=q_des[i]-q[i];

      if(delta_q>max_delta_q)
      {
        delta_q=max_delta_q;
      }
      else if(delta_q<-max_delta_q)
      {
        delta_q=-max_delta_q;
      }
      tau_PD[i]=(traj_K_p[i]*(delta_q)+traj_K_d[i]*(q_dot_des[i]-q_dot[i]));
      // limit tau within threshold:
      if(std::abs(tau_PD[i])>max_tau_des)
      {
        tau_PD[i]=std::copysign(max_tau_des, tau_PD[i]);
      }
    }
}

void allegroKDL::get_traj_PD(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot)
{
  allegroKDL::get_traj_PD(q_des, q_dot_des, q, q_dot, _tau_PD);
}


void allegroKDL::get_traj_PID(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot)
{
  allegroKDL::get_traj_PID(q_des, q_dot_des, q, q_dot, _tau_PD);
}

void allegroKDL::get_traj_PID(const Eigen::VectorXd &q_des, const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD)
{
  static int counter=0;
  tau_PD.resize(16);
  
  for (int i=0;i<16;i++)
    {
      delta_q=q_des[i]-q[i];

      if(delta_q>max_delta_q)
      {
        delta_q=max_delta_q;
      }
      else if(delta_q<-max_delta_q)
      {
        delta_q=-max_delta_q;
      }
      q_i[i]+=delta_q*(1/loop_rate);
      if((counter%int(.1*loop_rate + .5)==1) && i==1)
      {
        std::cerr<<"i: "<<traj_K_i[i]*q_i[i]<<" p:"<<traj_K_p[i]*(delta_q)<<std::endl;

      }
      tau_PD[i]=(traj_K_p[i]*(delta_q)+traj_K_d[i]*(q_dot_des[i]-q_dot[i])+traj_K_i[i]*q_i[i]);

      // limit tau within threshold:
      if(std::abs(tau_PD[i])>max_tau_des)
      {
        tau_PD[i]=std::copysign(max_tau_des, tau_PD[i]);
      }
    }
  counter++;
}

void allegroKDL::get_vel_PD(const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q_dot)
{
  allegroKDL::get_vel_PD(q_dot_des, q_dot, _tau_PD);
}

void allegroKDL::get_vel_PD(const Eigen::VectorXd &q_dot_des, const Eigen::VectorXd &q_dot, Eigen::VectorXd &tau_PD)
{

  tau_PD.resize(16);
  
  for (int i=0;i<16;i++)
  {
    double delta=q_dot_des[i]-q_dot[i];

    if(q_dot_des[i]==0.0)// && std::abs(q_dot[i])<5e-2)
    {
      delta=0.0;
    }
    if(std::abs(delta)>max_q_vel)
    {
      delta=std::copysign(max_q_vel,delta);
    }
    tau_PD[i]=(vel_K_d[i]*(delta));
        
    // limit tau within threshold:
    if(std::abs(tau_PD[i])>max_tau_des)
    {
      tau_PD[i]=std::copysign(max_tau_des, tau_PD[i]);
    }
  }
}

allegroKDL::~allegroKDL () {
  delete _allegro_kdl;
}
