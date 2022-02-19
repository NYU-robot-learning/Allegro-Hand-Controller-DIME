/*
 * Copyright (C) 2017 LL4MA lab, University of Utah
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// This function generates a kdl chain and has functions from KDL

#include "robot_kdl_bck.h"

// using std::map;

namespace manipulator_kdl_bck
{
robotKDL::robotKDL()
{
}
robotKDL::robotKDL(string robot_desc_param,ros::NodeHandle &n,vector<string> base_names,vector<string> EE_names,
                   std::vector<double> grav_vector)
{
  
  KDL::Vector grav_vec(grav_vector[0],grav_vector[1],grav_vector[2]);
  g_vec_=grav_vec;

  std::string robot_desc_string;
  n.param(robot_desc_param, robot_desc_string, std::string());
  // Build kdl tree:
  
  if (!kdl_parser::treeFromString(robot_desc_string, robot_tree_)){
    ROS_ERROR("Failed to construct kdl tree from ros parameter %s",robot_desc_param.c_str());
    //return false;
  }

  // Generate chains:
  kdl_chains_.resize(EE_names.size());
  fk_solvers_.resize(EE_names.size());
  dyn_solvers_.resize(EE_names.size());
  jacobian_solvers_.resize(EE_names.size());
  H_mat_.resize(EE_names.size());
  // resize data
  jacobians_.resize(EE_names.size());

  for(int i=0;i<EE_names.size();++i)
  {
    string ee_name=EE_names[i];
    if(!robot_tree_.getChain(base_names[i],ee_name,kdl_chains_[i]))
    {
      ROS_ERROR("Failed to construct kdl chain from %s to %s",base_names[i].c_str(),ee_name.c_str());
    }
    // No idea how this works. Switch to boost ptr at some point.

    fk_solvers_[i]=new KDL::ChainFkSolverPos_recursive(kdl_chains_[i]);
    dyn_solvers_[i]= new KDL::ChainDynParam(kdl_chains_[i],g_vec_);
    jacobian_solvers_[i] = new KDL::ChainJntToJacSolver(kdl_chains_[i]);
    // initialize data (allow use in real-time?)
    jacobians_[i] = new KDL::Jacobian(kdl_chains_[i].getNrOfJoints());
    
 
    H_mat_[i] = new KDL::JntSpaceInertiaMatrix(kdl_chains_[i].getNrOfJoints());
    #ifdef DEBUG
    cerr<<"Created kinematic and Dynamic chains"<<endl;
    #endif
    
  } 
  //ROS_INFO("initialized robotkdl");

  // This does not work, maybe because the created object is not returned back? seg fault occurs.
  //robotKDL(file_name,base_names,EE_names,grav_vec);
}


robotKDL::robotKDL(string file_name,vector<string> base_names,vector<string> EE_names,
                   std::vector<double> grav_vector)
{

  //up_bounds=;
  //low_bounds=;
  urdf_file_=file_name;
  KDL::Vector grav_vec(grav_vector[0],grav_vector[1],grav_vector[2]);
  g_vec_=grav_vec;
    // Generate kdl tree:
  if (!kdl_parser::treeFromFile(file_name, robot_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree from urdf file: %s",file_name.c_str());
  }
  dof=robot_tree_.getNrOfJoints();

  // Generate chains:
  kdl_chains_.resize(EE_names.size());
  fk_solvers_.resize(EE_names.size());
  dyn_solvers_.resize(EE_names.size());
  jacobian_solvers_.resize(EE_names.size());
  H_mat_.resize(EE_names.size());
  // resize data
  jacobians_.resize(EE_names.size());

  for(int i=0;i<EE_names.size();++i)
  {
    string ee_name=EE_names[i];
    if(!robot_tree_.getChain(base_names[i],ee_name,kdl_chains_[i]))
    {
      ROS_ERROR("Failed to construct kdl chain from %s to %s",base_names[i].c_str(),ee_name.c_str());
    }
    // No idea how this works. Switch to boost ptr at some point.

    fk_solvers_[i]=new KDL::ChainFkSolverPos_recursive(kdl_chains_[i]);
    dyn_solvers_[i]= new KDL::ChainDynParam(kdl_chains_[i],g_vec_);
    jacobian_solvers_[i] = new KDL::ChainJntToJacSolver(kdl_chains_[i]);
    // initialize data (allow use in real-time?)
    jacobians_[i] = new KDL::Jacobian(kdl_chains_[i].getNrOfJoints());
    
 
    H_mat_[i] = new KDL::JntSpaceInertiaMatrix(kdl_chains_[i].getNrOfJoints());
#ifdef DEBUG
    cerr<<"Created kinematic and Dynamic chains"<<endl;
#endif
    
  } 
  //ROS_INFO("initialized robotkdl");

  // This does not work, maybe because the created object is not returned back? seg fault occurs.
  //robotKDL(file_name,base_names,EE_names,grav_vec);
}

robotKDL::robotKDL(string file_name,vector<string> base_names,vector<string> EE_names,vector<string> link_names,
                   std::vector<double> grav_vector)
{
  
  urdf_file_=file_name;
  KDL::Vector grav_vec(grav_vector[0],grav_vector[1],grav_vector[2]);
  g_vec_=grav_vec;
    // Generate kdl tree:
  if (!kdl_parser::treeFromFile(file_name, robot_tree_))
  {
    cerr<<"Failed to construct kdl tree from urdf file: "<<file_name.c_str()<<endl;
  }
  dof=robot_tree_.getNrOfJoints();

  // Generate chains:
  kdl_chains_.resize(EE_names.size());
  fk_solvers_.resize(EE_names.size());
  dyn_solvers_.resize(EE_names.size());
  jacobian_solvers_.resize(EE_names.size());
  H_mat_.resize(EE_names.size());

  // Generate link chains
  link_kdl_chains_.resize(link_names.size());
  link_fk_solvers_.resize(link_names.size());
  link_jacobian_solvers_.resize(link_names.size());
  // resize data
  jacobians_.resize(EE_names.size());

  for(int i=0;i<EE_names.size();++i)
  {
    string ee_name=EE_names[i];
    if(!robot_tree_.getChain(base_names[i],ee_name,kdl_chains_[i]))
    {
      ROS_ERROR("Failed to construct kdl chain from %s to %s",base_names[i].c_str(),ee_name.c_str());
    }
    // No idea how this works. Switch to boost ptr at some point.

    fk_solvers_[i]=new KDL::ChainFkSolverPos_recursive(kdl_chains_[i]);
    dyn_solvers_[i]= new KDL::ChainDynParam(kdl_chains_[i],g_vec_);
    jacobian_solvers_[i] = new KDL::ChainJntToJacSolver(kdl_chains_[i]);
    // initialize data (allow use in real-time?)
    jacobians_[i] = new KDL::Jacobian(kdl_chains_[i].getNrOfJoints());
    
 
    H_mat_[i] = new KDL::JntSpaceInertiaMatrix(kdl_chains_[i].getNrOfJoints());
#ifdef DEBUG
    cerr<<"Created kinematic and Dynamic chains"<<endl;
#endif
    
  }
  for(int i=0;i<link_names.size();++i)
  {
    string link_name=link_names[i];
    if(!robot_tree_.getChain(base_names[0],link_name,link_kdl_chains_[i]))
    {
      cerr<<"Failed to construct kdl chain from "<<base_names[0].c_str()<<" to "<<link_name.c_str()<<endl;
    }
    // No idea how this works. Switch to boost ptr at some point.

    link_fk_solvers_[i]=new KDL::ChainFkSolverPos_recursive(link_kdl_chains_[i]);
    //dyn_solvers_[i]= new KDL::ChainDynParam(kdl_chains_[i],g_vec_);
    link_jacobian_solvers_[i] = new KDL::ChainJntToJacSolver(link_kdl_chains_[i]);
    // initialize data (allow use in real-time?)
    //jacobians_[i] = new KDL::Jacobian(kdl_chains_[i].getNrOfJoints());
    
 
    //H_mat_[i] = new KDL::JntSpaceInertiaMatrix(kdl_chains_[i].getNrOfJoints());
#ifdef DEBUG
    cerr<<"Created kinematic and Dynamic chains for links "<<i<<" "<<link_names.size()<<endl;
#endif
    
  }
#ifdef DEBUG
    cerr<<"initialized robotkdl!"<<endl;
#endif
  
  //ROS_INFO("initialized robotkdl");

  // This does not work, maybe because the created object is not returned back? seg fault occurs.
  //robotKDL(file_name,base_names,EE_names,grav_vec);
}


robotKDL::robotKDL(string file_name,vector<string> base_names,vector<string> EE_names,
                   KDL::Vector grav_vector)
{
  // TODO: read dof from urdf

  urdf_file_=file_name;  
  g_vec_=grav_vector;
  // Generate kdl tree:
  if (!kdl_parser::treeFromFile(file_name, robot_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree from urdf file: %s",file_name.c_str());
  }
  dof=robot_tree_.getNrOfJoints();

  // Generate chains:
  kdl_chains_.resize(EE_names.size());
  fk_solvers_.resize(EE_names.size());
  dyn_solvers_.resize(EE_names.size());
  jacobian_solvers_.resize(EE_names.size());
  H_mat_.resize(EE_names.size());
  // resize data
  jacobians_.resize(EE_names.size());

  for(int i=0;i<EE_names.size();++i)
  {
    string ee_name=EE_names[i];
    if(!robot_tree_.getChain(base_names[i],ee_name,kdl_chains_[i]))
    {
      ROS_ERROR("Failed to construct kdl chain from %s to %s",base_names[i].c_str(),ee_name.c_str());
    }
    // No idea how this works. Switch to boost ptr at some point.

    fk_solvers_[i]=new KDL::ChainFkSolverPos_recursive(kdl_chains_[i]);
    dyn_solvers_[i]= new KDL::ChainDynParam(kdl_chains_[i],g_vec_);
    jacobian_solvers_[i] = new KDL::ChainJntToJacSolver(kdl_chains_[i]);
    // initialize data (allow use in real-time?)
    jacobians_[i] = new KDL::Jacobian(kdl_chains_[i].getNrOfJoints());
    
 
    H_mat_[i] = new KDL::JntSpaceInertiaMatrix(kdl_chains_[i].getNrOfJoints());
#ifdef DEBUG
    cerr<<"Created kinematic and Dynamic chains"<<endl;
#endif
    
  } 
  //ROS_INFO("initialized robotkdl");

}

robotKDL::robotKDL(string file_name, string base_name, string EE_name, 
                   std::vector<double> grav_vector)
{
  // If seg fault occurs, replace this with the actual function
  robotKDL(file_name,base_name,EE_name,KDL::Vector(grav_vector[0],grav_vector[1],grav_vector[2]));
}

robotKDL::robotKDL(string file_name,string base_name,string EE_name,KDL::Vector grav_vector)
{
  g_vec_=grav_vector;

  // Generate kdl tree:
  if (!kdl_parser::treeFromFile(file_name, robot_tree_))
  {
    ROS_ERROR("Failed to construct kdl tree from urdf file: %s",file_name.c_str());
  }

  // Generate chain:
  kdl_chains_.resize(1);
  if(!robot_tree_.getChain(base_name,EE_name,kdl_chains_[0]))
    ROS_ERROR("Failed to construct kdl chain from %s to %s",base_name.c_str(),EE_name.c_str());

  // No idea how this works. Switch to boost ptr at some point.
  fk_solvers_.push_back(new KDL::ChainFkSolverPos_recursive(kdl_chains_[0]));
  dyn_solvers_.push_back(new KDL::ChainDynParam(kdl_chains_[0],g_vec_));
  jacobian_solvers_.push_back(new KDL::ChainJntToJacSolver(kdl_chains_[0]));

  // initialize data for use in real-time loop
  jacobians_.push_back(new KDL::Jacobian(kdl_chains_[0].getNrOfJoints()));
  H_mat_.push_back(new KDL::JntSpaceInertiaMatrix(kdl_chains_[0].getNrOfJoints()));
#ifdef DEBUG
    cerr<<"Created kinematic and Dynamic chains"<<endl;
#endif
}

vector<double> robotKDL::getFK(int chain_idx,vector<double> joint_pos,bool RPY) const
{
  KDL::JntArray q(joint_pos.size());
  KDL::Frame p_out;
  // std vec to eigen
  Map<Eigen::VectorXd> q_arr(&joint_pos[0],joint_pos.size());
  
  q.data=q_arr;
  //  cerr<<q.data<<endl;
  fk_solvers_[chain_idx]->JntToCart(q,p_out);

  vector<double> pose;
  pose=frame_to_pose_(p_out,RPY);
#ifdef DEBUG
  cerr<<pose[0]<<' '<<pose[1]<<' '<<pose[2]<<' '<<pose[3]<<' '<<pose[4]<<' '<<pose[5]<<' '<<pose[6]<<' '<<endl;
#endif
  return pose;
}
Eigen::MatrixXd robotKDL::getFKEigen(int chain_idx,vector<double> joint_pos) const
{
  KDL::JntArray q(joint_pos.size());
  KDL::Frame p_out;
  // std vec to eigen
  Map<Eigen::VectorXd> q_arr(&joint_pos[0],joint_pos.size());
  
  q.data=q_arr;
  //  cerr<<q.data<<endl;
  fk_solvers_[chain_idx]->JntToCart(q,p_out);
  Eigen::MatrixXd frame_(4,4);
  frame_.setIdentity();
  // translation
  frame_(0,3)=p_out.p[0];
  frame_(1,3)=p_out.p[1];
  frame_(2,3)=p_out.p[2];

  for(int i=0;i<3;++i)
  {
    frame_(i,0)=p_out.M(i,0);
    frame_(i,1)=p_out.M(i,1);
    frame_(i,2)=p_out.M(i,2);
  }
  
  return frame_;
}
vector<double> robotKDL::getLinkPose(vector<double> joint_pos,int link_num) const
{
  vector<double> l_pose;
  KDL::JntArray q(joint_pos.size());
  KDL::Frame p_out;
  // std vec to eigen
  Map<Eigen::VectorXd> q_arr(&joint_pos[0],joint_pos.size());
  //cerr<<q_arr<<endl;
  q.data=q_arr;
  
  link_fk_solvers_[link_num]->JntToCart(q,p_out);
  vector<double> pose;
  pose=frame_to_pose_(p_out,false);
#ifdef DEBUG
    cerr<<"link "<<i<<" pose: "<<p_out.p[0]<<' '<<pose[1]<<' '<<pose[2]<<' '<<pose[3]<<' '<<pose[4]<<' '<<pose[5]<<' '<<pose[6]<<' '<<endl;
#endif

    l_pose=pose;
    return l_pose;
}
vector<vector<double>> robotKDL::getLinkPoses(vector<double> joint_pos) const
{
  vector<vector<double>> r_poses;
  for (int i=0;i<link_fk_solvers_.size();++i)
  {
    KDL::JntArray q(i);
    KDL::Frame p_out;
    // std vec to eigen
    Map<Eigen::VectorXd> q_arr(&joint_pos[0],i);
    //cerr<<q_arr<<endl;
    q.data=q_arr;

    link_fk_solvers_[i]->JntToCart(q,p_out);
    vector<double> pose;
    pose=frame_to_pose_(p_out,false);
#ifdef DEBUG
    cerr<<"link "<<i<<" pose: "<<p_out.p[0]<<' '<<pose[1]<<' '<<pose[2]<<' '<<pose[3]<<' '<<pose[4]<<' '<<pose[5]<<' '<<pose[6]<<' '<<endl;
#endif

    r_poses.push_back(pose);

  }
  return r_poses;
}

void robotKDL::frame_to_pose_(KDL::Frame T,  vector<double> &pose)
{
  pose.resize(7);
  pose[0]=T.p[0];
  pose[1]=T.p[1];
  pose[2]=T.p[2];
    
  T.M.GetQuaternion(pose[3],pose[4],pose[5],pose[6]);    
}

vector<double> robotKDL::frame_to_pose_(KDL::Frame T,bool RPY) const
{
  vector<double> pose;
  if(RPY==true)
    pose.resize(6);
  else
    pose.resize(7);
  pose[0]=T.p[0];
  pose[1]=T.p[1];
  pose[2]=T.p[2];
  if(RPY==true)
    T.M.GetRPY(pose[3],pose[4],pose[5]);
  else
    T.M.GetQuaternion(pose[3],pose[4],pose[5],pose[6]);
  return pose;
}
vector<double> robotKDL::frame_to_pose_(Eigen::MatrixXd T,bool RPY) const
{
  vector<double> pose;
  if(RPY==true)
    pose.resize(6);
  else
    pose.resize(7);
  pose[0]=T(0,3);
  pose[1]=T(1,3);
  pose[2]=T(2,3);
  // Eigen to rotation matrix:
  tf::Matrix3x3 R(T(0,0),T(0,1),T(0,2),
                  T(1,0),T(1,1),T(1,2),
                  T(2,0),T(2,1),T(2,2));
  
  if(RPY==true)
    R.getRPY(pose[3],pose[4],pose[5]);
  else
  {
    tf::Quaternion q;
    R.getRotation(q);
    pose[3]=q.x();
    pose[4]=q.y();
    pose[5]=q.z();
    pose[6]=q.w();
  } 
  return pose;
}
/*
Eigen::VectorXd robotKDL::frame_to_pose_(Eigen::MatrixXd T,bool RPY) const
{
  vector<double> pose=frame_to_pose_(T,RPY);
  Map<Eigen::VectorXd> pose_v(&pose[0],pose.size());
  return pose_v;
}
*/
vector<double> robotKDL::get_q_RPY(double R, double P, double Y) const
{
  //
  tf::Matrix3x3 R_;
  R_.setRPY(R,P,Y);
  tf::Quaternion q;
  R_.getRotation(q);
  vector<double> q_;
  q_.resize(4);
  q_[0]=q.x();
  q_[1]=q.y();
  q_[2]=q.z();
  q_[3]=q.w();
  return q_;
  
}
vector<double> robotKDL::get_RPY_q(double x, double y, double z, double w) const
{
  tf::Matrix3x3 R_;
  tf::Quaternion q(x,y,z,w);
  R_.setRotation(q);
  vector<double> rpy_;
  rpy_.resize(3);
  R_.getRPY(rpy_[0],rpy_[1],rpy_[2]);
  
  return rpy_;
}

vector<double> robotKDL::euler_diff(vector<double> r_1, vector<double> r_d) const
{
  //
  vector<double> orient_diff;
  orient_diff.resize(3);

  KDL::Rotation R;
  KDL::Rotation R_1=R.RPY(r_1[0],r_1[1],r_1[2]);
  KDL::Rotation R_d=R.RPY(r_d[0],r_d[1],r_d[2]);
  KDL::Frame F_1(R_1);
  KDL::Frame F_d(R_d);

  KDL::Twist x_err=KDL::diff(F_1,F_d);

  Eigen::Matrix<double, 6, 1> err;
  tf::twistKDLToEigen(x_err,err);
  orient_diff[0]=err(3,0);
  orient_diff[1]=err(4,0);
  orient_diff[2]=err(5,0);
  /*
  KDL::Vector R_diff;
  R_diff=KDL::diff(R_1,R_d);
  orient_diff[0]=R_diff.x();
  orient_diff[1]=R_diff.y();
  orient_diff[2]=R_diff.z();
  */
  //R_diff.getRPY(orient_diff[0],orient_diff[1],orient_diff[2]);
  return orient_diff;
}
vector<double> robotKDL::euler_diff(Eigen::MatrixXd r_1, Eigen::MatrixXd r_d) const
{
  // get pose from frames:
  vector<double> pose_1=frame_to_pose_(r_1,true);
  vector<double> pose_d=frame_to_pose_(r_d,true);
  
  vector<double> orient_diff;
  orient_diff.resize(3);

  KDL::Rotation R;
  KDL::Rotation R_1=R.RPY(pose_1[3],pose_1[4],pose_1[5]);
  KDL::Rotation R_d=R.RPY(pose_d[3],pose_d[4],pose_d[5]);
  KDL::Frame F_1(R_1);
  KDL::Frame F_d(R_d);

  KDL::Twist x_err=KDL::diff(F_1,F_d);

  Eigen::Matrix<double, 6, 1> err;
  tf::twistKDLToEigen(x_err,err);
  orient_diff[0]=err(3,0);
  orient_diff[1]=err(4,0);
  orient_diff[2]=err(5,0);
  //R_diff.getRPY(orient_diff[0],orient_diff[1],orient_diff[2]);
  return orient_diff;
}
/*
KDL::Frame robotKDL::pose_to_frame(vector<double> pose,int i) const
{
  KDL::Frame f;
  return f;
}

KDL::Frame robotKDL::pose_to_frame(geometry_msgs::Pose pose) const
{
  KDL::Frame f;
  return f;
}
*/
Eigen::MatrixXd robotKDL::pose_to_frame(vector<double> pose) const
{
  Eigen::MatrixXd mat(4,4);
  mat.setIdentity();
  tf::Matrix3x3 R;
  if(pose.size()==6)
  {
    R.setRPY(pose[3],pose[4],pose[5]);
  }
  else
  {
    tf::Quaternion q(pose[3],pose[4],pose[5],pose[6]);
    R.setRotation(q);
  }
  
  for (int i=0;i<3;i++)
  {
    mat(i,0)=R[i].x();
    mat(i,1)=R[i].y();
    mat(i,2)=R[i].z();
  }
  mat(0,3)=pose[0];
  mat(1,3)=pose[1];
  mat(2,3)=pose[2];
  
  return mat;
}
Eigen::MatrixXd robotKDL::pose_to_frame(Eigen::VectorXd pose) const
{
  Eigen::MatrixXd mat(4,4);
  mat.setIdentity();
  tf::Matrix3x3 R;
  if(pose.size()==6)
  {
    R.setRPY(pose[3],pose[4],pose[5]);
  }
  else
  {
    tf::Quaternion q(pose[3],pose[4],pose[5],pose[6]);
    R.setRotation(q);
  }
  
  for (int i=0;i<3;i++)
  {
    mat(i,0)=R[i].x();
    mat(i,1)=R[i].y();
    mat(i,2)=R[i].z();
  }
  mat(0,3)=pose[0];
  mat(1,3)=pose[1];
  mat(2,3)=pose[2];
  
  return mat;
  
}

vector<double> robotKDL::getGtau(int chain_idx,vector<double> joint_pos)
{
  KDL::JntArray q(joint_pos.size());
  KDL::JntArray tau_g(joint_pos.size());
  // std vec to eigen
  Map<Eigen::VectorXd> q_arr(&joint_pos[0],joint_pos.size());
  
  q.data=q_arr;
  dyn_solvers_[chain_idx]->JntToGravity(q,tau_g);
  vector<double> tau;
  tau.resize(joint_pos.size());
  for (int i =0;i<tau.size();++i)
  {
    tau[i]=tau_g.data[i];
#ifdef DEBUG
    cerr<<tau[i]<<endl;
#endif
  }
  return tau;
}
  
vector<double> robotKDL::getCtau(int chain_idx,vector<double> joint_pos,vector<double> joint_vel)
{
  KDL::JntArray q(joint_pos.size());
  KDL::JntArray q_dot(joint_vel.size());
  KDL::JntArray tau_c(joint_pos.size());
  // std vec to eigen
  Map<Eigen::VectorXd> q_arr(&joint_pos[0],joint_pos.size());
  Map<Eigen::VectorXd> q_dot_arr(&joint_vel[0],joint_vel.size());
  
  q.data=q_arr;
  q_dot.data=q_dot_arr;
  dyn_solvers_[chain_idx]->JntToCoriolis(q,q_dot,tau_c);
  vector<double> tau;
  tau.resize(joint_pos.size());
  for (int i =0;i<tau.size();++i)
  {
    tau[i]=tau_c.data[i];
#ifdef DEBUG
    cerr<<tau[i]<<endl;
#endif
  }
  return tau;
}

void robotKDL::urdfParser()
{
  // function updates bounds by using urdf parser:

  urdf::Model model;
  if (!model.initFile(urdf_file_))
  {
    ROS_ERROR("Failed to parse urdf file");
  }
  dof=0;
  for(auto it = model.joints_.begin(); it != model.joints_.end(); ++it)
  {
    if (model.joints_[it->first]->type != urdf::Joint::UNKNOWN && model.joints_[it->first]->type != urdf::Joint::FIXED)
    {
      low_bounds.emplace_back(model.joints_[it->first]->limits->lower);
      up_bounds.emplace_back(model.joints_[it->first]->limits->upper);
      dof+=1;
    }
  }
  // cerr<<"read joint limits, dof: "<<dof<<endl;
}
Eigen::Matrix<double, 6, Eigen::Dynamic> robotKDL::getJacobian(int idx_chain, 
                                                                 std::vector<double> jnt_array) const
{
  KDL::JntArray q(jnt_array.size());
  // std vec to eigen
  Map<Eigen::VectorXd> q_arr(&jnt_array[0],jnt_array.size());
  
  q.data=q_arr;

  jacobian_solvers_[idx_chain]->JntToJac(q,*jacobians_[idx_chain]);

  return jacobians_[idx_chain]->data;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> robotKDL::getLinkJacobian(int idx_link, 
                                                                 std::vector<double> jnt_array) const
{
  
  KDL::JntArray q(jnt_array.size());
  // std vec to eigen
  Map<Eigen::VectorXd> q_arr(&jnt_array[0],jnt_array.size());
  //cerr<<q_arr<<endl;
  q.data=q_arr;
  KDL::Jacobian l_jacobians_(link_kdl_chains_[idx_link].getNrOfJoints());

  int er=link_jacobian_solvers_[idx_link]->JntToJac(q,l_jacobians_);
  //cerr<<idx_link<<' '<<q_arr.rows()<<' '<<link_kdl_chains_[idx_link].getNrOfJoints()<<' '<<er<<endl;
  //cerr<<link_jacobian_solvers_[idx_link]->strError(er)<<endl;
  // KDL::ChainJntToJacSolver::strError
  //cerr<<l_jacobians_.data.cols()<<endl;
  //cerr<<"KDL:computed link jacobian"<<endl;
  return l_jacobians_.data;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic >robotKDL::getM(int idx_chain,
                                                                     std::vector<double> jnt_array)
{
  KDL::JntArray q(jnt_array.size());
  Map<Eigen::VectorXd> q_arr(&jnt_array[0],jnt_array.size());
  q.data = q_arr;
  dyn_solvers_[idx_chain]->JntToMass(q,*H_mat_[idx_chain]);
  return H_mat_[idx_chain]->data;
}



} // namespace manipulator_kdl

