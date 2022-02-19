/*
 * Copyright (C) 2017  LL4MA lab, University of Utah
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

#include "debug.h"
#include<iostream>


// ros
#include<ros/ros.h>
#include <tf/transform_datatypes.h>
// headers from kdl
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
// kdl parser to read urdf
#include <kdl_parser/kdl_parser.hpp>

// Eigen library for vectors and matrices:
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
// urdf parser:
#include <urdf/model.h>
#include <geometry_msgs/Pose.h>
// string, cout..
using namespace std;
using namespace Eigen;
namespace manipulator_kdl_bck
{
  class robotKDL
  {
  public:
    // Constructor to load a tree and chains from file:
    robotKDL();

    robotKDL(string ros_param_name,ros::NodeHandle &n,vector<string> base_names,vector<string> EE_names,
             std::vector<double> grav_vector);

    robotKDL(string,vector<string>,vector<string>,vector<double>);
    robotKDL(string,vector<string>,vector<string>,vector<string>,vector<double>);

    robotKDL(string,vector<string>,vector<string>,KDL::Vector);
    robotKDL(string,string,string,vector<double>);
    robotKDL(string,string,string,KDL::Vector);
  

    // robot parameters:
    int dof;
    int f_dof;// finger dof
    vector<double> low_bounds;
    vector<double> up_bounds;

    // Kinematic Functions
    // FK, IK, Jacobian
    vector<double> getFK(int,vector<double>,bool RPY=false) const;// return the position and quaternion


    // Eigen overloading is ambiguos to compiler, hence making second FK function:
    Eigen::MatrixXd getFKEigen(int,vector<double>) const;// return the position and quaternion
    
    Eigen::Matrix< double, 6, Eigen::Dynamic > getJacobian(int,std::vector<double>) const; // return jacobian
    vector<vector<double>> getLinkPoses(vector<double>) const;
    vector<double> getLinkPose(vector<double> joint_pos,int link_num) const;
    Eigen::Matrix< double, 6, Eigen::Dynamic > getLinkJacobian(int,std::vector<double>) const; // return jacobian
    
    // Pseudo Inverse /

   
    template<typename Derived1, typename Derived2>
    void getPseudoInverse(const Eigen::MatrixBase<Derived1> &m,
                          Eigen::MatrixBase<Derived2> &m_pinv,
                          double tolerance) const
    {
      using namespace Eigen;
      
      JacobiSVD<typename Derived1::PlainObject> svd(m, ComputeFullU | ComputeFullV);
      typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType sing_vals = svd.singularValues();
      // set values within tolerance to zero
      for (int idx = 0; idx < sing_vals.size(); idx++)
      {
        if (tolerance > 0.0 && sing_vals(idx) > tolerance)
          sing_vals(idx) = 1.0 / sing_vals(idx);
        else
          sing_vals(idx) = 0.0;
      }
      
      m_pinv = svd.matrixV().leftCols(sing_vals.size()) * sing_vals.asDiagonal() * svd.matrixU().leftCols(sing_vals.size()).transpose();
    }

    // Dynamic Functions
    // G matrix:
    vector<double> getGtau(int,vector<double>);// return tau for gravity compensation
    // Coriolis :
    vector<double> getCtau(int,vector<double>,vector<double>);// return tau for compensation
    // Inertia Matrix
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> getM(int, vector<double>);


    Eigen::MatrixXd pose_to_frame(vector<double> pose)const;
    Eigen::MatrixXd pose_to_frame(Eigen::VectorXd pose)const;
    /*
    KDL::Frame pose_to_frame(vector<double> pose,int) const;
    KDL::Frame pose_to_frame(geometry_msgs::Pose pose) const;
    */
    void frame_to_pose_(KDL::Frame,vector<double>&);// Converts a kdl frame to a pose [x y z wx wy wz w] 
    vector<double> frame_to_pose_(KDL::Frame,bool RPY=false) const;
    vector<double> frame_to_pose_(Eigen::MatrixXd,bool RPY=false) const;
    //Eigen::VectorXd frame_to_pose_(Eigen::MatrixXd T,bool RPY) const;

    
    void urdfParser();
    vector<double> get_RPY_q(double x, double y, double z, double w) const;
    vector<double> get_q_RPY(double R, double P, double Y) const;
    vector<double> euler_diff(vector<double> r_1, vector<double> r_d) const;
    vector<double> euler_diff(Eigen::MatrixXd r_1, Eigen::MatrixXd r_d) const;
  private:
    //Private functions to simplify initialization...
    
    
    string urdf_file_;
    // kdl variables
    KDL::Tree robot_tree_;
    vector<KDL::Jacobian*> jacobians_;
    vector<KDL::JntSpaceInertiaMatrix*> H_mat_;
  
    // chains for end effector:
    vector<KDL::Chain> kdl_chains_;
    // chains for links:
    vector<KDL::Chain> link_kdl_chains_;
 
    // kinematic solvers:
    vector<KDL::ChainFkSolverPos*> fk_solvers_;
    vector<KDL::ChainJntToJacSolver*> jacobian_solvers_;
    
    // kinematic solvers for links:
    vector<KDL::ChainFkSolverPos*> link_fk_solvers_;
    vector<KDL::ChainJntToJacSolver*> link_jacobian_solvers_;
 
    // Dynamic solvers
    KDL::Vector g_vec_;
    vector<KDL::ChainDynParam*> dyn_solvers_;
    //ros::NodeHandle* node;
  
  };
}
