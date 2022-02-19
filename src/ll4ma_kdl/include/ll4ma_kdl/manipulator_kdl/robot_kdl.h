/*
 * Copyright (C) 2018 LL4MA lab, University of Utah
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted 
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions 
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of 
 * conditions and the following disclaimer in the documentation and/or other materials provided with 
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to 
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ROBOT_KDL_H
#define ROBOT_KDL_H

#include "debug.h"
#include<iostream>

#include <boost/shared_ptr.hpp>
// ros
#include<ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
// headers from kdl
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chaindynparam.hpp>
// Use local jacdotsolver if on 16.04 (not included in that version of KDL)
#ifdef XENIAL_BUILD
  #include "manipulator_kdl/chainjnttojacdotsolver.hpp"
#else
  #include <kdl/chainjnttojacdotsolver.hpp>
#endif

// kdl parser to read urdf
#include <kdl_parser/kdl_parser.hpp>

// Eigen library for vectors and matrices:
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <unsupported/Eigen/MatrixFunctions>

// urdf parser:
#include <urdf/model.h>
#include <geometry_msgs/Pose.h>

#include <fstream>
#include <streambuf>
#include <string>
#include <cerrno>

namespace manipulator_kdl
{

  class robotKDL
  {
  public:

    // Constructors:
    // TODO: there are too many, maybe remove few?
    robotKDL();
    
    // Use below constructor to build a kdl tree for a manipulator:
    robotKDL(std::string ros_param_name, ros::NodeHandle &n,std::vector<std::string> base_names,
             std::vector<std::string> EE_names, std::vector<double> grav_vector);

    // Use below constructor when you also want chains from base to different links:
    robotKDL(std::string robot_desc_param, ros::NodeHandle &n, std::vector<std::string> base_names,
             std::vector<std::string> EE_names, std::vector<std::string> link_names, std::vector<double> grav_vector);

    robotKDL(std::string, std::vector<std::string>, std::vector<std::string>, std::vector<double>);
    robotKDL(std::string, std::vector<std::string>, std::vector<std::string>, std::vector<std::string>,std::vector<double>);
    robotKDL(std::string, std::vector<std::string>, std::vector<std::string>, KDL::Vector);
    robotKDL(std::string, std::string,std::string, std::vector<double>);
    robotKDL(std::string, std::string,std::string, KDL::Vector);


    // Functions for initialization:
    void manipulator_init(std::string &base_name, std::string &EE_name);
    void robot_init(std::vector<std::string> &base_names, std::vector<std::string> &EE_names);
    void read_tree_param(std::string robot_desc_param, ros::NodeHandle &n);
    void read_tree_file(std::string urdf_file);
    void add_link_chains(std::vector<std::string> base_link_names, std::vector<std::string> link_names);
    void update_g_vec(std::vector<double> &grav_vec);

    // Ideally need to be private variables with functions to update them
    // dof and bounds are currently read from the urdf
    // f_dof is manually set.

    // robot parameters:
    int dof;
    int f_dof;// finger dof
    int hand_dof;
    int arm_dof;
    std::vector<double> low_bounds;
    std::vector<double> up_bounds;
    KDL::Vector g_vec_;

    // Kinematic Functions
    // FK, IK, Jacobian

    /// Returns FK
    void getFK(const int &chain_idx, const Eigen::VectorXd &j_arr, Eigen::VectorXd &out_pose,
               const bool &RPY=false) const;

    // To return T mat:
    void getFK(const int &chain_idx, const Eigen::VectorXd &j_arr, Eigen::MatrixXd &T_mat) const;
    void getFK(const int &chain_idx, const Eigen::VectorXd &joint_pos, Eigen::Affine3d &frame) const;


    /**
     * Computes task space pose and velocity error.
     *
     * @param chain_idx    index of chain to use for FK
     * @param joint_pos    current joint positions
     * @param joint_vel    current joint velocities
     * @param task_pos_des desired task space pose
     * @param task_vel_des desired task space velocity (twist)
     * @param task_pos_err (returned)
     * @param task_vel_err (returned)
     */
    void getTaskError(const int &chain_idx,
                      const Eigen::VectorXd &joint_pos, const Eigen::VectorXd &joint_vel,
                      const Eigen::Affine3d &task_pos_des, const Eigen::VectorXd &task_vel_des,
                      Eigen::VectorXd &task_pos_err, Eigen::VectorXd &task_vel_err);
    
    void getJacobian(const int&, const Eigen::VectorXd &j, Eigen::MatrixXd &J_mat) const;

    void getJacobianDot(const int&, const Eigen::VectorXd &j, const Eigen::VectorXd &k, Eigen::MatrixXd &J_mat) const;

    // Link chain kinematics:
    void getLinkPoses(const Eigen::VectorXd &j, std::vector<Eigen::VectorXd> &cart_poses) const;
    void getLinkPoses(const Eigen::VectorXd &joint_pos, std::vector<std::vector<double>> &cart_poses) const;
    void getLinkPose(const int &link_num, const Eigen::VectorXd &joint_pos,
                     Eigen::VectorXd &out_pose,const bool &RPY=false) const;
    void getLinkPose(const int &link_num, const Eigen::VectorXd &joint_pos,
                     Eigen::MatrixXd &out_frame) const;

    void getLinkJacobian(const int&, const Eigen::VectorXd &j, Eigen::MatrixXd &J_mat) const;


    // Dynamic Functions
    // G matrix (stores value in tau_g_ for gravity compensation):
    void getGtau(const int&, const Eigen::VectorXd &j, Eigen::VectorXd &tau_g_);
    // Coriolis (stores value in tau_c_ for compensation):
    void getCtau(const int&, const Eigen::VectorXd &j_pos, const Eigen::VectorXd &j_vel,
                 Eigen::VectorXd &tau_c_) const;
    // Inertia Matrix
    void getM(const int&, const Eigen::VectorXd &j_pos, Eigen::MatrixXd &M) const;

    // helper functions to convert between kdl frame and pose:
    // TODO currently has extra overloaded types, should be streamlined later.

    // Converts a kdl frame to a pose [x y z wx wy wz w]
    void frame_to_pose_(KDL::Frame&, std::vector<double>&) const;
    void frame_to_pose_(KDL::Frame&, Eigen::VectorXd &out_pose, const bool &RPY) const;
    void frame_to_pose_(const Eigen::MatrixXd&, std::vector<double> &out_pose, const bool &RPY=true) const;
    void frame_to_pose_(const Eigen::MatrixXd &T, Eigen::VectorXd &pose, const bool &RPY) const;

    void pose_to_frame(const std::vector<double> &pose, Eigen::MatrixXd &mat)const;
    void pose_to_frame(const Eigen::VectorXd &pose, Eigen::MatrixXd &mat)const;

    Eigen::Vector3d get_RPY(Eigen::MatrixXd T);
    /** 
     * returns the joint angle limits for the joints in the chain_idx
     * 
     * @param chain_idx  chain to use for getting joint limits
     * @param max_bounds returned upper joint limit
     * @param min_bounds returned lower joint limit
     */
    void getJointLimits(const int &chain_idx,std::vector<double> &max_bounds, std::vector<double> &min_bounds);

    /** 
     * returns the joint angles limits in a vector indexed by j_names
     * 
     * @param j_names 
     * @param max_bounds 
     * @param min_bounds 
     */
    void getJointLimits(std::vector<std::string> &j_names,std::vector<double> &max_bounds, std::vector<double> &min_bounds);

    void urdfParser();
    std::string get_file_contents(const char *filename);
    std::vector<double> get_RPY_q(const double &x, const double &y, const double &z,
                             const double &w) const;
    std::vector<double> get_q_RPY(const double &R, const double &P, const double &Y) const;
    void euler_diff(const std::vector<double> &r_1, const std::vector<double> &r_d,
                    std::vector<double> &orient_err) const;
    void euler_diff(const Eigen::MatrixXd &r_1, const Eigen::MatrixXd &r_d,
                    std::vector<double> &orient_err) const;
    void getSTDVector(const Eigen::VectorXd &in, std::vector<double> &out) const;

    /*
     * Return all joint names associated with manipulator (aggregated over all chains).
     */
    std::vector<std::string> getJointNames();

    bool updateJointStateNames(std::vector<std::string> j_names);
    

    /*
     * Return joint names associated with a particular chain.
     */
    std::vector<std::string> getJointNames(int chain_idx);

    /*
     * Return joint names associated with particular chains.
     */
    std::vector<std::string> getJointNames(std::vector<int> &chain_idxs);

    /**
     * Returns the name of the root of a chain.
     *
     * @param chain_idx the chain index to return
     * @param root_name (returned) the name of the root
     */
    void getRootName(int chain_idx, std::string& root_name);

 
    /**
     * Returns the number of KDL chains managed.
     * 
     * @return the number of KDL chains
     */
    int getNumChains();

    /** 
     * Returns the number of KDL links 
     * 
     * 
     * @return the number of KDL links
     */

    int getNumLinks();

    void getEEChainState(const int &chain_num,const Eigen::VectorXd &joint_state, Eigen::VectorXd &ee_js) const;

    /** 
     * Given a robot joint position array(joint_state), this function picks the indices that match for a link chain(link_num). The indices are obtained from link_joint_idx[link_num]
     * 
     * @param link_num 
     * @param joint_state 
     * @param link_js 
     */
    void getLinkChainState(const int &link_num,const Eigen::VectorXd &joint_state, Eigen::VectorXd &link_js) const;

    /** 
     * Initialization function that generates link_joint_idx,ee_joint_idx for the link_kdl_chains, kdl_chains_.
     * 
     */
    bool generateChainJointidx();

    std::vector<std::vector<int>> link_joint_idx_,ee_joint_idx_;

    /** 
     * Compute the jacobian with respect to a point on the robot's link
     * 
     * @param link_idx  link on which the point is present
     * @param j joint positions
     * @param pt_position position of point in link_idx frame
     * @param J_mat returned Jacobian
     */

    void getLinkPtJacobian(const int& link_idx,const Eigen::VectorXd &j,const Eigen::VectorXd &pt_position, Eigen::MatrixXd &J_mat) const;
    /// get Jacobian with respect to a contact point on a link:
    Eigen::MatrixXd getLinkPtJacobian(int l_idx, Eigen::VectorXd q_state, Eigen::VectorXd c_pt) const;
    /// get Jacobian with respect to a contact point on an end-effector:
    void getEEPtJacobian(const int &ee_idx, const Eigen::VectorXd &j, const Eigen::VectorXd &pt_position, Eigen::MatrixXd &J_mat) const;

    void getPseudoInverse(const Eigen::MatrixXd &in_mat, Eigen::MatrixXd &out_inv,const double &tolerance=1e-5) const;
  //  private:
    //Private functions to simplify initialization...
    // initializing object function:
    std::string urdf_file_, urdf_string_;
    // kdl variables
    KDL::Tree robot_tree_;
    std::vector<KDL::Jacobian*> jacobians_;
    std::vector<KDL::Jacobian*> jacobian_dots_;
    std::vector<KDL::JntSpaceInertiaMatrix*> H_mat_;

    // chains for end effector:
    std::vector<KDL::Chain> kdl_chains_;
    // chains for links:
    std::vector<KDL::Chain> link_kdl_chains_;

    // joint names stored for each chain TODO: rename to chain_jnt_names_ ??
    std::vector<std::vector<std::string>> jnt_names_;

    // joint names in the same order as a joint state (useful for computing fk,jacobian of links)
    // TODO: also setup for chains.
    std::vector<std::string> link_jnt_names_;
    std::vector<std::string> robot_js_names_;
    // kinematic solvers:
    std::vector<boost::shared_ptr<KDL::ChainFkSolverPos>> fk_pos_solvers_;
    std::vector<boost::shared_ptr<KDL::ChainFkSolverVel>> fk_vel_solvers_;
    std::vector<boost::shared_ptr<KDL::ChainJntToJacSolver>> jacobian_solvers_;
    std::vector<boost::shared_ptr<KDL::ChainJntToJacDotSolver>> jacobian_dot_solvers_;

    // kinematic solvers for links:
    std::vector<boost::shared_ptr<KDL::ChainFkSolverPos>> link_fk_solvers_;
    std::vector<boost::shared_ptr<KDL::ChainJntToJacSolver>> link_jacobian_solvers_;

    // Dynamic solvers
    std::vector<boost::shared_ptr<KDL::ChainDynParam>> dyn_solvers_;
  };
}
#endif
