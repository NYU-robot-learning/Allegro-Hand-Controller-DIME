# This file creates an instance of the KUKA LBR4 robot with FK and Jacobian using KDL.
import rospy
from rospkg import RosPack
import manipulator_model
import numpy as np
from numpy import cos,sin
from numpy.linalg import inv
from numpy.linalg import pinv
import PyKDL

class lbr4Kinematics:
    def __init__(self,delta_t=1.0/60.0,T=1):
        ''' delta_t= timestep
            T= total time steps
            fingers=[0-4],0=index,1=middle,2=ring,3=thumb        
            The model assumes all fingers have same number of joints.
        '''
        self.n=6
        self.m=7
        self.n_links=8
        self.T=T
        self.delta_t=delta_t
        rp = RosPack()
        packages = rp.list()
        path = rp.get_path('urlg_robots_description')
        #path='../urdf/allegro_right/allegro_hand_description_right.urdf'
        path= path + '/debug/lbr4_kdl.urdf'
        self.kdlModel=manipulator_model.ManipulatorSimpleModel(path,base_link='lbr4_0_link')
        low_bounds=[]
        up_bounds=[]
        
        low_bounds+=list(self.kdlModel.chain.get_joint_limits()[0])
        up_bounds+=list(self.kdlModel.chain.get_joint_limits()[1])
        self.bounds=np.array([low_bounds,up_bounds])
        self.link_offset=[[0.0,0.0,0.0],[0.0,0.0,0.0],
                          [0.0,0.0,0.0],[0.0,0.0,0.],
                          [0.0,0.0,0.0],[0.0,-0.0,0.],
                          [0.0,0.0,0.0],[0.0,0.0,0.0]]

        #self.link_offset=[[0.0,0.0,0.0],[0.0,0.05,0.20],
        #                  [0.0,-0.05,0.02],[0.0,-0.05,0.2],
        #                  [0.0,0.05,0.019],[0.0,0.04,0.18],
        #                  [0.0,-0.03,0.0],[0.0,0.0,0.0]]
        
        self.DOF=7
       
    def end_effector_pose_array(self,u,get_quaternion=False):
        T=self.kdlModel.FK(u)
        if(get_quaternion):
            pose=np.zeros(7)
        else:
            pose=np.zeros(6)
        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])
        if(get_quaternion):
            pose[3:7]=R.GetQuaternion()
        else:
            pose[3:6]=R.GetRPY()
        pose[0:3]=T[0:3,3].ravel()
        return pose

    def link_pose_array(self,u,link_num):
        T=self.kdlModel.FK_joint(u[0:link_num],link_num)
        pose=np.zeros(7)
        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])

        # Normalize quaternion:
        quat=R.GetQuaternion()
        #print np.linalg.norm(quat)
        #quat=quat/np.linalg.norm(quat)

        pose[3:7]=quat
        pose[0:3]=T[0:3,3].ravel()+self.link_offset[link_num]
        return pose

    def all_link_poses(self,u):
        r_link_poses=[]
        offset=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        for i in range(self.n_links):
            r_pose=self.link_pose_array(u,i)
            r_link_poses.append(r_pose+offset)
        return r_link_poses
    
    def predict_T(self,x0,u,w):
        # Predicts the next state
        T=self.T
        x_n=np.array([[0.0 for z in range(self.n)] for i in range(T)])
        for i in range(T):
            x_n[i,:]=self.predict(np.ravel(u[i,:]))
        return x_n

    def predict(self,u):
        x_k_plus_1=self.end_effector_pose_array(u)
        
        return x_k_plus_1

    def end_effector_pose(self,u,EE_index=0):

        T=self.kdlModel.FK(u,EE_index)
        return T


    def jacobian(self,u):
        J=self.kdlModel.Jacobian(u)[0:3,:]
        return J
    def jacobian_orient(self,u):
        J=self.kdlModel.Jacobian(u)[3:6,:]
        return J
    def jacobian_full(self,u):
        J=self.kdlModel.Jacobian(u)
        return J
    
    def link_jacobian(self,u,link_index):
        J=self.kdlModel.Jacobian_joint(u[0:link_index],link_index)
        return J

    def predict_xdot(self,x,u):
        # given the current object pose and the new input, find the velcity
        #des_finger_pose=np.ravel(self.linear_finger_poses[t_step])
        x_pose=self.end_effector_pose_array(u[3*4:3*4+4])
        x_dot=(x-x_pose)/self.delta_t
        x_new=x_dot
        return x_new

    
