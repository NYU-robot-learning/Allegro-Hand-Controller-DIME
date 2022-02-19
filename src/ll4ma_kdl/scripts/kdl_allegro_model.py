# This file creates an instance of the allegro robot with FK and Jacobian using KDL.
import rospy
from rospkg import RosPack
import handModel
import numpy as np
from numpy import cos,sin
from numpy.linalg import inv
from numpy.linalg import pinv
import PyKDL

class allegroKinematics:
    def __init__(self,delta_t=1.0/60.0,T=1,fingers=[0,3],fing=3,bounds=[[],[]]):
        ''' delta_t= timestep
            T= total time steps
            fingers=[0-4],0=index,1=middle,2=ring,3=thumb        
            The model assumes all fingers have same number of joints.
        '''
        self.n=6
        self.m=4*4
        self.T=T
        self.delta_t=delta_t
        self.fingers=np.array(fingers)
        self.fing=3
        rp = RosPack()
        packages = rp.list()
        path = rp.get_path('ll4ma_robots_description')
        #path='../urdf/allegro_right/allegro_hand_description_right.urdf'
        path= path + '/urdf/allegro_right/allegro_hand_description_right.urdf'
        self.kdlModel=handModel.HandSimpleModel(path)
        low_bounds=[]
        up_bounds=[]
        for i in xrange(4):
            low_bounds+=list(self.kdlModel.finger_chains[i].get_joint_limits()[0])
            up_bounds+=list(self.kdlModel.finger_chains[i].get_joint_limits()[1])
        self.bounds=np.array([low_bounds+bounds[0],up_bounds+bounds[1]])
        #print self.bounds
        self.DOF=4
        self.FINGERS=4
   
    

    '''
    def contact_points(self,joints,obj_frames):
        #finger=0
        for finger in range(4):        
            t_T_0=self.end_effector_pose(joints[finger*4:finger*4+4],finger)
            t_T_o=t_T_0*inv(obj_frames[finger])
            print t_T_o
        return 0
    '''
    def contact_points(self,des_obj_mat):
       
        des_contact_frames=[[np.eye(4)]for i in range(4)]
        des_contact_points=[[np.zeros(3)] for i in range(4)]
        for i in self.fingers:
            cp=np.zeros(4)
            cp[3]=1.0
            cp[0:3]=np.ravel(self.contact_frames[i][0:3,3])
            des_contact_frames[i]=des_obj_mat*self.contact_frames[i]
            des_contact_points[i]=(np.matrix(des_obj_mat)*np.matrix(cp).T)[0:3]
        self.des_contact_frames=des_contact_frames
        self.des_contact_points=des_contact_points

    def sampling_contact_points(self,des_obj_pose,init_obj_pose):
        T=self.T

        time_contacts=[]
        for k in range(T):
            sample_pose=init_obj_pose+(des_obj_pose-init_obj_pose)*float(k)/T
            # Building transformation matrix:
            x_des_mat=np.eye(4)
            x_des_mat[0:3,3]=sample_pose[0:3]
            R=PyKDL.Rotation.RPY(sample_pose[3],sample_pose[4],sample_pose[5])
            for i in range(3):
                for j in range(3):
                    x_des_mat[i,j]=R[i,j]
        

            des_contact_frames=[[np.eye(4)]for i in range(4)]
            des_contact_points=[[np.zeros(3)] for i in range(4)]
            
            for i in self.fingers:
                cp=np.zeros(4)
                cp[3]=1.0
                cp[0:3]=np.ravel(self.contact_frames[i][0:3,3])
                des_contact_frames[i]=x_des_mat*self.contact_frames[i]
                des_contact_points[i]=(np.matrix(x_des_mat)*np.matrix(cp).T)[0:3]
            time_contacts.append(des_contact_frames)
        
        self.time_contacts=time_contacts

    def end_effector_pose_array(self,u,EE_index):
        T=self.kdlModel.FK(u,EE_index)
        pose=np.zeros(6)
        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])
        pose[3:6]=R.GetRPY()
        pose[0:3]=T[0:3,3].ravel()
        return pose

    def predict_T(self,x0,u):
        # Predicts the next state
        T=self.T
        x_n=np.array([[0.0 for z in range(self.m)] for i in range(T)])
        x_n[0,:]=x0
        for i in range(1,T):
            x_n[i,:]=self.predict(x_n[i-1,:],np.ravel(u[i,:]))
        return x_n

    def predict(self,x,u):
        x_k_plus_1=np.array(u)
        return x_k_plus_1

    def end_effector_pose(self,u,EE_index=0):

        T=self.kdlModel.FK(u,EE_index)
        return T

    def end_effector_pose_T(self,x0,u,EE_index=0):
        T=self.T
        x_n=np.array([[0.0 for z in range(3)] for i in range(T)])
        x_n[0,:]=x0
        for i in range(1,T):
            x_n[i,:]=np.ravel(self.end_effector_pose(np.ravel(u[i,0:4]),EE_index)[0:3,3])
        return x_n

    def jacobian(self,u,EE_index=0):
        J=self.kdlModel.Jacobian(u,EE_index)[0:3,:]
        return J
    def jacobian_orient(self,u,EE_index=0):
        J=self.kdlModel.Jacobian(u,EE_index)[3:6,:]
        return J
    def jacobian_full(self,u,EE_index=0):
        J=self.kdlModel.Jacobian(u,EE_index)
        return J

    def predict_xdot(self,x,u):
        # given the current object pose and the new input, find the velcity
        #des_finger_pose=np.ravel(self.linear_finger_poses[t_step])
        x_pose=self.end_effector_pose_array(u[3*4:3*4+4],3)
        x_dot=(x-x_pose)/self.delta_t
        x_new=x_dot
        return x_new


    def get_T(self,pose):
        T=np.eye(4)
        R=PyKDL.Rotation()
        R=R.Quaternion(pose[3],pose[4],pose[5],pose[6])
        print R
        for i in range(3):
            for j in range(3):
                T[i,j]=R[i,j]
        T[0,3]=pose[0]
        T[1,3]=pose[1]
        T[2,3]=pose[2]

        return T
