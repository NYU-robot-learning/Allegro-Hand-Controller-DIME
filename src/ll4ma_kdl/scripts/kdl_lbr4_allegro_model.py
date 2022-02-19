import numpy as np
from handArmModel import handArmModel
import PyKDL
from rospkg import RosPack
rp=RosPack()
rp.list()
path_urdf=rp.get_path('urlg_robots_description')+'/robots/'


class lbr4_allegro_model:
    def __init__(self,T=1,delta_t=1.0/60):
        # import class:
        file_=path_urdf+'lbr4_allegro_right.urdf'
        
        print "Opening "+file_
        self.kdlModel=handArmModel(file_)

        self.m=7+4*4
        self.n=6
        self.delta_t=delta_t
        self.T=T
        #bounds:
        low_bounds=[]
        up_bounds=[]
        
        # Add joint limits of kuka lbr4
        low_bounds+=list(self.kdlModel.arm_chain.get_joint_limits()[0])
        up_bounds+=list(self.kdlModel.arm_chain.get_joint_limits()[1])
        # Add joint limits of allegro hand:
        for i in range(len(self.kdlModel.arm_finger_chains)):
            low_bounds+=list(self.kdlModel.arm_finger_chains[i].get_joint_limits()[0][7:])
            up_bounds+=list(self.kdlModel.arm_finger_chains[i].get_joint_limits()[1][7:])
        
        
        self.bounds=np.array([low_bounds,up_bounds])

        # This is the list of links we will use for collision checking:
        self.c_list=[1,2,3,5,6,7,9,10,11,14,15,23]
    def predict(self,u,tip_ind=0):
        x_k_plus_1=self.palm_pose_array(u[:7])
        
        return x_k_plus_1

    def predict_T(self,x0,u,w,tip_ind=0):
        # Predicts the next state
        T=self.T
        x_n=np.array([[0.0 for z in range(self.n)] for i in range(T)])
        for i in range(T):
            x_n[i,:]=self.predict(np.ravel(u[i,:]),tip_ind)
        return x_n

    def palm_pose_array(self,u,get_quaternion=False):
        # u contains only the joints of the kuka
        T=self.kdlModel.FK_arm(u)
        
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

    def palm_jacobian(self,u):
        J=self.kdlModel.Jacobian_arm(u)
        return J

    def ftip_pose_array(self,u,tip_index,get_quaternion=False):
        u_=np.zeros(11)
        u_[0:7]=u[0:7]
        u_[7:]=u[tip_index*4:tip_index*4+4]

        
        T=self.kdlModel.FK_arm_ftip(u_,tip_index)
        
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

    def ftip_jacobian(self,u,tip_index):
        # u contains all the joints, we only send the arm joints and finger joints to the function below
        u_=np.zeros(11)
        u_[0:7]=u[0:7]
        u_[7:]=u[tip_index*4:tip_index*4+4]
        J=self.kdlModel.Jacobian_arm_ftip(u_,tip_index)
        return J

    def collision_link_poses(self,u):
        # This function obtains pose of links that will be sent to the collision server:
        r_link_poses=[]
        # Add lbr4 link poses
        for i in range(8):
            r_pose=self.arm_link_pose_array(u,i)
            r_link_poses.append(r_pose)
        # Add palm pose:
        r_link_poses.append(self.palm_pose_array(u[:7],True))

        
        # Add finger link poses
        for i in range(4):
            for j in range(4):
                if (i*4+j in self.c_list):
                    r_pose=self.hand_link_pose_array(u,j,i)
                    r_link_poses.append(r_pose)

        # Adding thumb_tip:

        return r_link_poses

    def arm_link_pose_array(self,u,link_num):
        T=self.kdlModel.FK_link(u[0:link_num],link_num)
        pose=np.zeros(7)
        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])

        # Normalize quaternion:
        quat=R.GetQuaternion()
       
        pose[3:7]=quat
        pose[0:3]=T[0:3,3].ravel()#+self.link_offset[link_num]
        return pose
   
    def hand_link_pose_array(self,u,link_num,f_index):
        u_=list(np.ravel(u[0:7]))+list(np.ravel(u[7+f_index*4:7+f_index*4+link_num+1]))

        T=self.kdlModel.FK_arm_flink(u_,link_num,f_index)
        pose=np.zeros(7)
        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])

        # Normalize quaternion:
        quat=R.GetQuaternion()

        pose[3:7]=quat
        pose[0:3]=T[0:3,3].ravel()#+self.link_offset[link_num]
        return pose

    def link_jacobian(self,u,link_num,f_index):
        #if f_index==-1, we consider the request as only on the manipulator
        if(f_index==-1):
            J=self.kdlModel.Jacobian_link(u[0:link_num-1],link_num)
            return J
        else:
            # link_num=0-3,
            u_=list(np.ravel(u[0:7]))+list(np.ravel(u[7+f_index*4:7+f_index*4+link_num+1]))
            J=self.kdlModel.Jacobian_arm_flink(u_,link_num,f_index)
            return J


        
