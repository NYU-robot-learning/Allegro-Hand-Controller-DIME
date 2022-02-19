'''
This model builds kdl trees for:
1. Arm from base link to palm of the hand
2. Arm from base link to fingertips
3. Hand from base link to fingertips.
'''

import rospy
import roslib

# importing urdf, kdl related libs:
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics


# Declaring names for our robot in the RoboLLama lab
_LBR4_EE='palm_link'
_LBR4_BASE='lbr4_0_link' # The base link and link-0 of the kuka are the same
_LBR4_LINK_NAMES=['lbr4_0_link','lbr4_1_link','lbr4_2_link','lbr4_3_link','lbr4_4_link','lbr4_5_link','lbr4_6_link','lbr4_7_link']


_ALLEGRO_FTIPS = ['index_tip', 'middle_tip', 'ring_tip', 'thumb_tip']
_ALLEGRO_LINK_NAMES=['index_link_0','index_link_1','index_link_2','index_tip','middle_link_0','middle_link_1','middle_link_2', 'middle_tip','ring_link_0','ring_link_1','ring_link_2', 'ring_tip','thumb_link_0','thumb_link_1','thumb_link_2','thumb_link_3', 'thumb_tip']

# Most of the allegro links do not collide with other links/environment as they are protected. So building a second list which only has links that will be used for collision checking:
_ALLEGRO_COLLISION_LINK_NAMES=['index_link_1','index_link_2','index_tip','middle_link_1','middle_link_2', 'middle_tip','ring_link_1','ring_link_2', 'ring_tip','thumb_link_2','thumb_link_3','thumb_tip']


class HandArmModel:
    def __init__(self,urdf_param_name,base_link=_LBR4_BASE,arm_ee=_LBR4_EE,fingertips=_ALLEGRO_FTIPS):

        # Load urdf
        self.robot=Robot.from_parameter_server(urdf_param_name)

        # Build kdl tree:
        self.tree=kdl_tree_from_urdf_model(self.robot)

        # Build lbr4 to palm link chain:
        self.base_link=base_link

        self.arm_chain=KDLKinematics(self.robot,self.base_link,arm_ee)

        # Build arm to fingertip chains:
        self.arm_finger_chains=[]
        for f_ee_name in fingertips:
            fc=KDLKinematics(self.robot,self.base_link,f_ee_name)
            self.arm_finger_chains.append(fc)

        # Build joint chains to get links' pose:
        # We will build 4 chains(one for each finger) and each finger will have arm links(maybe redundant?).
        self.link_finger_chains=[]
        link_names=_ALLEGRO_LINK_NAMES
        print len(link_names)
        for i in range(len(fingertips)):
            fc=[]
            for j in range(4):
                a=KDLKinematics(self.robot,self.base_link,link_names[i*4+j])
                fc.append(a)
            self.link_finger_chains.append(fc)

        self.arm_link_chains=[]
        for l_name in _LBR4_LINK_NAMES:
            jc=KDLKinematics(self.robot,self.base_link,l_name)
            self.arm_link_chains.append(jc)

    def FK_arm(self,joint_angles):
        fi_x=self.arm_chain.forward(joint_angles)
        return fi_x

    def Jacobian_arm(self,joint_angles):
        ji_x = self.arm_chain.jacobian(joint_angles)
        return ji_x

    def FK_arm_ftip(self,joint_angles,tip_index):
        fi_x=self.arm_finger_chains[tip_index].forward(joint_angles)
        return fi_x

    def Jacobian_arm_ftip(self,joint_angles,tip_index):
        ji_x = self.arm_finger_chains[tip_index].jacobian(joint_angles)
        return ji_x

    def FK_arm_flink(self,joint_angles,j_index,f_index):
        # Joint angles should only contain arm joints + finger joints.
        fi_x=self.link_finger_chains[f_index][j_index].forward(joint_angles)
        return fi_x

    def Jacobian_arm_flink(self,joint_angles,j_index,f_index):
        # Joint angles should only contain arm joints + finger joints.
        ji_x=self.link_finger_chains[f_index][j_index].jacobian(joint_angles)
        return ji_x

    def FK_link(self,joint_angles,j_index):
        '''
        Method to return task coordinates between base link and any joint
        joint_angles must contain only 0:j_index joints
        '''
        fi_x=self.arm_link_chains[j_index].forward(joint_angles)

        return fi_x

    def Jacobian_link(self,joint_angles,j_index):
        ji_x=self.arm_link_chains[j_index].jacobian(joint_angles)
        return ji_x
