# Use this file to test kdl functions

import numpy as np
from handArmModel import handArmModel

from rospkg import RosPack
rp=RosPack()
rp.list()
path_urdf=rp.get_path('urlg_robots_description')+'/robots/'

def test_lbr4_allegro_model():
    # import class:
    file_=path_urdf+'lbr4_allegro_right.urdf'

    print "Opening "+file_
    lbr4_allegro=handArmModel(file_)
    j=np.zeros(7)
    print lbr4_allegro.FK_arm(j)

    j=np.zeros(7+4)
    print lbr4_allegro.FK_arm_ftip(j,3)
                            
test_lbr4_allegro_model()
