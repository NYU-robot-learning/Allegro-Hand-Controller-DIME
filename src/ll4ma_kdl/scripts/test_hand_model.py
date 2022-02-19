import numpy as np
from handModel import HandModel
from scipy.spatial.transform import Rotation as R

def test_hand_model_FK():
    model = HandModel("allegro/robot_description")
    q = np.array([0.1, 0.1, 0.1, 0.1]).T
    # 2 denotes RING finger
    fk = model.FK(q, 2)  # 4x4 matrix
    assert fk.shape == (4,4)
    position = np.squeeze(fk[0:3,3]).T
    assert abs(position[0,0] - 0.03917029) < 1e-4
    assert abs(position[1,0] - (-0.05071672)) < 1e-4
    assert abs(position[2,0] - 0.14898276) < 1e-4
    print("FK is correct")

    # fk for biotac origin must be different
    fk_origin = model.FK(q, 2,"ring_biotac_origin")
    position_origin = np.squeeze(fk_origin[0:3,3]).T
    assert abs(position_origin[0,0] - position[0,0]) > 1e-4
    assert abs(position_origin[1,0] - position[1,0]) > 1e-4
    assert abs(position_origin[2,0] - position[2,0]) > 1e-4
    print("chains to origins are constructed as well")

    # test jacobians
    q_1 = q
    q_2 = np.array([1e-2 + q_1[i] for i in range(4)]).T
    delta_q = q_2 - q_1
    fk_2 = model.FK(q_2, 2) 
    fk_1 = model.FK(q_1,2)  # 4x4 matrix
    position = np.squeeze(fk_2[0:3,3]).T - np.squeeze(fk_1[0:3,3]).T
    rotation = R.from_dcm(fk_2[:3,:3]).as_euler("xyz").T - R.from_dcm(fk_1[:3,:3]).as_euler("xyz").T
    rotation = np.expand_dims(rotation,1)
    delta_x = np.vstack((position,rotation))

    j = model.Jacobian(q_2, 2)
    print("j dot delta_q: ", np.dot(j, delta_q))
    print("delta_x: ", delta_x)

def main():
    test_hand_model_FK()
    
if __name__ == "__main__": 
    main()
