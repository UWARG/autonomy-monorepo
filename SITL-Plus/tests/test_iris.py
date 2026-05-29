import iris as iris
import pybullet as p
import state

def test_iris_init(bullet_connect):
    iris_obj=iris.Iris()
    assert iris_obj.motor_indices==[1,2,3,4]
    assert iris_obj.motor_dir==[1,1,-1,-1]
    assert iris_obj.motor_speed==5
    assert iris_obj.thrust_scale==0.01
    assert iris_obj.rotor_torque_dirs==[1,1,-1,-1]
    assert iris_obj.torque_coef==0.001
    L=0.2
    assert iris_obj.rotor_positions==[    
            [L, -L, 0],   # motor 1, Front-Right
            [-L, L, 0],   # motor 2, Rear-Left
            [L, L, 0],   # motor 3, Front-Left
            [-L, L, 0],   # motor 4, Rear-Right
        ]

def test_iris_reset(bullet_connect,iris_obj):
    iris_obj.reset()
    pos,orient=p.getBasePositionAndOrientation(state.robot_id)
    assert pos==(0,0,0.2)
    assert orient==(0,0,0,1)