import object as object
import pybullet as p
import math

def test_object_init(bullet_connect):
    object1=object.Object(name="sphere",position=[0,0,0],orientation=[0,0,0],radius=1)
    object2=object.Object(name="barrel",position=[0,4,0],orientation=[0,0,2],radius=0.5,height=1)
    object3=object.Object(name="hoop",position=[1,2,3],orientation=[math.pi/2,0,0],radius=1)
    object4=object.Object(name="r2d2.urdf",position=[4,5,6],orientation=[0,math.pi/2,math.pi/2],scale=1)
    
    object1.initialize()
    object2.initialize()
    object3.initialize()
    object4.initialize()

    assert object1.id is not None
    assert object2.id is not None
    assert object3.id is not None
    assert object4.id is not None
    
    pos,orn=p.getBasePositionAndOrientation(object1.id)
    assert math.isclose(pos[0],0,rel_tol=1e-3) and math.isclose(pos[1],0,rel_tol=1e-3) and math.isclose(pos[2],0,rel_tol=1e-3)
    assert math.isclose(orn[0],0,rel_tol=1e-3) and math.isclose(orn[1],0,rel_tol=1e-3) and math.isclose(orn[2],0,rel_tol=1e-3) and math.isclose(orn[3],1,rel_tol=1e-3)
    pos,orn=p.getBasePositionAndOrientation(object2.id)
    assert math.isclose(pos[0],0,rel_tol=1e-3) and math.isclose(pos[1],4,rel_tol=1e-3) and math.isclose(pos[2],0,rel_tol=1e-3)
    euler=p.getEulerFromQuaternion(orn)
    assert math.isclose(euler[0],0,rel_tol=1e-3) and math.isclose(euler[1],0,rel_tol=1e-3) and math.isclose(euler[2],2,rel_tol=1e-3)
    pos,orn=p.getBasePositionAndOrientation(object3.id)
    assert math.isclose(pos[0],1,rel_tol=1e-3) and math.isclose(pos[1],2,rel_tol=1e-3) and math.isclose(pos[2],3,rel_tol=1e-3)
    euler=p.getEulerFromQuaternion(orn)
    assert math.isclose(euler[0],math.pi/2,rel_tol=1e-3) and math.isclose(euler[1],0,rel_tol=1e-3) and math.isclose(euler[2],0,rel_tol=1e-3)
    pos,orn=p.getBasePositionAndOrientation(object4.id)
    assert math.isclose(pos[0],4,rel_tol=1e-3) and math.isclose(pos[1],5,rel_tol=1e-3) and math.isclose(pos[2],6,rel_tol=1e-3)
    euler= [i for i in p.getEulerFromQuaternion(orn)] 
    for i in range (len(euler)): #sometimes euler conversion gives values out of range of -pi to pi
        if euler[i]>math.pi:
            euler[i]=euler[i]-2*math.pi
        if euler[i]<-math.pi:
            euler[i]=euler[i]+2*math.pi
    assert math.isclose(euler[0],0,rel_tol=1e-3) and math.isclose(euler[1],math.pi/2,rel_tol=1e-3) and math.isclose(euler[2],math.pi/2,rel_tol=1e-3)