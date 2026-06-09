from utils.src.types import Vector3D, Quaternion, Pose, Rotation 
import math 

def target_relative_drone_setpoint( 
    drone_pose: Pose, 
    target_vector3d: Vector3D, 
    target_orientation: Quaternion|float, 
    distance: float 
) -> Pose: 
    """
    Calculates the target setpoint pose relative to the drone's current pose.
    
    The setpoint is located at the specified distance along the normal vector 
    of the target wall (computed from the target's orientation). The drone 
    is oriented to face the wall (rotating 180 degrees relative to the wall normal).
    
    Parameters:
    -----------
    drone_pose : Pose
        The current global pose of the drone.
    target_vector3d : Vector3D
        The global 3D position of the target.
    target_orientation : Quaternion | float
        The orientation of the target (either as a Quaternion or a float yaw angle in radians).
    distance : float
        The distance from the target to place the setpoint. (Should be in the same metric as the pose)
        
    Returns:
    --------
    Pose
        The setpoint pose in the drone's relative coordinate frame.
    """
    #We construct the setpoint in global coordinates from the target 
    t_v = target_vector3d
    t_o = Quaternion( 
        w=math.cos(target_orientation / 2), 
        x=0.0, 
        y=0.0, 
        z=math.sin( target_orientation / 2)) if isinstance(target_orientation, float) else target_orientation 
    
    forward_axis = Vector3D(1, 0, 0)

    #Calculate the global pose of the setpoint  
    wall_normal_axis = (t_o * forward_axis.cast_to_quaternion() * t_o.c()).cast_to_vector3d().normalized()
    g_v = t_v + (distance * wall_normal_axis)


    #We now need to flip t_o by 180 degrees
    r_axis = forward_axis.cross(-wall_normal_axis)

    if r_axis.norm() > 1e-5: 
        r_axis = r_axis.normalized()
        angle = math.acos((-wall_normal_axis).dot(forward_axis))
        g_o = Rotation(r_axis, angle).q
    else: 
        if (-wall_normal_axis).dot(forward_axis) < 0: 
            g_o = Rotation(Vector3D(0, 1, 0), math.pi).q
        else: 
            g_o = Quaternion(1, 0, 0, 0)

    #Convert to relative
    relative_pose =  drone_pose.convert_to_relative(Pose(g_v, g_o))

    return relative_pose 


    



    








    
