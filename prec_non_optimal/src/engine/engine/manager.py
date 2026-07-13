import py_trees
import py_trees_ros
import rclpy
from engine.behaviours.takeoff import Takeoff
from engine.behaviours.fly_around import FlyAround
from engine.behaviours.return_to_launch import ReturnToLaunch
from engine.behaviours.landing import Landing

def create_sequence():
    sequence=py_trees.composite.Sequence("Sequence")
    sequence.add_children([
        Takeoff(),
        FlyAround(),
        ReturnToLaunch(),
        Landing()
    ])
    return sequence

def initialize_blackboard():
    blackboard=py_trees.blackboard.Client(name="engine_blackboard")
    blackboard.register_key(key="altitude", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="longitude", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="latitude", access=py_trees.common.Access.WRITE)
    blackboard.altitude=0.0
    blackboard.longitude=0.0
    blackboard.latitude=0.0
    return blackboard

def main():
    rclpy.init()
    blackboard=initialize_blackboard()
    sequence=create_sequence()
    tree=py_trees_ros.trees.BehaviourTree(root=sequence)
    try:
        tree.setup(name="engine_tree",timeout=15.0)
    except Exception as e:
        tree.node.get_logger().error(f"Error setting up tree: {e}")
        rclpy.try_shutdown()
        return
    tree.tick_tock(period_ms=500.0)
    try:
        if tree is not None:
            rclpy.spin(tree.node)
    except Exception as e:
        tree.node.get_logger().error(f"Error ticking tree: {e}")
        rclpy.try_shutdown()
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
