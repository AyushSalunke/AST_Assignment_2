### Implement a behaviour tree using your previously implemented behaviours here

import py_trees as pt
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys
from my_pkg.robot_safety_behaviour import *

def create_root() -> pt.behaviour.Behaviour:
    """Structures a behaviour tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """

    # we define the root node
    root = pt.composites.Parallel(name="root", policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    

    ### we create a sequence node called "Topics2BB" and a selector node called "Priorities"
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)
    
    """
    TODO:  The first and second level of the tree structure is defined above, but please
    define the rest of the tree structure.

    Class definitions for your behaviours are provided in behaviours.py; you also need to fill out
    the behaviour implementations!

    HINT: Some behaviours from pt.behaviours may be useful to use as well.
    """
    battery_status_bb = BatteryStatus2bb(battery_voltage_topic_name="/battery_voltage", name="BatteryStatus2bb", threshold=20.0)
    laser_scan_bb = LaserScan2bb(topic_name="/scan", name="LaserScan2bb", safe_range=0.5)
    
    battery_emergency = pt.composites.Sequence("Battery low", memory=False)
    collison_emergency = pt.composites.Sequence("Collision", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)
    
    low_battery = pt.behaviours.CheckBlackboardVariableValue(name="if Battery is Low", 
                                                             check=pt.common.ComparisonExpression(variable="battery_low_warning",
                                                                                                  value=True,
                                                                                                  operator=operator.eq))


    collision = pt.behaviours.CheckBlackboardVariableValue(name="if Colliding", 
                                                             check=pt.common.ComparisonExpression(variable="possible_collision",
                                                                                                  value=True,
                                                                                                  operator=operator.eq))
    # YOUR CODE HERE
    #raise NotImplementedError()

    rotate_behavior = Rotate(name="Rotate", topic_name="/cmd_vel", ang_vel=1.0)
    stop_motion_behavior = StopMotion(name="StopMotion") #, topic_name="/cmd_vel")

    ### we create an "Idle" node, which is a running node to keep the robot idle
    idle = pt.behaviours.Running(name="Idle")

    # TODO: construct the behaviour tree structure using the nodes and behaviours defined above
    # HINT: for reference, the sample tree structure in the README.md file might be useful

    root.add_children([topics2BB, priorities])
    
    # topics2BB.add_children([BatteryStatus2bb, LaserScan2bb])
    topics2BB.add_children([battery_status_bb, laser_scan_bb])

    priorities.add_children([collison_emergency, battery_emergency, idle]) 

    battery_emergency.add_children([low_battery, rotate_behavior])
    collison_emergency.add_children([collision, stop_motion_behavior])


    # YOUR CODE HERE
    #raise NotImplementedError()

    return root

def main():
    """Initialises and executes the behaviour tree
    """
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=100)    

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()