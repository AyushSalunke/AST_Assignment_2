import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis 
    """
    def __init__(self, name="rotate platform", topic_name="/cmd_vel", ang_vel=1.0):
        super(Rotate, self).__init__(name)
        self.topic_name = topic_name
        self.ang_vel = ang_vel
        # self.node = None
        # self.twist_publisher = None
        self.sent_goal = False

        # TODO: initialise any necessary class variables
        # YOUR CODE HERE
        #raise NotImplementedError()

    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behaviour")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 

        # TODO: setup any necessary publishers or subscribers
        self.twist_publisher = self.node.create_publisher(Twist, self.topic_name, ptr.utilities.qos_profile_latched())
        # YOUR CODE HERE
        #raise NotImplementedError()

        return True

    def update(self):
        """Rotates the robot at the maximum allowed angular velocity.
        Note: The actual behaviour is implemented here.

        """
        self.logger.info("[ROTATE] update: updating rotate behaviour")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # TODO: implement the primary function of the behaviour and decide which status to return 
        # based on the structure of your behaviour tree

        # Hint: to return a status, for example, SUCCESS, pt.common.Status.SUCCESS can be used
        twist_msg = Twist()
        twist_msg.angular.z = self.ang_vel
        self.twist_publisher.publish(twist_msg)

        return pt.common.Status.RUNNING

        # YOUR CODE HERE
        #raise NotImplementedError()


    def terminate(self, new_status):
        """Trigerred once the execution of the behaviour finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")

        # TODO: implement the termination of the behaviour, i.e. what should happen when the behaviour 
        # finishes its execution
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0

        self.twist_publisher.publish(twist_msg)
        self.sent_goal = False

        # YOUR CODE HERE
        #raise NotImplementedError()

        return super().terminate(new_status)

class StopMotion(pt.behaviour.Behaviour):
    """Stops the robot when it is controlled using a joystick or with a cmd_vel command
    """
    
    # TODO: Implement a behaviour to stop the robot's motion
    def __init__(self, name="StopMotion", topic_name="/cmd_vel"):
        super(StopMotion, self).__init__(name)
        self.topic_name = topic_name
        # self.node = None
        # self.twist_publisher = None
        
    def setup(self, **kwargs):
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.twist_publisher = self.node.create_publisher(
            msg_type=Twist,
            topic= self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        
        return True

        # self.node = kwargs['node']
        # self.twist_publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        # return True
    
    def update(self):
        self.logger.info("[STOP] update: updating stop behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        self.twist_publisher.publish(twist_msg)

        return pt.common.Status.SUCCESS
    # YOUR CODE HERE
    #raise NotImplementedError()

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.twist_publisher.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)


class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Checks the battery status
    """
    def __init__(self, battery_voltage_topic_name: str="/battery_voltage", name: str='Battery2BB', threshold: float=30.0):
        super().__init__(name=name,
                         topic_name=battery_voltage_topic_name,
                         topic_type=Float32,
                         blackboard_variables={'battery': 'data'},
                         initialise_variables={'battery': 100.0},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=ptr.utilities.qos_profile_unlatched())
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.WRITE)
        
        self.blackboard.battery_low_warning = False
        self.threshold = threshold

        # YOUR CODE HERE
        #raise NotImplementedError()


    def update(self):
        """Calls the parent to write the raw data to the blackboard and then check against the
        threshold to determine if a low warning flag should also be updated.
        """
        self.logger.info('[BATTERY] update: running battery_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(BatteryStatus2bb, self).update()

        """check battery voltage level stored in self.blackboard.battery. By comparing with 
        threshold value, update the value of self.blackboad.battery_low_warning
        """

        # TODO: based on the battery voltage level, update the value of self.blackboard.battery_low_warning
        # and return the status of the behaviour based on your logic of the behaviour tree

        if self.blackboard.exists('battery'):
            if self.blackboard.battery < self.threshold:
                self.blackboard.battery_low_warning = True
                self.feedback_message = "Battery level is low"
            else:
                self.blackboard.battery_low_warning = False
                self.feedback_message = "Battery level is okay"
        return pt.common.Status.SUCCESS

        # YOUR CODE HERE
        #raise NotImplementedError()


class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Checks the laser scan measurements to avoid possible collisions.
    """
    def __init__(self, topic_name: str="/scan", name: str='Scan2BB', safe_range: float=0.25):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan':'ranges'},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                depth=10))
        
        # TODO: initialise class variables and blackboard variables
        # YOUR CODE HERE
        self.blackboard.register_key(
            key='possible_collision',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='point_at_min_distance',
            access=pt.common.Access.WRITE
        )
        self.blackboard.possible_collision = False   
        self.safe_range = safe_range
        self.blackboard.point_at_min_distance = 0.0

       # raise NotImplementedError()

    def update(self):
        self.logger.info("[LASER SCAN] update: running laser_scan_2bb update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(LaserScan2bb, self).update()
        # TODO: impletment the update function to check the laser scan data and update the blackboard variable
        # YOUR CODE HERE

        if self.blackboard.exists('laser_scan'):
            if min(self.blackboard.laser_scan) < self.safe_range:
                self.blackboard.possible_collision = True
                self.blackboard.point_at_min_distance = min(self.blackboard.laser_scan)
            else:
                self.blackboard.possible_collision = False
        else:
            return pt.common.Status.SUCCESS
        
        return pt.common.Status.SUCCESS


        #raise NotImplementedError()