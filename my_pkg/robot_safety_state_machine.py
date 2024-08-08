import rclpy
import smach
import time

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MonitorBatteryAndCollision(smach.State):
    """State to monitor the battery level and possible collisions"""
    def __init__(self, node):
        super(MonitorBatteryAndCollision, self).__init__(outcomes=['battery_ok', 'collision_detected'])
        
        self.node = node
        self.logger = self.node.get_logger()
        self.battery_subscriber = self.node.create_subscription(String, '/battery_level', self.battery_callback, 10)
        self.collision_subscriber = self.node.create_subscription(LaserScan, '/scan', self.collision_callback, 10)
        self.twist_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.battery_threshold = 20  
        self.collision_threshold = 0.5  
        self.battery_ok = False
        self.collision_detected = False

        self.logger.info("MonitorBatteryAndCollision state initialized")

    def battery_callback(self, msg):
        if int(msg.data) > self.battery_threshold:
            self.battery_ok = True
        else:
            self.logger.warning('Battery low')
            self.battery_ok = False
            
    def collision_callback(self, msg):
        if min(msg.ranges) < self.collision_threshold:
            self.logger.warning('Collision possible')
            self.collision_detected = True
        else:
            self.collision_detected = False

    def execute(self, userdata):
        self.logger.info("Executing MonitorBatteryAndCollision state")
        while not self.battery_ok:
            twist_msg = Twist()
            twist_msg.angular.z = 0.5  
            self.twist_publisher.publish(twist_msg)
            time.sleep(1)  
        return 'battery_ok'

class RotateBase(smach.State):
    """State to rotate the robot's base"""
    def __init__(self, node):
        super(RotateBase, self).__init__(outcomes=['rotated'])
        
        self.node = node
        self.logger = self.node.get_logger()
        self.twist_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.logger.info("RotateBase state initialized")

    def execute(self, userdata):
        self.logger.info("Executing RotateBase state")
        twist_msg = Twist()
        twist_msg.angular.z = 1 
        self.twist_publisher.publish(twist_msg)
        time.sleep(0.5)  
        return 'rotated'

class StopMotion(smach.State):
    """State to stop the robot's motion"""
    def __init__(self, node):
        super(StopMotion, self).__init__(outcomes=['stopped'])
        
        self.node = node
        self.logger = self.node.get_logger()
        self.twist_publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.logger.info("StopMotion state initialized")

    def execute(self, userdata):
        self.logger.info("Executing StopMotion state")
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        self.twist_publisher.publish(twist_msg)
        time.sleep(0.5)
        return 'stopped'

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('robot_state_machine')

    sm = smach.StateMachine(outcomes=['completed'])
    with sm:
        smach.StateMachine.add('MONITOR', MonitorBatteryAndCollision(node),
                               transitions={'battery_ok': 'completed', 'collision_detected': 'STOP'})
        smach.StateMachine.add('ROTATE', RotateBase(node),
                               transitions={'rotated': 'MONITOR'})
        smach.StateMachine.add('STOP', StopMotion(node),
                               transitions={'stopped': 'MONITOR'})

    outcome = sm.execute()
    if outcome == 'completed':
        node.get_logger().info('State machine executed successfully')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()