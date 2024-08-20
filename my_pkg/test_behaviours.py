import unittest
import rclpy
import math
from my_pkg.robot_safety_behaviour import Rotate, StopMotion, BatteryStatus2bb, LaserScan2bb
import py_trees
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from time import sleep
from srv import GetRobotCount  # Assume a service exists that returns the robot count

class TestRotateBehavior(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)

    def setUp(self):
        self.rotate_behavior = Rotate(name="Rotate", topic_name="/cmd_vel", ang_vel=1.0)
        self.node = rclpy.create_node('test_rotate_node')
        self.rotate_behavior.setup(node=self.node)

    def test_initial_setup(self):
        self.assertIsNotNone(self.rotate_behavior.twist_publisher)
    
    def test_rotate_update(self):
        status = self.rotate_behavior.update()
        self.assertEqual(status, py_trees.common.Status.RUNNING)

    def test_terminate(self):
        self.rotate_behavior.terminate(py_trees.common.Status.SUCCESS)
        self.assertEqual(self.rotate_behavior.sent_goal, False)

    def tearDown(self):
        self.node.destroy_node()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

class TestStopMotionBehavior(unittest.TestCase):

    def setUp(self):
        self.stop_motion_behavior = StopMotion(name="StopMotion")
        self.node = rclpy.create_node('test_stop_motion_node')
        self.stop_motion_behavior.setup(node=self.node)

    def test_initial_setup(self):
        self.assertIsNotNone(self.stop_motion_behavior.twist_publisher)
    
    def test_stop_update(self):
        status = self.stop_motion_behavior.update()
        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def tearDown(self):
        self.node.destroy_node()

class TestBatteryStatus2bb(unittest.TestCase):

    def setUp(self):
        self.battery_behavior = BatteryStatus2bb(battery_voltage_topic_name="/battery_voltage", name="BatteryStatus2bb", threshold=20.0)
        self.node = rclpy.create_node('test_battery_status_node')
        self.battery_behavior.setup(node=self.node)

    def test_initial_setup(self):
        self.assertTrue(self.battery_behavior.blackboard.exists('battery_low_warning'))

    def test_update(self):
        self.battery_behavior.blackboard.battery = 15.0
        status = self.battery_behavior.update()
        self.assertEqual(self.battery_behavior.blackboard.battery_low_warning, True)
        self.assertEqual(status, py_trees.common.Status.SUCCESS)

    def tearDown(self):
        self.node.destroy_node()

class TestMultiRobotSystem(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    def setUp(self):
        self.node = rclpy.create_node('test_multi_robot_system')

    def test_robot_count(self):
        client = self.node.create_client(GetRobotCount, '/get_robot_count')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        request = GetRobotCount.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        expected_robot_count = 5  # Expected number of robots
        self.assertEqual(response.robot_count, expected_robot_count)

    def tearDown(self):
        self.node.destroy_node()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

class TestDistanceConstraint(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    def setUp(self):
        self.node = rclpy.create_node('test_distance_constraint')
        self.leader_pose_sub = self.node.create_subscription(PoseStamped, '/leader/pose', self.leader_pose_callback, 10)
        self.follower_pose_sub = self.node.create_subscription(PoseStamped, '/follower1/pose', self.follower_pose_callback, 10)
        self.leader_pose = None
        self.follower_pose = None

    def leader_pose_callback(self, msg):
        self.leader_pose = msg.pose

    def follower_pose_callback(self, msg):
        self.follower_pose = msg.pose

    def test_distance_maintained(self):
        predefined_distance = 1.0
        tolerance = 0.1

        for _ in range(100):  # Run multiple iterations to check consistency
            if self.leader_pose and self.follower_pose:
                distance = self.calculate_distance(self.leader_pose, self.follower_pose)
                self.assertAlmostEqual(distance, predefined_distance, delta=tolerance)
            sleep(0.1)

    def calculate_distance(self, position1, position2):
        return math.sqrt(
            (position1.position.x - position2.position.x)**2 +
            (position1.position.y - position2.position.y)**2
        )

    def tearDown(self):
        self.node.destroy_node()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
