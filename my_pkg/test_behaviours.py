import unittest
import rclpy
from my_pkg.robot_safety_behaviour import Rotate, StopMotion, BatteryStatus2bb, LaserScan2bb
import py_trees

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

if __name__ == '__main__':
    unittest.main()
