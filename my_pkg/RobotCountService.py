import rclpy
from rclpy.node import Node
from srv import GetRobotCount  # Replace with your actual package name
from rclpy.qos import QoSProfile

class RobotCountService(Node):
    def __init__(self):
        super().__init__('robot_count_service')
        self.srv = self.create_service(GetRobotCount, 'get_robot_count', self.get_robot_count_callback)
        
        # Assuming that robot nodes have a common prefix like 'robot_' in their names
        self.robot_node_prefix = 'robot_'
        self.get_logger().info('Robot Count Service is ready')

    def get_robot_count_callback(self, request, response):
        node_names = self.get_node_names()
        robot_count = sum(1 for name in node_names if name.startswith(self.robot_node_prefix))
        response.robot_count = robot_count
        self.get_logger().info(f'Number of robots: {robot_count}')
        return response

    def get_node_names(self):
        # This will return a list of active node names in the ROS graph
        node_names = self.get_node_names_and_namespaces()
        return [name[0] for name in node_names]

def main(args=None):
    rclpy.init(args=args)
    robot_count_service = RobotCountService()
    rclpy.spin(robot_count_service)
    robot_count_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
