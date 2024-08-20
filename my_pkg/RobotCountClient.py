import rclpy
from rclpy.node import Node
from srv import GetRobotCount

class RobotCountClient(Node):
    def __init__(self):
        super().__init__('robot_count_client')
        self.client = self.create_client(GetRobotCount, 'get_robot_count')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        self.get_logger().info('Service is available')

    def send_request(self):
        request = GetRobotCount.Request()
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    robot_count_client = RobotCountClient()
    response = robot_count_client.send_request()
    robot_count_client.get_logger().info(f'Number of robots: {response.robot_count}')
    robot_count_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
