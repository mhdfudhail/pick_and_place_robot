import rclpy
from rclpy.node import Node
from robot_interface.srv import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from functools import partial
import time


class SumClientNode(Node):
    def __init__(self):
        super().__init__('sum_client_node')
        self.get_logger().info('Sum Client Python node has been created')

        # declare parameters for AddTwoInts
        a_ = 0.0
        b_ = -0.55
        
        self.call_gripper_server_close(value=b_)
        time.sleep(5.0)
        self.call_gripper_server_open(value=a_)
    
    def call_gripper_server_open(self, value):
        client = self.create_client(GripperCommand, 'gripper_cmd')
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for the Server...')

        # create request
        request = GripperCommand.Request()
        request.opening = value

        #send request asynchronously
        future = client.call_async(request)
        future.add_done_callback(partial(self.gripper_service_callback))

    def call_gripper_server_close(self, value):
        client = self.create_client(GripperCommand, 'gripper_cmd')
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for the Server...')

        # create request
        request = GripperCommand.Request()
        request.opening = value

        #send request asynchronously
        future = client.call_async(request)
        future.add_done_callback(partial(self.gripper_service_callback))

    def gripper_service_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Gripper value:{response.status}')
        except Exception as e:
            self.get_logger().info(f'Service call failed {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SumClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()
