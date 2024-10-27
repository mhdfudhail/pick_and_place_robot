import rclpy
from rclpy.node import Node
from robot_interface.srv import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ControllServer(Node):
    def __init__(self):
        super().__init__("sum_server")
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/grip_controller/joint_trajectory', 10)
        self.server_ = self.create_service(GripperCommand, "gripper_cmd", self.gripper_callback)
        self.get_logger().info("Service server Python node has been created")
    
    def gripper_callback(self, request, response): 
        self.get_logger().info(f'Recieved Input: {request.opening}')
        status = self.publish_gripper_trajectory(request.opening)
        response.status = True
        self.get_logger().info(f'Processed the request: {status} ')
        return response 
    
    def publish_gripper_trajectory(self, pose):
        # Create the JointTrajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['gripper_controller']
        
        # Create the JointTrajectoryPoint message
        point = JointTrajectoryPoint()
        point.positions = [pose]
        
        # Set the time_from_start (5 seconds)
        point.time_from_start = Duration(sec=5, nanosec=0)
        
        # Add the point to the trajectory message
        traj_msg.points.append(point)
        
        # Publish the message
        self.gripper_publisher.publish(traj_msg)
        self.get_logger().info('Published gripper joint trajectory')
        return True


def main(args = None):
    rclpy.init(args=args)
    node = ControllServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()