import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GripperTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('gripper_trajectory_publisher')
        
        # Create a publisher for the JointTrajectory topic
        self.publisher_ = self.create_publisher(JointTrajectory, '/grip_controller/joint_trajectory', 10)
        
        # Publish after 1 second to allow for connection establishment
        self.timer = self.create_timer(1.0, self.publish_gripper_trajectory)

    def publish_gripper_trajectory(self):
        # Create the JointTrajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['gripper_controller']
        
        # Create the JointTrajectoryPoint message
        point = JointTrajectoryPoint()
        point.positions = [-0.70]
        
        # Set the time_from_start (5 seconds)
        point.time_from_start = Duration(sec=5, nanosec=0)
        
        # Add the point to the trajectory message
        traj_msg.points.append(point)
        
        # Publish the message
        self.publisher_.publish(traj_msg)
        self.get_logger().info('Published gripper joint trajectory')

        # Shutdown after publishing once
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node and start spinning
    node = GripperTrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()