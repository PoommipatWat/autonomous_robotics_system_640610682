import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import coppeliasim_zmqremoteapi_client as zmqRemoteApi
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class RobotOdom(Node):
    def __init__(self):
        super().__init__('ros_odom')
        
        # CoppeliaSim remote API setup
        self.client = zmqRemoteApi.RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.startSimulation()
        self.ground_handle = self.sim.getObject('/Floor')
        self.robot_handle = self.sim.getObject('/robot')
        
        # Create odometry publisher
        self.odom_pub_ = self.create_publisher(Odometry, 'odom', 10)
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for publishing
        self.timer_ = self.create_timer(0.05, self.timer_callback)
        
        # Initialize previous position and time for velocity calculation
        self.prev_position = np.zeros(3)
        self.prev_orientation = np.zeros(3)
        self.prev_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        
        # Get robot position and orientation
        robot_position = np.array(self.sim.getObjectPosition(self.robot_handle, self.ground_handle))
        robot_position = np.round(robot_position, 5)
        robot_orientation = np.array(self.sim.getObjectOrientation(self.robot_handle, self.ground_handle))
        robot_orientation = np.round(robot_orientation, 5)
        
        # Convert Euler angles to quaternion
        quat = self.euler_to_quaternion(robot_orientation[2])
        
        # Calculate velocity (if needed)
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt > 0:
            linear_vel = (robot_position - self.prev_position) / dt
            angular_vel = (robot_orientation - self.prev_orientation) / dt
        else:
            linear_vel = np.zeros(3)
            angular_vel = np.zeros(3)
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = float(robot_position[0])
        odom_msg.pose.pose.position.y = float(robot_position[1])
        odom_msg.pose.pose.position.z = float(robot_position[2])
        
        # Set orientation
        odom_msg.pose.pose.orientation.x = float(quat[0])
        odom_msg.pose.pose.orientation.y = float(quat[1])
        odom_msg.pose.pose.orientation.z = float(quat[2])
        odom_msg.pose.pose.orientation.w = float(quat[3])
        
        # Set velocities
        odom_msg.twist.twist.linear.x = float(linear_vel[0])
        odom_msg.twist.twist.linear.y = float(linear_vel[1])
        odom_msg.twist.twist.linear.z = float(linear_vel[2])
        odom_msg.twist.twist.angular.x = float(angular_vel[0])
        odom_msg.twist.twist.angular.y = float(angular_vel[1])
        odom_msg.twist.twist.angular.z = float(angular_vel[2])
        
        # Publish odometry message
        self.odom_pub_.publish(odom_msg)
        
        # Broadcast transform
        self.broadcast_transform(robot_position, quat, current_time)
        
        # Update previous values
        self.prev_position = robot_position
        self.prev_orientation = robot_orientation
        self.prev_time = current_time

    def broadcast_transform(self, position, quaternion, timestamp):
        t = TransformStamped()
        
        # Set header
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Set transform
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, yaw, pitch=0, roll=0):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return x, y, z, w

def main(args=None):
    rclpy.init(args=args)
    odom = RobotOdom()
    try:
        rclpy.spin(odom)
    except KeyboardInterrupt:
        pass
    finally:
        odom.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()