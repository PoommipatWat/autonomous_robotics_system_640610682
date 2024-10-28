import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import coppeliasim_zmqremoteapi_client as zmqRemoteApi
import numpy as np

class RosTeleop(Node):
    def __init__(self):
        super().__init__('ros_teleop')
        
        # CoppeliaSim remote API setup
        self.client = zmqRemoteApi.RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.startSimulation()
        
        # Get handles
        self.ground_handle = self.sim.getObject('/Floor')
        self.robot_handle = self.sim.getObject('/robot')
        
        # Get motor handles
        self.motor_fl_handle = self.sim.getObject('/robot/fl_wheels')  # Front Left
        self.motor_bl_handle = self.sim.getObject('/robot/bl_wheels')  # Back Left
        self.motor_br_handle = self.sim.getObject('/robot/br_wheels')  # Back Right
        self.motor_fr_handle = self.sim.getObject('/robot/fr_wheels')  # Front Right
        
        # Create subscriber
        self.twist_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        
        self.get_logger().info('RosTeleop node has been initialized')

    def kinematics(self, forw_back_vel, left_right_vel, rot_vel):
        """
        Convert velocity commands to wheel velocities
        Args:
            forw_back_vel (float): Forward/Backward velocity
            left_right_vel (float): Left/Right velocity
            rot_vel (float): Rotational velocity
        Returns:
            tuple: Velocities for (front_left, back_left, back_right, front_right) wheels
        """
        # Following the Lua script logic:
        # front_left  = forwBackVel + leftRightVel - rotVel
        # back_left   = forwBackVel - leftRightVel - rotVel
        # back_right  = forwBackVel + leftRightVel + rotVel
        # front_right = forwBackVel - leftRightVel + rotVel
        
        # Scale factor for velocity (same as in Lua script)
        scale = 20.0
        
        front_left = scale * (forw_back_vel + left_right_vel - rot_vel)
        back_left = scale * (forw_back_vel - left_right_vel - rot_vel)
        back_right = scale * (forw_back_vel + left_right_vel + rot_vel)
        front_right = scale * (forw_back_vel - left_right_vel + rot_vel)
        
        return front_left, back_left, back_right, front_right

    def twist_callback(self, msg):
        """
        Callback function for cmd_vel topic
        Args:
            msg (Twist): Velocity command message
        """
        # Extract velocities from message
        forward_vel = msg.linear.x
        side_vel = msg.linear.y
        rot_vel = msg.angular.z
        
        # Get wheel velocities
        v_fl, v_bl, v_br, v_fr = self.kinematics(forward_vel, side_vel, rot_vel)
        
        # Log velocities for debugging
        self.get_logger().debug(
            f'Wheel velocities: FL={v_fl:.2f}, BL={v_bl:.2f}, '
            f'BR={v_br:.2f}, FR={v_fr:.2f}'
        )
        
        # Set joint velocities in CoppeliaSim
        # Note the negative signs match the original script's wheel directions
        self.sim.setJointTargetVelocity(self.motor_fl_handle, -v_fl)
        self.sim.setJointTargetVelocity(self.motor_bl_handle, -v_bl)
        self.sim.setJointTargetVelocity(self.motor_br_handle, v_br)
        self.sim.setJointTargetVelocity(self.motor_fr_handle, v_fr)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop = RosTeleop()
        rclpy.spin(teleop)
    except Exception as e:
        print(f'An error occurred: {e}')
    finally:
        # Make sure to cleanup properly
        if 'teleop' in locals():
            teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()