import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Point, Vector3, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import numpy as np
from .ai_samples.rrt_star import RRTStar
import random
import coppeliasim_zmqremoteapi_client as zmqRemoteApi

class RRTExplorer(Node):
    def __init__(self):
        super().__init__('rrt_explorer')
        
        # Initialize map-related variables
        self.map_data = None
        self.map_origin = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.is_map_ready = False
        
        # CoppeliaSim setup
        self.client = zmqRemoteApi.RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.startSimulation()
        self.ground_handle = self.sim.getObject('/Floor')
        self.robot_handle = self.sim.getObject('/robot')
        
        # Navigation variables
        self.current_path = None
        self.current_path_index = 0
        self.robot_position = None
        self.robot_pose = None  # (x, y, yaw)
        self.robot_radius = 0.001
        self.goal = None
        self.goal_orientation = None
        self.is_rotating = True  # Flag to indicate if robot is rotating to target
        
        # Control parameters
        self.k_angular = 1.0
        self.linear_speed = 2.0  # Fixed linear speed
        self.max_angular_speed = 0.5
        self.goal_tolerance = 0.1
        self.angle_tolerance = np.deg2rad(5.0)  # Tighter angle tolerance
        
        
        # Subscribers
        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/path_marker', 10)
        
        # Timers
        self.create_timer(0.1, self.control_loop)  # Control loop at 10Hz
        self.create_timer(1.0, self.planning_loop)  # Planning loop at 1Hz
        
        self.get_logger().info('RRT Explorer initialized')
        self.cmd_pub.publish(Twist())

    def simplify_path(self, path):
        """Simplify path by merging points with similar angles"""
        if not path or len(path) < 3:  # Need at least 3 points to simplify
            return path

        simplified_path = [path[0]]  # Start with first point
        angle_threshold = np.deg2rad(self.angle_tolerance)  # Maximum angle difference threshold (20 degrees)
        
        i = 0
        while i < len(path) - 2:
            current = path[i]
            next_point = path[i + 1]
            next_next = path[i + 2]
            
            # Convert points to world coordinates
            current_world = self.grid_to_world(current[0], current[1])
            next_world = self.grid_to_world(next_point[0], next_point[1])
            next_next_world = self.grid_to_world(next_next[0], next_next[1])
            
            if not all([current_world, next_world, next_next_world]):
                i += 1
                continue
            
            # Calculate angles between segments
            angle1 = np.arctan2(next_world[1] - current_world[1], 
                              next_world[0] - current_world[0])
            angle2 = np.arctan2(next_next_world[1] - next_world[1], 
                              next_next_world[0] - next_world[0])
            
            # Calculate angle difference
            angle_diff = abs(self.normalize_angle(angle2 - angle1))
            
            if angle_diff < angle_threshold:
                # Skip the middle point if angles are similar
                i += 2
            else:
                # Add the next point and move forward
                simplified_path.append(next_point)
                i += 1
        
        # Always add the last point
        simplified_path.append(path[-1])
        
        self.get_logger().info(f'Path simplified from {len(path)} to {len(simplified_path)} points')
        return simplified_path

    def set_random_spawn_position(self):
        """Set random initial position avoiding obstacles (only x,y, limited to ±6)"""
        if not self.is_map_ready:
            return
            
        max_attempts = 100
        for _ in range(max_attempts):
            # Random position in range [-6, 6]
            x = random.uniform(-6.0, 6.0)
            y = random.uniform(-6.0, 6.0)
            
            # Convert to grid coordinates
            gx, gy = self.world_to_grid(x, y)
            
            # Check if position is valid in grid
            if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                if self.map_data[gy, gx] == 0:  # Check if free space
                    # Get current z position
                    current_pos = self.sim.getObjectPosition(self.robot_handle, self.ground_handle)
                    z = current_pos[2]  # Keep current z value
                    
                    # Set only x,y position in CoppeliaSim
                    self.sim.setObjectPosition(self.robot_handle, self.ground_handle, 
                                            [x, y, z])
                    
                    # Random orientation in yaw
                    random_orientation = random.uniform(-np.pi, np.pi)
                    current_ori = self.sim.getObjectOrientation(self.robot_handle, self.ground_handle)
                    # Keep current x,y rotation, only set z rotation (yaw)
                    self.sim.setObjectOrientation(self.robot_handle, self.ground_handle,
                                               [current_ori[0], current_ori[1], random_orientation])
                    
                    self.get_logger().info(f'Set random spawn at: ({x:.2f}, {y:.2f}), orientation: {np.rad2deg(random_orientation):.2f} degrees')
                    return True
        
        self.get_logger().error('Failed to find valid spawn position')
        return False

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return yaw

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

    def occupancy_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        self.map_data = (self.map_data > 50).astype(int)
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.is_map_ready = True
        
        # Set initial random position after map is ready
        if self.robot_position is None:
            self.set_random_spawn_position()

    def odom_callback(self, msg):
        if not self.is_map_ready:
            return
            
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_position = self.world_to_grid(x, y)
        
        orientation_q = msg.pose.pose.orientation
        yaw = self.quaternion_to_euler(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        self.robot_pose = (x, y, yaw)

    def world_to_grid(self, x, y):
        if not self.is_map_ready:
            return None
        gx = int((x - self.map_origin[0]) / self.map_resolution)
        gy = int((y - self.map_origin[1]) / self.map_resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        if not self.is_map_ready:
            return None
        x = gx * self.map_resolution + self.map_origin[0]
        y = gy * self.map_resolution + self.map_origin[1]
        return (x, y)

    def generate_random_goal(self):
        if not self.is_map_ready:
            return None
            
        max_attempts = 100
        for _ in range(max_attempts):
            # Random position in range [-6, 6]
            x = random.uniform(-6.0, 6.0)
            y = random.uniform(-6.0, 6.0)
            
            # Convert to grid coordinates
            gx, gy = self.world_to_grid(x, y)
            
            # Check if position is valid in grid
            if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                if self.map_data[gy, gx] == 0:
                    random_orientation = random.uniform(-np.pi, np.pi)
                    self.goal_orientation = random_orientation
                    self.get_logger().info(f'Generated random goal at: ({x:.2f}, {y:.2f}) with orientation: {np.rad2deg(random_orientation):.2f} degrees')
                    return (gx, gy)
        
        self.get_logger().error('Failed to generate valid random goal')
        return None

    def plan_path(self, goal_position):
        if not self.is_map_ready or self.robot_position is None:
            return None

        self.get_logger().info(f"Planning path from {self.robot_position} to {goal_position}")
        
        rrt_star = RRTStar(
            self.map_data,
            self.robot_position,
            goal_position,
            max_iterations=5000,
            max_travel_distance=int(0.4 / self.map_resolution)*4.00,
            search_radius=int(0.4 / self.map_resolution)*4.0,
            goal_radius=int(0.2 / self.map_resolution)
        )
        
        path = rrt_star.build()
        if path:
            # Simplify the path before using it
            simplified_path = self.simplify_path(path)
            self.get_logger().info(f'Path found and simplified: {len(path)} -> {len(simplified_path)} waypoints')
            self.publish_path_marker(simplified_path)
            return simplified_path
        
        self.get_logger().warn('No path found')
        return None

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2*np.pi
        while angle < -np.pi:
            angle += 2*np.pi
        return angle

    def control_loop(self):
        """Execute control loop for navigation"""
        if not self.robot_pose or not self.current_path:
            return

        # Get current robot pose
        x, y, yaw = self.robot_pose
        
        # Get current target point
        target = self.current_path[self.current_path_index]
        target_world = self.grid_to_world(target[0], target[1])
        if not target_world:
            return
            
        goal_x, goal_y = target_world
        
        # Calculate distance and angle to goal
        dx = goal_x - x
        dy = goal_y - y
        distance = np.sqrt(dx*dx + dy*dy)
        target_angle = np.arctan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - yaw)
        
        cmd = Twist()
        
        # If rotating, only handle rotation
        if self.is_rotating:
            if abs(angle_diff) < self.angle_tolerance:
                self.is_rotating = False
                self.get_logger().info('Target angle reached, starting forward motion')
            else:
                cmd.angular.z = np.clip(self.k_angular * angle_diff, -self.max_angular_speed, self.max_angular_speed)
        # If not rotating, move forward
        else:
            # If angle diff becomes too large, switch back to rotation mode
            if abs(angle_diff) > self.angle_tolerance * 2:
                self.is_rotating = True
                self.get_logger().info('Angle error too large, returning to rotation mode')
            else:
                cmd.linear.x = self.linear_speed
        
        self.cmd_pub.publish(cmd)
        
        # Check if waypoint is reached
        if distance < self.goal_tolerance:
            if self.current_path_index < len(self.current_path) - 1:
                self.current_path_index += 1
                self.is_rotating = True  # Start rotating to next waypoint
                self.get_logger().info(f'Waypoint {self.current_path_index}/{len(self.current_path)} reached')
            else:
                # Final goal reached, rotate to final orientation
                angle_diff_final = self.normalize_angle(self.goal_orientation - yaw)
                if abs(angle_diff_final) < self.angle_tolerance:
                    self.get_logger().info('Final goal and orientation reached')
                    self.current_path = None
                    self.current_path_index = 0
                    self.cmd_pub.publish(Twist())  # Stop the robot
                else:
                    cmd = Twist()
                    cmd.angular.z = np.clip(self.k_angular * angle_diff_final, 
                                          -self.max_angular_speed, 
                                          self.max_angular_speed)
                    self.cmd_pub.publish(cmd)

    def planning_loop(self):
        if not self.is_map_ready or self.robot_position is None:
            return

        if self.current_path is None:
            goal_grid = self.generate_random_goal()
            if goal_grid is None:
                return

            path = self.plan_path(goal_grid)
            if path is not None:
                self.current_path = path
                self.current_path_index = 0
                self.is_rotating = True  # Start with rotation to first waypoint
                self.get_logger().info('New path generated')

    def publish_path_marker(self, path):
        """Publish path and goal markers"""
        # Path marker
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.05
        path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        for point in path:
            world_point = self.grid_to_world(point[0], point[1])
            if world_point:
                path_marker.points.append(Point(x=world_point[0], y=world_point[1], z=0.0))

        self.marker_pub.publish(path_marker)

        # Goal arrow marker
        if path:
            goal_point = path[-1]
            goal_world = self.grid_to_world(goal_point[0], goal_point[1])
            if goal_world:
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "map"
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.ns = "goal_arrow"
                arrow_marker.id = 1
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                # Position
                arrow_marker.pose.position.x = goal_world[0]
                arrow_marker.pose.position.y = goal_world[1]
                arrow_marker.pose.position.z = 0.0
                
                # Orientation using goal_orientation
                x, y, z, w = self.euler_to_quaternion(self.goal_orientation)
                arrow_marker.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
                
                # Scale
                arrow_marker.scale = Vector3(x=0.3, y=0.05, z=0.05)
                # Color
                arrow_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red arrow
                
                self.marker_pub.publish(arrow_marker)

def main(args=None):
    rclpy.init(args=args)
    explorer = RRTExplorer()
    rclpy.spin(explorer)
    explorer.sim.stopSimulation()
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()