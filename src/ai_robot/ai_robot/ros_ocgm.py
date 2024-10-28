import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from .ai_samples.OccupancyGridMap import OccupancyGridMap
import coppeliasim_zmqremoteapi_client as zmqRemoteApi

import csv
import os
from ament_index_python.packages import get_package_share_directory


class Ocgm_Ros(Node):
    def __init__(self):
        super().__init__('test')

        package_name = 'ai_robot'  # เปลี่ยนเป็นชื่อ package ของคุณ
        package_path = get_package_share_directory(package_name)
        workspace_path = os.path.dirname(os.path.dirname(package_path))
        self.map_dir = workspace_path.split('/install')[0]
        
        # Create map directory if it doesn't exist
        if not os.path.exists(self.map_dir):
            os.makedirs(self.map_dir)

        # Open CSV file in map directory
        csv_path = os.path.join(self.map_dir, 'data.csv')
        self.get_logger().info(f"Saving data to {csv_path}")
        self.f = open(csv_path, 'w')
        self.f = open(csv_path, 'a', newline='')
        self.writer = csv.writer(self.f)

        # CoppeliaSim setup
        self.client = zmqRemoteApi.RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.startSimulation()

        # Get handles
        self.ground_handle = self.sim.getObject('/Floor')
        self.robot_handle = self.sim.getObject('/robot')
        self.sensorhandle1 = self.sim.getObject('/robot/fastHokuyo/sensor1')
        self.sensorhandle2 = self.sim.getObject('/robot/fastHokuyo/sensor2')

        # Get transformation matrices
        self.R_sb_s1 = self.sim.getObjectMatrix(self.sensorhandle1, self.robot_handle)
        self.R_sb_s2 = self.sim.getObjectMatrix(self.sensorhandle2, self.robot_handle)

        # Publishers
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer_ = self.create_timer(0.2, self.timer_callback)

        # Initialize OccupancyGridMap
        self.ogm = OccupancyGridMap(-7, 7, -7, 7, cell_size=0.05)

        self.get_logger().info('TEST node initialized')

    def broadcast_map_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def get_robot_pose(self):
        position = np.array(self.sim.getObjectPosition(self.robot_handle, self.ground_handle))
        orientation = np.array(self.sim.getObjectOrientation(self.robot_handle, self.ground_handle))
        return position, orientation[2]  # Return position and yaw

    def get_laser_data(self, point1, point2):
        skip = 4
        length_angle = np.array([])
        
        # Process data from both sensors
        for points, rot_matrix in [(point1, self.R_sb_s1), (point2, self.R_sb_s2)]:
            if not points:
                continue
                
            data = np.array(points)
            x = np.array(data[2::skip])
            y = np.array(data[3::skip])
            z = np.array(data[4::skip])
            
            Rot = np.array(rot_matrix).reshape(3,4)[0:3,0:3]
            
            for i in range(len(x)):
                point = np.array([x[i], y[i], z[i]]).reshape(3,1)
                transformed = np.matmul(Rot, point)
                
                length, angle = self.conv_pc2_length_phi(transformed[0,0], transformed[1,0])
                length_angle = np.append(length_angle, [length, angle])
        
        return length_angle

    def conv_pc2_length_phi(self, x, y):
        angle = np.arctan2(y, x)
        length = np.sqrt(x*x + y*y)
        return length, angle

    def timer_callback(self):
        data = np.array([])

        self.broadcast_map_transform()
        
        # Get robot pose
        robot_pos, robot_yaw = self.get_robot_pose()
        
        # Get laser data
        res1, dist1, point1 = self.sim.readVisionSensor(self.sensorhandle1)
        res2, dist2, point2 = self.sim.readVisionSensor(self.sensorhandle2)

        data = np.append(data, [robot_pos[0], robot_pos[1], robot_yaw])

        # Process laser data
        length_angle = self.get_laser_data(point1, point2)

        data = np.append(data, length_angle)
        self.writer.writerow(data)
        
        if len(length_angle) > 0:
            lengths = length_angle[::2]
            angles = length_angle[1::2]

            # Convert to global coordinates
            for length, angle in zip(lengths, angles):
                if 0.0 <= length <= 15.0:  # Valid range check
                    # อ้างอิงมุมกับ global frame โดยตรง
                    absolute_angle = angle + robot_yaw
                    
                    # คำนวณจุดปลายใน global frame
                    x_global = robot_pos[0] + length * np.cos(absolute_angle)
                    y_global = robot_pos[1] + length * np.sin(absolute_angle)
                    
                    # Update map
                    self.ogm.update_line(robot_pos[0], robot_pos[1], x_global, y_global)

            # Publish map
            self.publish_map()

    def publish_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        
        grid_msg.info.resolution = self.ogm.cell_size
        grid_msg.info.width = self.ogm.grid_width
        grid_msg.info.height = self.ogm.grid_height
        
        grid_msg.info.origin.position.x = float(self.ogm.x_min)
        grid_msg.info.origin.position.y = float(self.ogm.y_min)
        grid_msg.info.origin.orientation.w = 1.0
        
        # Process map data
        raw_map = self.ogm.get_map()
        occupancy_data = (raw_map * 100).astype(int)
        occupancy_data[raw_map < 0] = -1
        occupancy_data = np.clip(occupancy_data, -1, 100)
        
        grid_msg.data = occupancy_data.flatten().tolist()
        self.map_publisher.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        test = Ocgm_Ros()
        rclpy.spin(test)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'test' in locals():
            test.sim.stopSimulation()
            test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()