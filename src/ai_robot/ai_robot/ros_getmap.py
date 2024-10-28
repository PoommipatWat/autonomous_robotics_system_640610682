import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import csv
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from .ai_samples.OccupancyGridMap import OccupancyGridMap
from scipy import ndimage

class GetMap(Node):
    def __init__(self):
        super().__init__('get_map')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # สร้าง kernel สำหรับ dilation
        self.dilation_kernel_size = 12  # ขนาดของ kernel (3x3)
        self.dilation_kernel = np.ones((self.dilation_kernel_size, self.dilation_kernel_size))
        
        # กำหนดค่า threshold สำหรับการระบุพื้นที่
        self.obstacle_threshold = 51  # ค่ามากกว่า 80 ถือว่าเป็นสิ่งกีดขวาง (สีดำ)

        # Get package path and map directory
        package_name = 'ai_robot'  # เปลี่ยนเป็นชื่อ package ของคุณ
        package_path = get_package_share_directory(package_name)
        workspace_path = os.path.dirname(os.path.dirname(package_path))
        workspace_path = workspace_path.split('/install')[0]
        
        # Open CSV file in map directory
        csv_path = os.path.join(workspace_path, 'data_example.csv')
        self.get_logger().info(f"Reading data from {csv_path}")
        data = self.read_csv(csv_path)

        self.ogm = OccupancyGridMap(-6, 6, -6, 6, cell_size=0.05)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.gen_map(data)
        self.map_msg = self.create_msg()
        # Create publisher
        self.pub_map_ = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # Timer
        self.timer_ = self.create_timer(1.0, self.timer_callback)        
        self.get_logger().info('GetMap node initialized')

    def broadcast_map_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def create_msg(self):
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
        
        dilated_occupancy = self.binarize_and_dilate_map(occupancy_data)
        
        grid_msg.data = dilated_occupancy.flatten().tolist()

        return grid_msg

    def timer_callback(self):
        self.broadcast_map_transform()
        
        self.map_msg = self.create_msg()
        self.pub_map_.publish(self.map_msg)


    def read_csv(self, file_path):
        data = []
        with open(file_path, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                data.append([float(val) for val in row])
        return np.array(data)
    
    def gen_map(self, data):
        # กำหนดระยะห่างขั้นต่ำระหว่างจุดที่จะใช้ในการสร้างแผนที่
        min_distance = 0.1
        
        # วนรอบแถวข้อมูล CSV
        for row in data:
            robot_pos_x = row[0]
            robot_pos_y = row[1]
            robot_yaw = row[2]

            # แยกข้อมูล lidar range และ angle จากคอลัมน์ที่ 3 เป็นต้นไป
            ranges = np.array(row[3::2])
            angles = np.array(row[4::2])

            # กรองเฉพาะจุดที่อยู่ในช่วงและมีระยะห่างมากกว่าค่าที่กำหนด
            valid_indices = np.where((ranges >= 0.0) & (ranges <= 15.0))[0]
            ranges = ranges[valid_indices]
            angles = angles[valid_indices]

            # ลดจำนวนจุดข้อมูลโดยเลือกเฉพาะจุดที่มีระยะห่างกันอย่างน้อย min_distance
            selected_indices = []
            prev_x, prev_y = None, None
            for i in range(len(ranges)):
                absolute_angle = angles[i] + robot_yaw
                x_global = robot_pos_x + ranges[i] * np.cos(absolute_angle)
                y_global = robot_pos_y + ranges[i] * np.sin(absolute_angle)
                
                if prev_x is None or np.sqrt((x_global - prev_x)**2 + (y_global - prev_y)**2) >= min_distance:
                    selected_indices.append(i)
                    prev_x, prev_y = x_global, y_global

            ranges = ranges[selected_indices]
            angles = angles[selected_indices]

            # คำนวณจุดปลายใน global frame โดยใช้ NumPy
            absolute_angles = angles + robot_yaw
            x_global = robot_pos_x + ranges * np.cos(absolute_angles)
            y_global = robot_pos_y + ranges * np.sin(absolute_angles)

            # อัปเดตแผนที่
            for i in range(len(x_global)):
                self.ogm.update_line(robot_pos_x, robot_pos_y, x_global[i], y_global[i])


    def binarize_and_dilate_map(self, occupancy_data):
        """
        แปลงแผนที่ให้เป็นแบบ binary (ขาว-ดำ) ก่อนทำ closing
        พื้นที่ที่ไม่มีข้อมูล (-1) จะถูกแปลงเป็นพื้นที่ว่าง (0)
        """
        # สร้าง binary map โดยเริ่มจากให้ทุกพื้นที่เป็นสีขาว (0)
        binary_map = np.zeros_like(occupancy_data)
        
        # กำหนดค่าให้เป็นสีดำ (100) เฉพาะพื้นที่ที่เป็นสิ่งกีดขวางที่แน่นอน
        binary_map[occupancy_data >= self.obstacle_threshold] = 100
        
        # ทำ dilation บน binary map
        dilated_map = ndimage.binary_dilation(binary_map == 100, structure=self.dilation_kernel)
        
        # ทำ erosion ตาม dilation เพื่อให้เป็นการ closing
        # closed_map = ndimage.binary_erosion(dilated_map, structure=self.dilation_kernel)
        
        # แปลงกลับเป็นค่า occupancy (0-100)
        result_map = np.zeros_like(occupancy_data)
        result_map[dilated_map] = 100
        
        return result_map


def main(args=None):
    rclpy.init(args=args)
    get_map = GetMap()
    rclpy.spin(get_map)
    get_map.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()