import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import coppeliasim_zmqremoteapi_client as zmqRemoteApi
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class RosScan(Node):
    def __init__(self):
        super().__init__('ros_scan')
        
        self.client = zmqRemoteApi.RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.sim.startSimulation()

        self.ground_handle = self.sim.getObject('/Floor')
        self.sensorbase_handle = self.sim.getObject('/robot')
        self.sensorhandle1 = self.sim.getObject('/robot/fastHokuyo/sensor1')
        self.sensorhandle2 = self.sim.getObject('/robot/fastHokuyo/sensor2')

        self.R_sb_s1 = self.sim.getObjectMatrix(self.sensorhandle1, self.sensorbase_handle)
        self.R_sb_s2 = self.sim.getObjectMatrix(self.sensorhandle2, self.sensorbase_handle)

        # Create publisher and broadcaster
        self.scan_pub_ = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timers_ = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info('RosScan node has been initialized')

    def timer_callback(self):
        # Broadcast transform from base_link to laser
        self.broadcast_laser_transform()
        
        res1, dist1, point1 = self.sim.readVisionSensor(self.sensorhandle1)
        res2, dist2, point2 = self.sim.readVisionSensor(self.sensorhandle2)

        l_x, l_y, length_angle = self.get_laser_data(point1, point2)

        lengths = length_angle[::2]  # ค่า length อยู่ในตำแหน่งเลขคู่
        angles = length_angle[1::2]  # ค่า angle อยู่ในตำแหน่งเลขคี่

        # เรียงข้อมูลตามมุม angle
        sorted_indices = np.argsort(angles)
        angles = np.array(angles)[sorted_indices]
        lengths = np.array(lengths)[sorted_indices]

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser'  # เปลี่ยนจาก base_link เป็น laser

        scan_msg.angle_min = angles[0]
        scan_msg.angle_max = angles[-1]
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (len(angles) - 1) if len(angles) > 1 else 0.0
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1  # 20Hz
        scan_msg.range_min = 0.0
        scan_msg.range_max = 15.0

        # Filter out invalid readings
        filtered_lengths = [float(l) if 0.0 <= l <= 15.0 else float('inf') for l in lengths]
        scan_msg.ranges = filtered_lengths

        self.scan_pub_.publish(scan_msg)

    def broadcast_laser_transform(self):
        """Broadcast static transform from base_link to laser frame"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        
        # Set translation (if LIDAR is offset from robot center)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Set rotation (if LIDAR is rotated relative to robot)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

    def get_laser_data(self, point1, point2):
        skip = 4
        data = np.array(point1)
        x1 = np.array(data[2::skip])
        y1 = np.array(data[3::skip])
        z1 = np.array(data[4::skip])

        data2 = np.array(point2)
        x2 = np.array(data2[2::skip])
        y2 = np.array(data2[3::skip])
        z2 = np.array(data2[4::skip])

        xyz1 = np.hstack((x1,y1,z1))
        Rot1 = np.array(self.R_sb_s1).reshape(3,4)
        Rot1 = Rot1[0:3,0:3]

        xyz2 = np.hstack((x2,y2,z2))
        Rot2 = np.array(self.R_sb_s2).reshape(3,4)
        Rot2 = Rot2[0:3,0:3]

        u = np.array([])
        v = np.array([])
        length_angle = np.array([])

        for i in range(len(x1)):
            ww = np.array([x1[i],y1[i],z1[i]]).reshape(3,1)
            uvw = np.matmul(Rot1,ww)

            le, th = self.conv_pc2_length_phi(uvw[0,0],uvw[1,0])
            length_angle = np.append(length_angle, [le, th])

            u = np.append(u,uvw[0,0])
            v = np.append(v,uvw[1,0])

        for i in range(len(x2)):
            ww = np.array([x2[i],y2[i],z2[i]]).reshape(3,1)
            uvw = np.matmul(Rot2,ww)

            le, th = self.conv_pc2_length_phi(uvw[0,0],uvw[1,0])
            length_angle = np.append(length_angle, [le, th])

            u = np.append(u,uvw[0,0])
            v = np.append(v,uvw[1,0])

        return u, v, length_angle
    
    def conv_pc2_length_phi(self,x,y):
        th = np.arctan2(y,x)
        length = np.sqrt(x*x+y*y)
        return length, th

def main(args=None):
    rclpy.init(args=args)

    try:
        ros_scan = RosScan()
        rclpy.spin(ros_scan)
    except Exception as e:
        print(f'An error occurred: {e}')
    finally:
        if 'ros_scan' in locals():
            ros_scan.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()