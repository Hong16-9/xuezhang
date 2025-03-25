import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import math

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        
        #启用仿真时间
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.subscription = self.create_subscription(
            LaserScan,
            '/top_laser',  # 订阅的LaserScan主题
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/point_cloud', 10)

    def scan_callback(self, msg):
        points = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0  # 2D激光雷达通常z为0
                points.append((x, y, z))
        
        # 创建PointCloud2消息
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id  # 保持与激光相同的坐标系
        
        # 使用sensor_msgs_py生成点云
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()