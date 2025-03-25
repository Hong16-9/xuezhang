import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_py as tf2
from collections import deque
import struct
from rclpy.qos import QoSProfile
from scipy.spatial import KDTree
from transformations import quaternion_matrix  # 确保安装 tf_transformations

class LocalCostmap(Node):
    def __init__(self):
        super().__init__('local_costmap')

        # 启用仿真时间
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 参数声明
        self.declare_parameters(namespace='', parameters=[
            ('target_link', 'base_link'),
            ('inflated_range_x', 0.4),
            ('inflated_range_y', 0.4),
            ('costmap_size', 9.0),
            ('max_point_cloud', 1.6),
            ('min_point_cloud', 0.0),
            ('voxel_grid_resolution', 0.1),
            ('search_radius', 0.1),
            ('search_num', 4),
            ('queue_size', 5)
        ])
        # 获取参数
        self.target_link = self.get_parameter('target_link').value
        self.inflated_range_x = self.get_parameter('inflated_range_x').value
        self.inflated_range_y = self.get_parameter('inflated_range_y').value
        self.costmap_size = self.get_parameter('costmap_size').value
        self.max_z = self.get_parameter('max_point_cloud').value
        self.min_z = self.get_parameter('min_point_cloud').value
        self.voxel_res = self.get_parameter('voxel_grid_resolution').value
        self.search_radius = self.get_parameter('search_radius').value
        self.search_num = self.get_parameter('search_num').value
        self.queue_size = self.get_parameter('queue_size').value
        
        # 初始化变量
        self.update_map = False
        self.global_map = None
        self.global_origin = (0.0, 0.0)
        self.global_res = 0.05
        self.global_width = 0
        self.global_height = 0
        
        # 发布者
        self.local_costmap_pub = self.create_publisher(OccupancyGrid, 'local_costmap', 10)
        self.raw_costmap_pub = self.create_publisher(OccupancyGrid, 'raw_costmap', 10)
        
        # 订阅者
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        qos = QoSProfile(depth=10)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.cloud_callback,
            qos)
        
        # TF相关
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud_queue = deque(maxlen=self.queue_size)
    
    def map_callback(self, msg):
        if not self.update_map:
            self.global_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
            self.global_res = msg.info.resolution
            self.global_width = msg.info.width
            self.global_height = msg.info.height
            self.global_map = np.array(msg.data, dtype=np.int8).reshape((self.global_height, self.global_width))
            self.update_map = True
            self.get_logger().info("Global map received")
    
    def cloud_callback(self, msg):
        if not self.update_map:
            return
        
        # 转换点云到odom坐标系
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                msg.header.frame_id,
                msg.header.stamp)
            transformed_cloud = self.transform_point_cloud(msg, transform)
            self.cloud_queue.append(transformed_cloud)
        except TransformException as e:
            self.get_logger().error(f'TF error in cloud transform: {e}')
            return
        
        # 合并点云
        combined = []
        for cloud_msg in self.cloud_queue:
            points = np.array(list(self.read_points(cloud_msg)), dtype=np.float32)
            if points.shape[0] == 0:
                continue
            # Z轴过滤
            mask = (points[:, 2] > self.min_z) & (points[:, 2] < self.max_z)
            points = points[mask]
            # 距离过滤
            dist = np.linalg.norm(points[:, :2], axis=1)
            mask = dist < self.costmap_size / 2
            combined.append(points[mask])
        
        if not combined:
            return
        points = np.vstack(combined)
        
        # 体素滤波
        voxel_grid = {}
        for pt in points:
            voxel = (int(pt[0] / self.voxel_res), int(pt[1] / self.voxel_res))
            if voxel not in voxel_grid:
                voxel_grid[voxel] = pt
        filtered = np.array(list(voxel_grid.values()))
        
        # 离群点去除（使用KD树）
        if filtered.shape[0] > 0:
            tree = KDTree(filtered[:, :2])
            keep_mask = []
            for i, pt in enumerate(filtered):
                indices = tree.query_ball_point(pt[:2], r=self.search_radius)
                count = len(indices) - 1
                keep_mask.append(count >= self.search_num)
            filtered = filtered[keep_mask]
        
        # 生成原始代价地图
        costmap = OccupancyGrid()
        costmap.header.frame_id = 'odom'
        costmap.info.resolution = self.global_res
        size = int(self.costmap_size / self.global_res)
        costmap.info.width = size
        costmap.info.height = size
        costmap.data = [-1] * (size * size)
        
        # 获取当前位置
        try:
            transform = self.tf_buffer.lookup_transform('odom', self.target_link, msg.header.stamp)
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
        except TransformException as e:
            self.get_logger().error(f'TF error in position lookup: {e}')
            return
        
        # 设置原点
        costmap.info.origin.position.x = current_x - self.costmap_size / 2
        costmap.info.origin.position.y = current_y - self.costmap_size / 2
        
        # 填充障碍物
        for pt in filtered:
            map_x = int((pt[0] - costmap.info.origin.position.x) / self.global_res)
            map_y = int((pt[1] - costmap.info.origin.position.y) / self.global_res)
            
            if 0 <= map_x < size and 0 <= map_y < size:
                dx = int(self.inflated_range_x / self.global_res)
                dy = int(self.inflated_range_y / self.global_res)
                x_start = max(0, map_x - dx)
                x_end = min(size, map_x + dx + 1)
                y_start = max(0, map_y - dy)
                y_end = min(size, map_y + dy + 1)
                
                for x in range(x_start, x_end):
                    for y in range(y_start, y_end):
                        idx = y * size + x
                        costmap.data[idx] = 100
        
        # 发布原始代价地图
        costmap.header.stamp = self.get_clock().now().to_msg()
        self.raw_costmap_pub.publish(costmap)
        
        # 融合全局地图
        if self.global_map is not None:
            global_start_x = int((costmap.info.origin.position.x - self.global_origin[0]) / self.global_res)
            global_start_y = int((costmap.info.origin.position.y - self.global_origin[1]) / self.global_res)
            
            for x in range(size):
                for y in range(size):
                    gx = global_start_x + x
                    gy = global_start_y + y
                    if 0 <= gx < self.global_width and 0 <= gy < self.global_height:
                        if self.global_map[gy, gx] == 100:
                            idx = y * size + x
                            costmap.data[idx] = 100
        
        # 发布局部代价地图
        costmap.header.stamp = self.get_clock().now().to_msg()
        self.local_costmap_pub.publish(costmap)
        
        # 添加调试信息
        self.get_logger().info(f"Published local costmap with stamp: {costmap.header.stamp}")
        self.get_logger().info(f"Map data min: {min(costmap.data)}, max: {max(costmap.data)}")

    def transform_point_cloud(self, cloud_msg, transform):
        # 从 PointCloud2 提取点
        points = np.array(list(self.read_points(cloud_msg)), dtype=np.float32)
        if points.shape[0] == 0:
            return cloud_msg

        # 获取变换的平移和旋转
        trans = transform.transform.translation
        rot = transform.transform.rotation
        trans_vec = np.array([trans.x, trans.y, trans.z])
        rot_matrix = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])[:3, :3]

        # 应用变换
        transformed_points = (rot_matrix @ points.T).T + trans_vec

        # 创建新的 PointCloud2 消息
        new_cloud = PointCloud2()
        new_cloud.header.frame_id = 'odom'
        new_cloud.header.stamp = cloud_msg.header.stamp
        new_cloud.height = 1
        new_cloud.width = len(transformed_points)
        new_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        new_cloud.is_bigendian = False
        new_cloud.point_step = 12  # 3 floats * 4 bytes
        new_cloud.row_step = new_cloud.point_step * new_cloud.width
        new_cloud.data = transformed_points.tobytes()

        return new_cloud

    def read_points(self, cloud_msg):
        fmt = '<fff'  # x, y, z 为 float32
        point_size = struct.calcsize(fmt)
        for offset in range(0, len(cloud_msg.data), point_size):
            x, y, z = struct.unpack_from(fmt, cloud_msg.data, offset)
            yield (x, y, z)

def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
