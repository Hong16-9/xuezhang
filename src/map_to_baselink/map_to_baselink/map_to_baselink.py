import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry  # P3D插件发布的是Odometry消息

class MapToBaseLink(Node):
    def __init__(self):
        super().__init__('map_to_base_link_tf_publisher')
        
        #启用仿真时间
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 初始化动态变换广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅示例话题（可选）
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        # 处理P3D插件的数据,发布map->base_link变换
        transform = TransformStamped()
        
        # 设置时间戳和坐标系关系
        transform.header.stamp = self.get_clock().now().to_msg()  # 我改成真实时间的时间戳了
        transform.header.frame_id = 'map'          # 父坐标系（与Gazebo配置一致）
        transform.child_frame_id = 'base_link'     # 子坐标系（机器人本体）
        
        # 从Odometry消息中提取位姿
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation  # 四元数
        
        # 发布变换
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug('Published map->base_link transform')

def main(args=None):
    rclpy.init(args=args)
    node = MapToBaseLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

