import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry  # 更标准的里程计消息类型

class OdomToBaselink(Node):
    def __init__(self):
        super().__init__('odom_to_base_link_tf_publisher')
        
        #启用仿真时间
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 初始化动态变换广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅示例话题（可选）
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odometry,
            10
        )

    def handle_odometry(self, msg):
        # 每次收到里程计消息时发布新的变换
        transform = TransformStamped()
        
        # 使用消息的时间戳确保同步
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
        # 从里程计消息中提取位姿
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().debug('Published odom->base_link transform')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaselink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()