import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('print_tf_node')
        
        #启用仿真时间
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 声明参数，默认打印odom到map的变换
        self.declare_parameter('print_frame', 'odom')
        self.print_frame = self.get_parameter('print_frame').get_parameter_value().string_value
        
        # 初始化TF2 Buffer和Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建定时器（每秒查询一次）
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # 查询从map到目标坐标系的变换
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.print_frame,  # 目标坐标系（例如odom）
                source_frame='map',             # 源坐标系（map）
                time=rclpy.time.Time()          # 获取最新可用的变换
            )
            
            # 打印变换信息
            self.get_logger().info(
                f"map -> {self.print_frame} Transform:\n"
                f"  Translation: x={transform.transform.translation.x:.2f}, "
                f"y={transform.transform.translation.y:.2f}, "
                f"z={transform.transform.translation.z:.2f}\n"
                f"  Rotation: x={transform.transform.rotation.x:.2f}, "
                f"y={transform.transform.rotation.y:.2f}, "
                f"z={transform.transform.rotation.z:.2f}, "
                f"w={transform.transform.rotation.w:.2f}"
            )
        except TransformException as ex:
            self.get_logger().error(f"变换获取失败: {ex}")

def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#这么用
#ros2 run print_tf print_tf --ros-args -p print_frame:=odom