import rclpy  
from rclpy.node import Node  
from nav_msgs.msg import OccupancyGrid  
import yaml  
import cv2
import numpy as np

class MapToTopic(Node):  
    def __init__(self):  
        super().__init__('map_to_topic')  

        #启用仿真时间
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 声明地图路径参数  
        self.declare_parameter('map_path', 'my_map.yaml')  

        # 创建地图发布器  
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)  

        # 加载地图  
        self.load_map()  

        # 定义定时器  
        self.timer = self.create_timer(1.0, self.publish_map)  

    def load_map(self):  
        try:
            # 获取参数并加载地图  
            map_path = self.get_parameter('map_path').get_parameter_value().string_value  
            with open(map_path, 'r') as file:  
                data = yaml.safe_load(file)  

            map_image = cv2.imread(data['image'], cv2.IMREAD_UNCHANGED)
            # 对数组（例如图像）中的每个元素进行按位求反操作
            map_image = np.invert(map_image)  # Invert the image if negate is set to 1
            # 镜像反转，因为图像坐标系的原点在左上，地图在左下
            map_image=cv2.flip(map_image, 0)
            
            occupancy_grid = OccupancyGrid()
            occupancy_grid.header.frame_id = 'map'
            occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            occupancy_grid.info.resolution = float(data['resolution'])
            occupancy_grid.info.width = map_image.shape[1]
            occupancy_grid.info.height = map_image.shape[0]
            occupancy_grid.info.origin.position.x = float(data['origin'][0])
            occupancy_grid.info.origin.position.y = float(data['origin'][1])
            # occupancy_grid.info.origin.position.y = -10.4
            occupancy_grid.info.origin.position.z = 0.0
            map_data = map_image.flatten()  # Flatten the map image array
            map_data[map_data == 255] = 100
            map_data[map_data == 1] = 0
            # 这边看是-1，实际上值为255
            map_data[map_data == 50] = -1
            occupancy_grid.data = map_data.astype(np.int8).tolist()

            # 声明地图
            self.map = occupancy_grid 
            self.map.header.stamp =self.get_clock().now().to_msg()

        except Exception as e:
            self.get_logger().error(f"加载地图失败：{e}")
            self.map = OccupancyGrid()
            self.map.header.frame_id = "map"

    def publish_map(self):  
        if hasattr(self, 'map'):
            self.map.header.stamp = self.get_clock().now().to_msg()  
            self.map_publisher.publish(self.map)
        else:
            self.get_logger().warn("地图未初始化！")

def main(args=None):  
    rclpy.init(args=args)  
    node = MapToTopic()  
    rclpy.spin(node)  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()  