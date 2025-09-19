import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import cv2
import yaml
import numpy as np
import os

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        self.current_directory = os.path.dirname(os.path.abspath(__file__))
        yaml_file = os.path.join(self.current_directory, "maze.yaml")
        self.declare_parameter('map_info_file', yaml_file)
        #map_file = self.get_parameter('map_file').get_parameter_value().string_value
        map_info_file = self.get_parameter('map_info_file').get_parameter_value().string_value
        self.load_map_info(map_info_file)


        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(OccupancyGrid, '/grid_map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

    def load_map_info(self, file_path):
        try:
            with open(file_path, 'r') as file:
                config = yaml.safe_load(file)
                
                map_file = os.path.join(self.current_directory, config.get('image', "maze.pgm"))
                img_pgm = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
                self.img = img_pgm #cv2.flip(img_pgm, 0)
                if self.img is None:
                    self.get_logger().error(f"Failed to load map image: {map_file}")
                    return
            
                self.resolution = config.get('resolution', 0.05)
                self.origin = config.get('origin', [0.0, 0.0, 0.0])
                self.negate = config.get('negate', 0)
                self.occupied_thresh = config.get('occupied_thresh', 0.9)
                
                self.free_thresh = config.get('free_thresh', 0.1)
        except Exception as e:
            self.get_logger().error(f"Failed to load map info: {e}")

    def publish_map(self):
        # Convert image to occupancy grid
        map_data = []
        for pixel in self.img.flatten():
            if self.negate:
                pixel = 255 - pixel
            occ = 1.0 - (pixel / 255.0)

            if occ > self.occupied_thresh:
                map_data.append(100)
            elif occ < self.free_thresh:
                map_data.append(0)
            else:
                map_data.append(-1)

        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "world"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.img.shape[1]
        grid_msg.info.height = self.img.shape[0]

        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = self.origin[0]
        grid_msg.info.origin.position.y = self.origin[1]
        grid_msg.info.origin.position.z = self.origin[2]

        grid_msg.data = map_data

        self.publisher_.publish(grid_msg)
        self.get_logger().info("Published OccupancyGrid message")

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
