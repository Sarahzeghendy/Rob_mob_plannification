#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid
import yaml
from PIL import Image
import numpy as np
import os


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        
        # Param
        self.declare_parameter('map_yaml_file', '')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('global_frame', 'map')
        
        map_yaml_file = self.get_parameter('map_yaml_file').value
        publish_rate = self.get_parameter('publish_rate').value
        
        if not map_yaml_file:
            self.get_logger().error('No map_yaml_file specified!')
            self.get_logger().info('Please provide map_yaml_file parameter')
            return
        
        # Charger la carte
        self.map_msg = self.load_map_from_file(map_yaml_file)
        
        if self.map_msg is None:
            self.get_logger().error('Failed to load map!')
            return
        
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )
        
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)
        
        self.get_logger().info('Map Manager started!')
        self.get_logger().info(f'Publishing map from: {map_yaml_file}')
        self.get_logger().info(f'Map size: {self.map_msg.info.width}x{self.map_msg.info.height}')
        self.get_logger().info(f'Resolution: {self.map_msg.info.resolution} m/cell')
    
    def load_map_from_file(self, yaml_path):
        try:
            self.get_logger().info(f'Loading map from: {yaml_path}')
            
            with open(yaml_path, 'r') as f:
                map_metadata = yaml.safe_load(f)
            
            resolution = map_metadata['resolution']
            origin = map_metadata['origin']
            # negate = map_metadata.get('negate', 0)
            occupied_thresh = map_metadata.get('occupied_thresh', 0.65)
            free_thresh = map_metadata.get('free_thresh', 0.196)
            
            image_path = map_metadata['image']
            if not os.path.isabs(image_path):
                map_dir = os.path.dirname(yaml_path)
                image_path = os.path.join(map_dir, image_path)
            
            self.get_logger().info(f'Loading image: {image_path}')
            
            img = Image.open(image_path)
            img_array = np.array(img)
            
            # if negate:
            #     img_array = 255 - img_array
            
            height, width = img_array.shape
            
            map_msg = OccupancyGrid()
            map_msg.header.frame_id = self.get_parameter('global_frame').value
            map_msg.header.stamp = self.get_clock().now().to_msg()
            
            map_msg.info.resolution = resolution
            map_msg.info.width = width
            map_msg.info.height = height
            map_msg.info.origin.position.x = origin[0]
            map_msg.info.origin.position.y = origin[1]
            map_msg.info.origin.position.z = 0.0
            
            yaw = origin[2] if len(origin) > 2 else 0.0
            map_msg.info.origin.orientation.z = np.sin(yaw / 2.0)
            map_msg.info.origin.orientation.w = np.cos(yaw / 2.0)
            
            # Convertir en format ROS OccupancyGrid
            # -1 = unknown, 0 = free, 100 = occupied
            data = []
            
            for y in range(height - 1, -1, -1):
                for x in range(width):
                    pixel = img_array[y, x]
                    normalized = pixel / 255.0
                    
                    if normalized > occupied_thresh:
                        data.append(0)  # Free
                    elif normalized < free_thresh:
                        data.append(100)  # Occupied
                    else:
                        # Unknown/uncertain
                        occ_prob = (occupied_thresh - normalized) / (occupied_thresh - free_thresh)
                        data.append(int(occ_prob * 100))
            
            map_msg.data = data
            
            self.get_logger().info('Map loaded successfully!')
            
            return map_msg
            
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            return None
    
    def publish_map(self):
      
        if self.map_msg is not None:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
