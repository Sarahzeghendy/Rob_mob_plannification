#!/usr/bin/env python3
"""
Simple static map publisher node
Lit une carte depuis un fichier YAML/PGM et la publie sur /map
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
from PIL import Image
import numpy as np
import os


class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        
        # Paramètres
        self.declare_parameter('map_yaml_file', '')
        self.declare_parameter('publish_rate', 1.0)
        #self.declare_parameter('use_sim_time', True)
        
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
            
        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer pour publier périodiquement
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)
        
        self.get_logger().info(f'Static map publisher started!')
        self.get_logger().info(f'Publishing map from: {map_yaml_file}')
        self.get_logger().info(f'Publishing on topic: /map at {publish_rate} Hz')

    def load_map_from_file(self, yaml_path):
        """Charger une carte depuis un fichier YAML et PGM"""
        try:
            # Lire le fichier YAML
            with open(yaml_path, 'r') as f:
                map_metadata = yaml.safe_load(f)
            
            # Extraire les paramètres
            resolution = map_metadata['resolution']
            origin = map_metadata['origin']  # [x, y, theta]
            negate = map_metadata.get('negate', 0)
            occupied_thresh = map_metadata.get('occupied_thresh', 0.65)
            free_thresh = map_metadata.get('free_thresh', 0.196)
            
            # Construire le chemin de l'image
            image_path = map_metadata['image']
            if not os.path.isabs(image_path):
                # Chemin relatif par rapport au YAML
                map_dir = os.path.dirname(yaml_path)
                image_path = os.path.join(map_dir, image_path)
            
            self.get_logger().info(f'Loading image: {image_path}')
            
            # Charger l'image PGM
            img = Image.open(image_path)
            img_array = np.array(img)
            
            # Si negate est activé, inverser les couleurs
            if negate:
                img_array = 255 - img_array
            
            height, width = img_array.shape
            
            # Créer le message OccupancyGrid
            map_msg = OccupancyGrid()
            map_msg.header.frame_id = "map"
            
            map_msg.info.resolution = resolution
            map_msg.info.width = width
            map_msg.info.height = height
            map_msg.info.origin.position.x = origin[0]
            map_msg.info.origin.position.y = origin[1]
            map_msg.info.origin.position.z = 0.0
            
            # Quaternion pour l'orientation (simplifiée, juste yaw)
            yaw = origin[2] if len(origin) > 2 else 0.0
            map_msg.info.origin.orientation.z = np.sin(yaw / 2.0)
            map_msg.info.origin.orientation.w = np.cos(yaw / 2.0)
            
            # Convertir les pixels en valeurs d'occupation
            # Format ROS OccupancyGrid: -1 = unknown, 0 = free, 100 = occupied
            data = []
            
            # Parcourir l'image ligne par ligne (de bas en haut pour ROS)
            for y in range(height - 1, -1, -1):
                for x in range(width):
                    pixel = img_array[y, x]
                    
                    # Normaliser la valeur du pixel
                    normalized = pixel / 255.0
                    
                    if normalized > occupied_thresh:
                        # Espace libre
                        data.append(0)
                    elif normalized < free_thresh:
                        # Occupé
                        data.append(100)
                    else:
                        # Inconnu ou incertain
                        # Convertir en probabilité d'occupation
                        occ_prob = (occupied_thresh - normalized) / (occupied_thresh - free_thresh)
                        data.append(int(occ_prob * 100))
            
            map_msg.data = data
            
            self.get_logger().info(f'Map loaded successfully!')
            self.get_logger().info(f'   Size: {width}x{height} pixels')
            self.get_logger().info(f'   Resolution: {resolution} m/pixel')
            self.get_logger().info(f'   Origin: ({origin[0]:.2f}, {origin[1]:.2f})')
            
            return map_msg
            
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            return None

    def publish_map(self):
        """Publier la carte"""
        if self.map_msg is not None:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()