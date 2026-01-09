#!/usr/bin/env python3
"""
Planificateur A* qui utilise une carte sauvegardée
Charge une carte depuis un fichier YAML/PGM au lieu d'attendre le topic /map
"""

import math
import numpy as np
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

import tf2_ros

from .map_utils import occgrid_to_2d, inflate_occupancy, world_to_cell, cell_to_world
from .astar import astar_occ_grid


def yaw_from_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AStarPlannerWithSavedMap(Node):
    def __init__(self):
        super().__init__("tb4_astar_planner_saved_map")

        # Paramètres pour charger la carte
        self.declare_parameter("map_yaml_path", "")  # Chemin vers le fichier .yaml de la carte
        self.declare_parameter("use_topic_map", False)  # Si True, utilise aussi le topic /map
        
        # Paramètres existants
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/astar_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_map", True)  # Publier la carte chargée

        self.declare_parameter("robot_radius_m", 0.20)
        self.declare_parameter("occ_thresh", 50)
        self.declare_parameter("unknown_is_free", False)
        self.declare_parameter("allow_diag", True)

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("auto_use_robot_pose", True)

        # Paramètres du contrôleur
        self.declare_parameter("lookahead_m", 0.30)
        self.declare_parameter("v_max", 0.20)
        self.declare_parameter("w_max", 1.2)
        self.declare_parameter("yaw_kp", 1.8)
        self.declare_parameter("goal_tolerance_m", 0.12)

        # Publishers
        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        
        # Publisher pour la carte
        if self.get_parameter("publish_map").value:
            self.map_pub = self.create_publisher(OccupancyGrid, "/map_static", 10)

        # Subscribers
        self.init_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initialpose_topic").value,
            self.on_initialpose,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.on_goal,
            10
        )
        
        # écouter le topic /map
        if self.get_parameter("use_topic_map").value:
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                self.get_parameter("map_topic").value,
                self.on_map,
                10
            )

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # État
        self.map_msg = None
        self.occ_2d = None
        self.inflated_2d = None

        self.start_world = None
        self.goal_world = None

        self.path_world = []
        self.path_idx = 0

        # Timer pour le contrôle
        self.timer = self.create_timer(0.05, self.on_control)
        
        # Timer pour publier la carte
        if self.get_parameter("publish_map").value:
            self.map_pub_timer = self.create_timer(1.0, self.publish_loaded_map)

        self.get_logger().info("="*60)
        self.get_logger().info("A* Planner with Saved Map started!")
        
        # Charger la carte depuis le fichier
        map_yaml_path = self.get_parameter("map_yaml_path").value
        if map_yaml_path:
            self.load_map_from_file(map_yaml_path)
        else:
            self.get_logger().warn("No map file specified. Set 'map_yaml_path' parameter!")
            self.get_logger().info("Example: ros2 run tb4_astar_planner planner_node --ros-args -p map_yaml_path:=/path/to/map.yaml")
        
        self.get_logger().info("="*60)

    def load_map_from_file(self, yaml_path):
        try:
            self.get_logger().info(f"Loading map from: {yaml_path}")
            
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
            if not image_path.startswith('/'):
                # Chemin relatif
                import os
                map_dir = os.path.dirname(yaml_path)
                image_path = os.path.join(map_dir, image_path)
            
            self.get_logger().info(f"Loading image: {image_path}")
            
            # Charger l'image PGM
            img = Image.open(image_path)
            img_array = np.array(img)
            
            # Convertir en format OccupancyGrid
            # Les valeurs PGM: 254=free, 0=occupied, 205=unknown
            height, width = img_array.shape
            
            # Créer le message OccupancyGrid
            self.map_msg = OccupancyGrid()
            self.map_msg.header.frame_id = self.get_parameter("global_frame").value
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.map_msg.info.resolution = resolution
            self.map_msg.info.width = width
            self.map_msg.info.height = height
            self.map_msg.info.origin.position.x = origin[0]
            self.map_msg.info.origin.position.y = origin[1]
            self.map_msg.info.origin.position.z = 0.0
            self.map_msg.info.origin.orientation.w = 1.0
            
            # Convertir les pixels en valeurs d'occupation
            data = []
            for row in reversed(img_array):  # ROS utilise row-major bottom-left origin
                for pixel in row:
                    if pixel == 254:  # Free
                        data.append(0)
                    elif pixel == 0:  # Occupied
                        data.append(100)
                    elif pixel == 205:  # Unknown
                        data.append(-1)
                    else:
                        # Interpoler
                        normalized = pixel / 255.0
                        if normalized > occupied_thresh:
                            data.append(0)
                        elif normalized < free_thresh:
                            data.append(100)
                        else:
                            occ = int((1.0 - normalized) * 100)
                            data.append(occ)
            
            self.map_msg.data = data
            
            # Convertir en array 2D pour le planning
            self.occ_2d = occgrid_to_2d(data, width, height)
            
            # Inflater les obstacles
            res = resolution
            robot_radius = float(self.get_parameter("robot_radius_m").value)
            inflation_cells = int(robot_radius / res)
            occ_thresh = int(self.get_parameter("occ_thresh").value)
            
            self.inflated_2d = inflate_occupancy(self.occ_2d, inflation_cells, occ_thresh=occ_thresh)
            
            self.get_logger().info(f"Map loaded successfully!")
            self.get_logger().info(f"   Size: {width}x{height} pixels")
            self.get_logger().info(f"   Resolution: {resolution} m/pixel")
            self.get_logger().info(f"   Origin: ({origin[0]:.2f}, {origin[1]:.2f})")
            
            # Si on a déjà un goal, replanifier
            if self.goal_world is not None:
                self.try_plan()
                
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            self.get_logger().info("Make sure the YAML and PGM files exist and are readable")

    def publish_loaded_map(self):
        """Publier la carte chargée pour visualization dans RViz"""
        if self.map_msg is not None:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_msg)

    def on_map(self, msg: OccupancyGrid):
        """Callback optionnel si on veut aussi écouter le topic /map"""
        self.get_logger().info(f"Received dynamic map update: {msg.info.width}x{msg.info.height}")
        # On peut choisir d'ignorer ou de mettre à jour avec cette nouvelle carte
        if self.get_parameter("use_topic_map").value:
            self.map_msg = msg
            W = msg.info.width
            H = msg.info.height
            self.occ_2d = occgrid_to_2d(msg.data, W, H)
            
            res = msg.info.resolution
            robot_radius = float(self.get_parameter("robot_radius_m").value)
            inflation_cells = int(robot_radius / res)
            occ_thresh = int(self.get_parameter("occ_thresh").value)
            
            self.inflated_2d = inflate_occupancy(self.occ_2d, inflation_cells, occ_thresh=occ_thresh)
            
            if self.goal_world is not None:
                self.try_plan()

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        self.start_world = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Manual start set: ({self.start_world[0]:.2f}, {self.start_world[1]:.2f})")
        self.try_plan()

    def on_goal(self, msg: PoseStamped):
        new_goal = (msg.pose.position.x, msg.pose.position.y)
        
        # ✅ Éviter de replanifier si c'est le même goal
        if self.goal_world is not None:
            dist = math.hypot(new_goal[0] - self.goal_world[0], 
                            new_goal[1] - self.goal_world[1])
            if dist < 0.1:  # Même goal (tolérance 10cm)
                self.get_logger().debug("Same goal received, ignoring")
                return
        
        self.goal_world = new_goal
        self.get_logger().info(f"Goal received: ({self.goal_world[0]:.2f}, {self.goal_world[1]:.2f})")
        self.try_plan()

    def try_plan(self):
        if self.map_msg is None or self.inflated_2d is None:
            self.get_logger().warn("No map loaded yet, cannot plan.")
            self.get_logger().info("Set the 'map_yaml_path' parameter to load a saved map")
            return
        
        if self.goal_world is None:
            self.get_logger().info("Waiting for goal...")
            return

        # Auto-use robot pose if enabled
        auto_use = bool(self.get_parameter("auto_use_robot_pose").value)
        if auto_use and self.start_world is None:
            pose = self.get_robot_pose()
            if pose is not None:
                x, y, _ = pose
                self.start_world = (x, y)
                self.get_logger().info(f"Using current robot pose as start: ({x:.2f}, {y:.2f})")
            else:
                self.get_logger().warn("Cannot get robot pose from TF. Waiting...")
                return
        
        if self.start_world is None:
            self.get_logger().info("Waiting for start position...")
            return

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        res = self.map_msg.info.resolution

        start_rc = world_to_cell(self.start_world[0], self.start_world[1], origin_x, origin_y, res)
        goal_rc = world_to_cell(self.goal_world[0], self.goal_world[1], origin_x, origin_y, res)

        self.get_logger().info(f"Planning from {start_rc} to {goal_rc}...")

        allow_diag = bool(self.get_parameter("allow_diag").value)
        occ_thresh = int(self.get_parameter("occ_thresh").value)
        unknown_is_free = bool(self.get_parameter("unknown_is_free").value)

        try:
            path_rc = astar_occ_grid(
                self.inflated_2d,
                start_rc,
                goal_rc,
                allow_diag=allow_diag,
                occ_thresh=occ_thresh,
                unknown_is_free=unknown_is_free
            )
        except Exception as e:
            self.get_logger().error(f"Planning error: {e}")
            self.stop_robot()
            return

        if path_rc is None:
            self.get_logger().warn("No path found!")
            self.path_world = []
            self.path_idx = 0
            self.stop_robot()
            return

        # Convert to world coordinates
        self.path_world = [cell_to_world(r, c, origin_x, origin_y, res) for (r, c) in path_rc]
        self.path_idx = 0

        self.publish_path()
        self.get_logger().info(f"Path planned: {len(self.path_world)} waypoints")

    def publish_path(self):
        if self.map_msg is None:
            return
        path_msg = Path()
        path_msg.header.frame_id = self.get_parameter("global_frame").value
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in self.path_world:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

    def get_robot_pose(self):
        global_frame = self.get_parameter("global_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                global_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = yaw_from_quat(q)
            return x, y, yaw
        except Exception as e:
            if not hasattr(self, '_tf_error_count'):
                self._tf_error_count = 0
            self._tf_error_count += 1
            if self._tf_error_count % 100 == 1:
                self.get_logger().warn(f"TF lookup failed: {str(e)[:100]}")
            return None

    def stop_robot(self):
        """Send zero velocity command"""
        t = Twist()
        self.cmd_pub.publish(t)

    def on_control(self):
        """Main control loop"""
        if not self.path_world:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose
        goal_tol = float(self.get_parameter("goal_tolerance_m").value)

        # Check if goal reached
        gx, gy = self.path_world[-1]
        dist_to_goal = math.hypot(gx - x, gy - y)
        
        if dist_to_goal < goal_tol:
            self.stop_robot()
            self.get_logger().info(f"Goal reached! Distance: {dist_to_goal:.3f}m")
            self.path_world = []
            self.start_world = None
            self.goal_world = None 
            self.path_idx = 0    
            return

        lookahead = float(self.get_parameter("lookahead_m").value)

        # Advance path index
        while self.path_idx < len(self.path_world) - 1:
            wx, wy = self.path_world[self.path_idx]
            if math.hypot(wx - x, wy - y) < lookahead * 0.6:
                self.path_idx += 1
            else:
                break

        tx, ty = self.path_world[self.path_idx]

        # Heading control
        angle_to_target = math.atan2(ty - y, tx - x)
        ang_err = self.wrap_to_pi(angle_to_target - yaw)

        kp = float(self.get_parameter("yaw_kp").value)
        w = kp * ang_err

        w_max = float(self.get_parameter("w_max").value)
        w = max(-w_max, min(w_max, w))

        v_max = float(self.get_parameter("v_max").value)
        v = v_max * max(0.0, 1.0 - abs(ang_err) / 1.2)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    @staticmethod
    def wrap_to_pi(a):
        """Wrap angle to [-pi, pi]"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main():
    rclpy.init()
    node = AStarPlannerWithSavedMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()