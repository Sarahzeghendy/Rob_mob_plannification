#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys


class GoalInputNode(Node):
    def __init__(self):
        super().__init__("goal_input")
        
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("global_frame", "map")
        
        self.goal_pub = self.create_publisher(
            PoseStamped, 
            self.get_parameter("goal_topic").value, 
            10
        )
        
        self.get_logger().info("Goal Input Node started!")
        
    def publish_goal(self, x, y, yaw=0.0):
        """Publier un objectif (x, y, yaw en radians)"""
        msg = PoseStamped()
        msg.header.frame_id = self.get_parameter("global_frame").value
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        
        import math
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Goal published: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")
        return True


def interactive_mode(node):
    print("Entrez les coordonnées de l'objectif dans le repère 'map'")
    print("Tapez 'quit' ou 'q' pour quitter\n")
    
    while rclpy.ok():
        try:
            # Demander X
            x_input = input("X (mètres) : ").strip()
            if x_input.lower() in ['q', 'quit', 'exit']:
                break
            x = float(x_input)
            
            # Demander Y
            y_input = input("Y (mètres) : ").strip()
            if y_input.lower() in ['q', 'quit', 'exit']:
                break
            y = float(y_input)
            
            # Demander Yaw
            yaw_input = input("Yaw en degrés [0] : ").strip()
            if yaw_input.lower() in ['q', 'quit', 'exit']:
                break
            
            if yaw_input == "":
                yaw = 0.0
            else:
                import math
                yaw = math.radians(float(yaw_input))
            
            # Publier
            node.publish_goal(x, y, yaw)
            print("\nObjectif envoyé! Entrez un nouvel objectif ou 'q' pour quitter.\n")
            
        except ValueError:
            print("Erreur: entrez des nombres valides")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Erreur: {e}")
    
    print("\nAu revoir!")


def waypoint_mode(node, waypoints_file):
    
    print(f"\nMode WAYPOINTS - Lecture depuis {waypoints_file}")
    
    try:
        with open(waypoints_file, 'r') as f:
            lines = f.readlines()
        
        waypoints = []
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) >= 2:
                x, y = float(parts[0]), float(parts[1])
                yaw = float(parts[2]) if len(parts) >= 3 else 0.0
                waypoints.append((x, y, yaw))
        
        print(f"{len(waypoints)} waypoints chargés")
        
        for i, (x, y, yaw) in enumerate(waypoints, 1):
            input(f"\nAppuyez sur Entrée pour envoyer le waypoint {i}/{len(waypoints)}...")
            node.publish_goal(x, y, yaw)
            print(f"Waypoint {i}/{len(waypoints)} envoyé: ({x:.2f}, {y:.2f})")
        
        print("\nTous les waypoints ont été envoyés!")
        
    except FileNotFoundError:
        print(f"Fichier non trouvé: {waypoints_file}")
    except Exception as e:
        print(f"Erreur: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalInputNode()
    
    if len(sys.argv) == 1:
        try:
            interactive_mode(node)
        except KeyboardInterrupt:
            pass
    
    elif len(sys.argv) >= 3 and sys.argv[1] not in ['--waypoints', '-w']:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            yaw = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.0
            
            import math
            if abs(yaw) > 2 * math.pi:
                yaw = math.radians(yaw)
            
            node.publish_goal(x, y, yaw)
            print("Objectif envoyé")
            
        except ValueError:
            print("Usage: ros2 run tb4_astar_planner goal_input <x> <y> [yaw_degrees]")
    
    elif sys.argv[1] in ['--waypoints', '-w'] and len(sys.argv) >= 3:
        waypoints_file = sys.argv[2]
        waypoint_mode(node, waypoints_file)
    
    else:
        print("\nUsage:")
        print("  Mode interactif:")
        print("    ros2 run tb4_astar_planner goal_input")
        print("\n  Mode direct:")
        print("    ros2 run tb4_astar_planner goal_input <x> <y> [yaw_degrees]")
        print("    Exemple: ros2 run tb4_astar_planner goal_input 2.5 1.3 90")
        print("\n  Mode waypoints:")
        print("    ros2 run tb4_astar_planner goal_input --waypoints <fichier>")
        print("    Exemple: ros2 run tb4_astar_planner goal_input -w waypoints.txt")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()