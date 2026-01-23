import math


def yaw_from_quaternion(q):
    """
    Extract yaw angle from a quaternion
    Returns:
        float: yaw angle in radians
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """
    Convert yaw angle to quaternion
    Returns:
        dict: quaternion with keys 'x', 'y', 'z', 'w'
    """
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0)
    }


def wrap_angle(angle):
    """
    Wrap angle to [-pi, pi]
    Returns:
        float: wrapped angle in [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_2d(p1, p2):
    """
    Calculate 2D Euclidean distance between two points
    
    Args:
        p1: tuple (x, y) or object with x, y attributes
        p2: tuple (x, y) or object with x, y attributes
        
    Returns:
        float: distance
    """
    if isinstance(p1, tuple):
        x1, y1 = p1
    else:
        x1, y1 = p1.x, p1.y
        
    if isinstance(p2, tuple):
        x2, y2 = p2
    else:
        x2, y2 = p2.x, p2.y
        
    return math.hypot(x2 - x1, y2 - y1)


def angle_between_points(p1, p2):
    """
    Calculate angle from p1 to p2
    
    Args:
        p1: tuple (x, y) or object with x, y attributes
        p2: tuple (x, y) or object with x, y attributes
        
    Returns:
        float: angle in radians
    """
    if isinstance(p1, tuple):
        x1, y1 = p1
    else:
        x1, y1 = p1.x, p1.y
        
    if isinstance(p2, tuple):
        x2, y2 = p2
    else:
        x2, y2 = p2.x, p2.y
        
    return math.atan2(y2 - y1, x2 - x1)
