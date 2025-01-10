from shapely.geometry import Polygon, LineString
from interact import get_obstacle_xy_polygon
import numpy as np
from obstacles import load_obstacles_from_file

def find_polygon_for_node(node, obstacles, epsilon=0.01):
    """
    根据节点坐标找到其所属的凸多边形。

    参数:
        node: tuple，节点坐标 (x, y, z)
        obstacles: list of tuple，障碍物信息
        epsilon: 浮动容差，用于坐标匹配

    返回:
        polygon: Polygon，节点所属的凸多边形，如果未找到，返回 None。
    """
    for obstacle in obstacles:
        polygon = get_obstacle_xy_polygon(obstacle)  # 获取障碍物的投影多边形
        
        # 遍历多边形的顶点
        for coord in polygon.exterior.coords:
            # 计算节点与顶点的距离
            distance = np.linalg.norm(np.array(node[:2]) - np.array(coord))
            
            # 如果距离小于 epsilon，认为节点与该顶点重合
            if distance < epsilon:
                return polygon

    return None

def extend_node(node1, node2):
    """
    
    参数:
        node1: tuple, 起点 (x1, y1)
        node2: tuple, 终点 (x2, y2)
    
    返回:
        tuple: 延伸后的 node2 坐标 (x2_new, y2_new)
    """
    # 获取线段的两个端点
    x1, y1 = node1[:2]
    x2, y2 = node2[:2]
    
    # 计算方向向量
    dx, dy = x2 - x1, y2 - y1
    
    # 计算线段的长度
    length = np.sqrt(dx**2 + dy**2)

    if length == 0:
        return x2, y2  # 或者其他处理方式，比如跳过这个节点对
    
    # 将 node2 端点沿着方向向量移动 `extension_length` 单位
    x2_new = x2 + .0001*dx
    y2_new = y2 + .0001*dy

    return x2_new, y2_new

# obstacles = load_obstacles_from_file('obstacles20.txt')
# point = (216.83107940749431, 306.0763208816967,60)
# print(find_polygon_for_node(point, obstacles))

# 创建一个简单的凸多边形（例如正方形）
polygon_coords = [(0, 0), (5, 0), (5, 5), (0, 5), (0, 0)]
polygon = Polygon(polygon_coords)

# 创建几条直线来测试与多边形的关系
# 1. 一条与多边形相交的直线
point1, point2 = (0, 0), (5, 3)
new_point2 = extend_node(point1, point2)
line1 = LineString([point1, point2])
line2 = LineString([point1, new_point2])

