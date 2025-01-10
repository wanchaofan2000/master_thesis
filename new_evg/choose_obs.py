from shapely.geometry import MultiPoint, Polygon
from interact import line_intersects_2d

def choosing_obstacles(start, goal, obstacles):
    """
    二阶段选择障碍物的主函数。
    阶段 1: 找到初始线段与障碍物投影相交的障碍物集合。
    阶段 2: 创建凸包并找到与凸包相交的障碍物集合。

    参数:
        start: tuple, 起点 (x, y, z)
        goal: tuple, 终点 (x, y, z)
        obstacles: list of tuple, 障碍物 (x_min, y_min, z_min, x_max, y_max, z_max, expansion_value)

    返回:
        list of tuple: 最终筛选出的障碍物，格式与输入相同。
    """
    # 阶段 1: 找到与线段相交的障碍物
    obstacles1_2D = line_intersects_2d(start, goal, obstacles)
    
    # 获取障碍物对应的二维投影的原始数据
    obstacles1_original = [
        obs for obs in obstacles
        if get_obstacle_xy_polygon(obs) in obstacles1_2D
    ]
    
    # 阶段 2: 根据第一阶段结果创建凸包并寻找与凸包相交的障碍物
    convex_hull = create_convex_hull(obstacles1_2D)
    obstacles2_original = [
        obs for obs in obstacles
        if convex_hull.intersects(get_obstacle_xy_polygon(obs))
    ]
    
    return obstacles2_original

def get_obstacle_xy_polygon(obstacle):
    """
    根据障碍物的三维数据提取 xy 平面的二维投影多边形。
    """
    x_min, y_min, _, x_max, y_max, _= obstacle
    vertices = [
        (x_min, y_min), 
        (x_min, y_max), 
        (x_max, y_max), 
        (x_max, y_min)
    ]
    return Polygon(vertices)

def create_convex_hull(obstacles_2D):
    """
    根据输入的多边形列表生成一个凸包。
    """
    # 收集所有多边形的顶点
    all_points = []
    for poly in obstacles_2D:
        all_points.extend(poly.exterior.coords)
    
    # 创建一个 MultiPoint 对象并生成凸包
    convex_hull = MultiPoint(all_points).convex_hull
    return convex_hull

def obstacle_expansion(obstacles, expansion):
    """
    对障碍物进行扩展。

    参数:
        obstacles: list of tuple, 障碍物 (x_min, y_min, z_min, x_max, y_max, z_max, expansion_value)
        expansion: float, 扩展值

    返回:
        list of tuple: 扩展后的障碍物，格式与输入相同。
    """
    expanded_obstacles = []
    for obstacle in obstacles:
        x_min, y_min, z_min, x_max, y_max, z_max = obstacle
        expanded_obstacles.append((
            x_min - expansion, y_min - expansion, z_min,
            x_max + expansion, y_max + expansion, z_max + expansion
        ))
    return expanded_obstacles