from shapely.geometry import LineString, Polygon, Point
import numpy as np
from corner_plot import create_corners_3d
from interact import get_obstacle_xy_polygon, line_intersects_box
from calculate_search import calculate_cost

load, vel= 10, 12


def build_evg_graph(obstacles, start, goal):
    """
    参数:
        obstacles: list of tuple，障碍物信息 (x_min, y_min, z_min, x_max, y_max, z_max)
        start: tuple, 起点 (x, y, z)
        goal: tuple, 终点 (x, y, z)

    返回:
        nodes: set of tuple，顶点集合
        edges: dict, 每个节点的邻接边，格式为 {node: [(neighbor, weight), ...]}
    """
    # 初始化
    nodes = {start, goal}
    edges = {start: [], goal: []}

    # Step 1: 定义切割平面
    cut_planes = define_cut_planes(obstacles)

    # Step 2: 生成顶点集合
    for obstacle in obstacles:
        corners = create_corners_3d(*obstacle)
        for z in cut_planes:
            if obstacle[2] <= z <= obstacle[5]:  # 检查切割平面是否与障碍物相交
                cut_nodes = generate_cut_nodes(corners, z)
                nodes.update(cut_nodes)

    # Step 3: 计算可视性边
    edges = compute_visibility_edges(nodes, obstacles, start, goal)

    return nodes, edges


def define_cut_planes(obstacles):
    """
    定义切割平面高度。

    参数:
        obstacles: list of tuple，障碍物信息

    返回:
        cut_planes: list of float, 切割平面高度列表
    """
    z0 = 10
    heights = [obs[5] for obs in obstacles]
    heights.sort()
    
    z_25 = heights[int(0.25 * len(heights))] + 0.01
    z_50 = heights[int(0.50 * len(heights))] + 0.01
    z_75 = heights[int(0.75 * len(heights))] + 0.01
    
    # return [z0, z_25, z_50, z_75]
    return [10, 20, 30, 40, 50]


def generate_cut_nodes(corners, z):
    """
    生成切割平面与障碍物的交点。

    参数:
        expanded_corners: list of tuple，障碍物的 3D 扩展角点
        z: float，切割平面的 z 坐标

    返回:
        cut_nodes: list of tuple，切割平面上的节点集合 (x, y, z)
    """
    return [(x, y, z) for x, y, _ in corners]


def extend_node(node1, node2):
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
    x2_new = x2 + .01 * dx
    y2_new = y2 + .01 * dy
    
    return x2_new, y2_new


def compute_visibility_edges(nodes, obstacles, start, goal):
    """
    计算节点之间的可视性边。

    参数:
        nodes: set of tuple，节点集合
        obstacles: list of tuple，障碍物信息
        start: tuple，起点
        goal: tuple，终点

    返回:
        edges: dict，节点的邻接边集合，格式为 {node: [(neighbor, weight), ...]}
    """
    # 初始化边集
    edges = {node: [] for node in nodes}

    # 提取 cut_nodes，不包含 start 和 goal
    cut_nodes = nodes - {start, goal}

    # 将 cut_nodes 转为列表进行遍历
    cut_node_list = list(cut_nodes)

    # 1. 处理 cut_nodes 间的可视性边
    for i, node1 in enumerate(cut_node_list):
        for j in range(i + 1, len(cut_node_list)):
            node2 = cut_node_list[j]

            # 延长线段：分别对 node1 和 node2 进行延长
            x2_new, y2_new = extend_node(node1, node2)

            # 检查延长后的线段是否与障碍物相交
            line = LineString([node1[:2], (x2_new, y2_new)])
            polygon1 = find_polygon_for_node(node1, obstacles)
            polygon2 = find_polygon_for_node(node2, obstacles)
            polygon2 = polygon2.buffer(-0.0001)

            if polygon1 == polygon2:
                continue

            if line.intersects(polygon2):
                continue

            # 检查3D可视性
            if line_intersects_box(node1, node2, obstacles):
                continue

            # 计算权重并添加边
            weight = calculate_cost(node1, node2, load, vel)
            edges[node1].append((node2, weight))
            edges[node2].append((node1, weight))  # 无向图

    # 2. 处理 start 和 goal 的连接
    for special_node in [start, goal]:
        for node in cut_node_list:
            # 延长线段：分别对 special_node 和 node 进行延长
            x2_new, y2_new = extend_node(special_node, node)

            # 检查延长后的线段是否与障碍物相交
            line = LineString([special_node[:2], (x2_new, y2_new)])
            polygon2 = find_polygon_for_node(node, obstacles)
            polygon2 = polygon2.buffer(-0.0001)

            if line.intersects(polygon2):
                continue

            # 检查3D可视性
            if line_intersects_box(special_node, node, obstacles):
                continue

            # 计算权重并添加边
            weight = calculate_cost(special_node, node, load, vel)
            edges[special_node].append((node, weight))
            edges[node].append((special_node, weight))  # 无向图

    return edges


def find_polygon_for_node(node, obstacles, epsilon=0.0001):
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

