'''
    本文件实现了判断线段与长方体是否相交的函数line_intersects_box
'''
import numpy as np
from shapely.geometry import LineString, Polygon

def get_obstacle_xy_polygon(obstacle):
    x_min, y_min, _, x_max, y_max, _ = obstacle
    vertices = [
        (x_min, y_min), 
        (x_min, y_max), 
        (x_max, y_min), 
        (x_max, y_max)
    ]
    return Polygon(vertices)

def line_intersects_2d(start, goal, obstacles):
    # 计算线段的 xy 投影
    line = LineString([(start[0], start[1]), (goal[0], goal[1])])
    intersecting_projections = []
    
    for obstacle in obstacles:
        # 获取障碍物的 xy 投影多边形
        obstacle_polygon = get_obstacle_xy_polygon(obstacle)
        
        # 判断线段是否与多边形相交
        if line.intersects(obstacle_polygon):
            intersecting_projections.append(obstacle_polygon)
    
    return intersecting_projections


def line_intersects_box(p1, p2, obstacles):
    """
    判断线段是否与任何障碍物（长方体）相交。
    
    参数:
        p1: tuple，线段起点 (x1, y1, z1)
        p2: tuple，线段终点 (x2, y2, z2)
        obstacles: list of tuples，每个障碍物由 (xmin, ymin, zmin, xmax, ymax, zmax) 表示
        
    返回:
        bool: 如果线段与任一障碍物相交，返回 True；否则返回 False。
    """
    P1, P2 = np.array(p1), np.array(p2)
    d = P2 - P1  # 线段方向向量

    box_normals = [
        np.array([1, 0, 0]),
        np.array([0, 1, 0]),
        np.array([0, 0, 1])
    ]

    # 构造分离轴集合
    axes = [d] + box_normals + [np.cross(d, n) for n in box_normals if np.linalg.norm(np.cross(d, n)) > 1e-6]

    for obstacle in obstacles:
        box_min = np.array(obstacle[:3]) + 1e-6  # 收缩边界，避免与边界相交
        box_max = np.array(obstacle[3:6]) - 1e-6
        box_vertices = get_box_vertices(box_min, box_max)

        # 逐轴检测投影是否重叠
        if all(projection_overlaps(P1, P2, box_vertices, axis) for axis in axes):
            # 如果所有分离轴的投影重叠，表示穿过了长方体
            return True

    return False

def get_box_vertices(box_min, box_max):
    """
    计算长方体的8个顶点。
    """
    box_vertices = [
        [box_min[0], box_min[1], box_min[2]],
        [box_min[0], box_min[1], box_max[2]],
        [box_min[0], box_max[1], box_min[2]],
        [box_min[0], box_max[1], box_max[2]],
        [box_max[0], box_min[1], box_min[2]],
        [box_max[0], box_min[1], box_max[2]],
        [box_max[0], box_max[1], box_min[2]],
        [box_max[0], box_max[1], box_max[2]]
    ]
    return np.array(box_vertices)

def projection_overlaps(P1, P2, box_vertices, axis):
    """
    检查线段和长方体在某个轴上的投影是否重叠。
    """
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-6:  # 避免零向量
        return True

    axis = axis / axis_norm

    # 计算线段的投影范围
    line_proj = [np.dot(P1, axis), np.dot(P2, axis)]
    # 计算长方体的投影范围
    box_proj = [np.dot(vertex, axis) for vertex in box_vertices]

    line_min, line_max = min(line_proj), max(line_proj)
    box_min, box_max = min(box_proj), max(box_proj)

    # 检查投影是否重叠，并确保线段不止交于边界点
    if line_max < box_min or box_max < line_min:
        return False  # 投影不重叠，返回 False

    # 如果投影重叠，还需确保线段与长方体的交点不是单点相交
    return not (line_min == box_min or line_max == box_max)

# 示例输入
if __name__ == '__main__':
    start,goal = [1, 1, 1], [40, 40, 40]
    obstacles = [
    (55, 20, 0, 70, 40, 40),
    (14, 14, 0, 34, 34, 40),
    ]
    intersecting = line_intersects_box(start, goal, obstacles)
    print("线段是否与任意一个长方体相交:", intersecting)


