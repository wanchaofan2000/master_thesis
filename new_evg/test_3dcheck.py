import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def line_intersects_box(p1, p2, obstacles):
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
    """计算长方体顶点"""
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
    """检查线段和长方体在某个轴上的投影是否重叠"""
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
    # 判断线段的最小投影和最大投影是否都严格大于长方体的投影范围
    return not (line_min == box_min or line_max == box_max)

# 示例测试
p1 = [0, 0, 0]
p2 = [5, 5, 5]
obstacles = [
    [3, 4, 5, 6, 0, 0]  # 定义一个长方体的坐标范围
]

result = line_intersects_box(p1, p2, obstacles)
print("是否穿过长方体:", result)  # 期望输出: True


# 绘制三维图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制线段
p1 = np.array(p1)
p2 = np.array(p2)
ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color='b', label="Line Segment")

# 绘制长方体
box_min = np.array(obstacles[0][:3])
box_max = np.array(obstacles[0][3:6])
vertices = get_box_vertices(box_min, box_max)

# 绘制长方体的12条边
edges = [
    [0, 1], [1, 3], [3, 2], [2, 0],
    [4, 5], [5, 7], [7, 6], [6, 4],
    [0, 4], [1, 5], [2, 6], [3, 7]
]

for edge in edges:
    ax.plot([vertices[edge[0]][0], vertices[edge[1]][0]],
            [vertices[edge[0]][1], vertices[edge[1]][1]],
            [vertices[edge[0]][2], vertices[edge[1]][2]], color='r')

# 设置图形属性
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 显示图形
plt.show()
