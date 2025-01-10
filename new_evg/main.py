import matplotlib.pyplot as plt
from obstacles import load_obstacles_from_file
from choose_obs import choosing_obstacles, obstacle_expansion
from corner_plot import plot_obstacles_3d
from build_graph import build_evg_graph
from calculate_search import a_star_search_3d


obstacles = load_obstacles_from_file('obstacles40.txt')

start = (0, 0, 2)
goal = (780, 780, 20)
expansion = 2

obstacles_final = choosing_obstacles(start, goal, obstacles)
obstacles_final = obstacle_expansion(obstacles_final, expansion)

nodes, edges = build_evg_graph(obstacles_final, start, goal)
path = a_star_search_3d(start, goal, nodes, edges)
print(path)

print("顶点数量:", len(nodes))
print("边数量:", sum(len(v) for v in edges.values()))


def set_axes_limits(ax, obstacles):
    """
    设置3D图形的坐标轴范围，以适应障碍物的实际尺寸。
    """
    z_min = min([obs[2] for obs in obstacles])
    z_max = max([obs[5] for obs in obstacles])

    # 设置坐标轴范围
    ax.set_xlim(start[0]- 50, goal[0] + 50)
    ax.set_ylim(start[1] - 50, goal[1] + 50)
    ax.set_zlim(z_min, z_max + 20)

def plot_edges_3d(ax, edges, color='blue', alpha=0.5, linewidth=0.5):
    """
    在3D图中绘制可视性图的边。

    参数:
        ax: matplotlib 3D axes
        edges: dict, 可视性图的边，格式为 {node: [(neighbor, weight), ...]}
        color: str, 线条颜色
        alpha: float, 线条透明度
        linewidth: float, 线条宽度
    """
    for node, neighbors in edges.items():
        for neighbor, _ in neighbors:
            # 提取起点和终点
            x_coords = [node[0], neighbor[0]]
            y_coords = [node[1], neighbor[1]]
            z_coords = [node[2], neighbor[2]]

            # 绘制连线
            ax.plot(x_coords, y_coords, z_coords, color=color, alpha=alpha, linewidth=linewidth)


def plot_path_3d(ax, path, color='red', linewidth=2, label='Optimal Path'):
    """
    在3D图中绘制A*搜索的最优路径。

    参数:
        ax: matplotlib 3D axes
        path: list of tuple，路径节点
        color: str, 路径颜色
        linewidth: float, 路径线宽
        label: str, 图例标签
    """
    if len(path) > 1:  # 如果路径节点数大于1
        x_coords, y_coords, z_coords = zip(*path)  # 提取路径的 x, y, z 坐标
        ax.plot(x_coords, y_coords, z_coords, color=color, linewidth=linewidth, label=label)




# 绘制筛选后的障碍物及可视性边
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 使用现有的 plot_obstacles_3d 函数绘制障碍物
plot_obstacles_3d(ax, obstacles_final)

# 绘制可视性图的边
plot_edges_3d(ax, edges, color='blue', alpha=0.7, linewidth=0.7)

plot_path_3d(ax, path, color='red', linewidth=2, label='Optimal Path')

# 设置坐标轴范围
set_axes_limits(ax, obstacles_final)

# 设置坐标轴标签
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# 显示图形
plt.show()