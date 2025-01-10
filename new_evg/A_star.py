import numpy as np
import heapq

# 定义速度和能量代价系数
horizontal_speed = 23  # 水平速度
ascent_speed = 6  # 上升速度
descent_speed = 5  # 下降速度
vertical_energy_cost = 4  # 垂直方向能量代价系数
horizontal_energy_cost = 1  # 水平方向能量代价系数

# 启发式函数
def heuristic_3d(a, b, k1=1, k2=1):
    horizontal_distance = np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
    vertical_distance = abs(b[2] - a[2])

    # 计算时间代价
    if b[2] > a[2]:  # 上升
        time_cost = horizontal_distance / horizontal_speed + vertical_distance / ascent_speed
    else:  # 下降
        time_cost = horizontal_distance / horizontal_speed + vertical_distance / descent_speed

    # 计算能量代价
    energy_cost = vertical_distance * vertical_energy_cost + horizontal_distance * horizontal_energy_cost

    # 计算加权总代价
    total_cost = k1 * time_cost + k2 * energy_cost

    return total_cost

# A* 搜索算法
def a_star_search_3d(start, goal, nodes, edges, k1=1, k2=1):
    open_heap = []
    heapq.heappush(open_heap, (heuristic_3d(start, goal, k1, k2), start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor, cost in edges[current]:
            new_cost = cost_so_far[current] + cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic_3d(neighbor, goal, k1, k2)
                heapq.heappush(open_heap, (priority, neighbor))
                came_from[neighbor] = current

    return None


def calculate_path_costs(path):
    total_time_cost = 0
    total_energy_cost = 0
    for i in range(1, len(path)):
        current = path[i - 1]
        next = path[i]
        horizontal_distance = np.sqrt((next[0] - current[0])**2 + (next[1] - current[1])**2)
        vertical_distance = abs(next[2] - current[2])

        if next[2] > current[2]:  # 上升
            total_time_cost += horizontal_distance / horizontal_speed + vertical_distance / ascent_speed
        else:  # 下降
            total_time_cost += horizontal_distance / horizontal_speed + vertical_distance / descent_speed

        total_energy_cost += vertical_distance * vertical_energy_cost + horizontal_distance * horizontal_energy_cost
    
    return total_time_cost, total_energy_cost