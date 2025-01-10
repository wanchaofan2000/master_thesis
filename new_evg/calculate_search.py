import numpy as np
import heapq

def calculate_cost(node1, node2, load = 0, vel = 12, k1 = 1 , k2 = 1):
    c_air = 0.3
    kappa = 1.15
    mass = 50 + load
    area = 1.59 * 1.9 + 4 * 3.14 * 0.3**2
    w =  5
    distancex = node2[0] - node1[0]
    distancey = node2[1] - node1[1]
    distancez = node2[2] - node1[2]
    distance = np.sqrt(distancex**2 + distancey**2 + distancez**2)
    theta = np.arcsin(distancez / distance) if distance != 0 else 0
    time = distance / vel
    P_climbing = max(mass *9.8 *vel * np.sin(theta), 0)
    D_body = 0.5 * 1.225 * (vel**2) * area * c_air

    # 计算推力 T
    thrust = np.sqrt((mass * 9.8)**2 + D_body**2 + 2 * D_body * P_climbing)
    
    P_air_resistance = 0.5 * 1.225 * area * c_air *(vel**3)
    P_downwash = kappa * w * thrust
    P_uav = P_air_resistance + P_downwash + P_climbing
    energy = P_uav * time
    distance = np.sqrt(distancex**2 + distancey**2 + distancez**2)

    return k1 * energy + k2 * distance


def heuristic_3d(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    dz = abs(a[2] - b[2])

    return np.sqrt(dx**2 + dy**2 + dz**2)


def a_star_search_3d(start, goal, nodes, edges):
    open_heap = []
    heapq.heappush(open_heap, (calculate_cost(start, goal), start))
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
                priority = new_cost + calculate_cost(neighbor, goal)
                heapq.heappush(open_heap, (priority, neighbor))
                came_from[neighbor] = current

    return None


if __name__ == '__main__':
    node1 = (0, 0, 0)
    node2 = (10, 10, 10)
    print("Cost between node1 and node2:", calculate_cost(node1, node2))
    print("Heuristic between node1 and node2:", heuristic_3d(node1, node2))