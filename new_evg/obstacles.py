import random
import math
from corner_plot import plot_obstacles_3d
import matplotlib.pyplot as plt

total_length = 720
ob_length_low, ob_length_high = 30, 100
def distance_between_boxes(box1, box2):
    # Calculate the center distance between two boxes (simplified as the distance between their centers)
    center1 = [(box1[0] + box1[3]) / 2, (box1[1] + box1[4]) / 2, (box1[2] + box1[5]) / 2]
    center2 = [(box2[0] + box2[3]) / 2, (box2[1] + box2[4]) / 2, (box2[2] + box2[5]) / 2]
    
    # Calculate the half sizes
    half_size1 = [(box1[3] - box1[0]) / 2, (box1[4] - box1[1]) / 2, (box1[5] - box1[2]) / 2]
    half_size2 = [(box2[3] - box2[0]) / 2, (box2[4] - box2[1]) / 2, (box2[5] - box2[2]) / 2]
    
    # Calculate the distance between the boxes along each axis
    dist_x = abs(center1[0] - center2[0]) - (half_size1[0] + half_size2[0])
    dist_y = abs(center1[1] - center2[1]) - (half_size1[1] + half_size2[1])
    dist_z = abs(center1[2] - center2[2]) - (half_size1[2] + half_size2[2])
    
    # If the distance along any axis is negative, it means they overlap on that axis
    dist_x = max(dist_x, 0)
    dist_y = max(dist_y, 0)
    dist_z = max(dist_z, 0)
    
    return math.sqrt(dist_x**2 + dist_y**2 + dist_z**2)

def generate_obstacles(num_obstacles, min_distance, expansion_value):
    obstacles = []
    
    while len(obstacles) < num_obstacles:
        x_min = round(random.uniform(0, total_length), 1)
        y_min = round(random.uniform(0, total_length), 1)
        z_min = 0
        x_max = round(x_min + random.uniform(ob_length_low, ob_length_high), 1)
        y_max = round(y_min + random.uniform(ob_length_low, ob_length_high), 1)
        z_max = round(z_min + random.uniform(10, 50), 1)
        
        new_obstacle = (x_min, y_min, z_min, x_max, y_max, z_max)
        
        valid = True
        for obs in obstacles:
            if distance_between_boxes(new_obstacle, obs) < min_distance:
                valid = False
                break
        
        if valid:
            obstacles.append(new_obstacle)
    
    return obstacles

def save_obstacles_to_file(obstacles, filename):
    with open(filename, 'w') as f:
        for obs in obstacles:
            line = ' '.join(f"{coord:.1f}" for coord in obs) + '\n'  # 保留一位小数
            f.write(line)

def load_obstacles_from_file(filename):
    obstacles = []
    with open(filename, 'r') as f:
        for line in f:
            obstacles.append(tuple(map(float, line.strip().split())))
    return obstacles

# 生成或加载障碍物
if __name__ == '__main__':
    filename = 'obstacles40.txt'

    obstacles = generate_obstacles(40, 14, 2)
    save_obstacles_to_file(obstacles, filename)
    print("障碍物已生成并保存到文件。")
    obstacles = load_obstacles_from_file('obstacles40.txt')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-5, total_length*1.2])
    ax.set_ylim([-5, total_length*1.2])
    ax.set_zlim([0, 50])

    # 设置轴标签
    ax.set_xlabel('X (m)', fontsize=7)
    ax.set_ylabel('Y (m)', fontsize=7)
    ax.set_zlabel('Z (m)', fontsize=7)
    plot_obstacles_3d(ax, obstacles)
    plt.show()

