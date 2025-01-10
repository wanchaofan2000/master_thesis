from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def create_expanded_corners_3d(x1, y1, z1, x2, y2, z2, expansion):
    
    return [
        (x1 - expansion, y1 - expansion, z1),
        (x2 + expansion, y1 - expansion, z1),
        (x1 - expansion, y2 + expansion, z1),
        (x2 + expansion, y2 + expansion, z1),
        (x1 - expansion, y1 - expansion, z2 + expansion),
        (x2 + expansion, y1 - expansion, z2 + expansion),
        (x1 - expansion, y2 + expansion, z2 + expansion),
        (x2 + expansion, y2 + expansion, z2 + expansion)
    ]

def create_corners_3d(x1, y1, z1, x2, y2, z2):
    return [
        (x1 , y1 , z1),
        (x2 , y1 , z1),
        (x1 , y2 , z1),
        (x2 , y2 , z1),
        (x1 , y1 , z2),
        (x2 , y1 , z2),
        (x1 , y2 , z2),
        (x2 , y2 , z2)
    ]

def plot_obstacles_3d(ax, obstacles):
    for (x1, y1, z1, x2, y2, z2) in obstacles:
        corners = create_corners_3d(x1, y1, z1, x2, y2, z2)
        faces = [
            [corners[0], corners[1], corners[5], corners[4]],  # bottom face
            [corners[2], corners[3], corners[7], corners[6]],  # top face
            [corners[0], corners[1], corners[3], corners[2]],  # front face
            [corners[4], corners[5], corners[7], corners[6]],  # back face
            [corners[0], corners[2], corners[6], corners[4]],  # left face
            [corners[1], corners[3], corners[7], corners[5]]   # right face
        ]
        poly3d = Poly3DCollection(faces, facecolors='red', linewidths=1, edgecolors='lightcoral', alpha=.5)
        ax.add_collection3d(poly3d)