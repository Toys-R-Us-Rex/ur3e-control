import numpy as np

def draw_cercle(center, radius = 0.03, n_step = 32):

    path_offsets = [
        (
            radius * np.cos(2 * np.pi * i / n_step),    # x
            radius * np.sin(2 * np.pi * i / n_step),    # y
            0                                           # z
        ) for i in range(n_step + 1)
    ]

    path = []
    xc,yc,zc = center
    for x,y,z in path_offsets:
        path.append([x+xc,y+yc,z+zc])
    
    return path