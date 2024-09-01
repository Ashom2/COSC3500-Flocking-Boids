import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
#from p5 import setup, draw, size, background, run



showDir = False
xSize = 128
ySize = 128
markerSize = 1



class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"
    


class Boid:
    def __init__(self, posX, posY, dirX, dirY):
        self.pos = Vector2D(posX, posY)
        self.dir = Vector2D(dirX, dirY)



# Taken from https://stackoverflow.com/questions/23345565/is-it-possible-to-control-matplotlib-marker-orientation
def gen_arrow_head_marker(rot):
    """generate a marker to plot with matplotlib scatter, plot, ...

    https://matplotlib.org/stable/api/markers_api.html#module-matplotlib.markers

    rot=0: positive x direction
    Parameters
    ----------
    rot : float
        rotation in degree
        0 is positive x direction

    Returns
    -------
    arrow_head_marker : Path
        use this path for marker argument of plt.scatter
    scale : float
        multiply a argument of plt.scatter with this factor got get markers
        with the same size independent of their rotation.
        Paths are autoscaled to a box of size -1 <= x, y <= 1 by plt.scatter
    """
    arr = np.array([[.1, .3], [.1, -.3], [1, 0], [.1, .3]])  # arrow shape
    angle = rot / 180 * np.pi
    rot_mat = np.array([
        [np.cos(angle), np.sin(angle)],
        [-np.sin(angle), np.cos(angle)]
        ])
    arr = np.matmul(arr, rot_mat)  # rotates the arrow

    # scale
    x0 = np.amin(arr[:, 0])
    x1 = np.amax(arr[:, 0])
    y0 = np.amin(arr[:, 1])
    y1 = np.amax(arr[:, 1])
    scale = np.amax(np.abs([x0, x1, y0, y1]))
    codes = [mpl.path.Path.MOVETO, mpl.path.Path.LINETO, mpl.path.Path.LINETO, mpl.path.Path.CLOSEPOLY]
    arrow_head_marker = mpl.path.Path(arr, codes)
    return arrow_head_marker, scale



arr = []
with open("data.txt", "r") as file1:
    for line in file1.readlines():
        f_list = [float(i) for i in line.split(" ")]
        arr.append(Boid(f_list[0], f_list[1], f_list[2], f_list[3]))

    # TODO this is probably the slowest way of doing this dear god
    x, y = [], []
    for v in arr:
        x.append(v.pos.x)
        y.append(v.pos.y)

        if (showDir):
            # It is not possible to pass a list of markers to plot
            # plt.arrow(v.pos.x, v.pos.y, v.dir.x, v.dir.y, width = 1, head_starts_at_zero = True)
            
            # plt.plot(v.pos.x, v.pos.y, marker=(3, 0, 180 * math.atan2(v.dir.y, v.dir.x)), markersize=20, linestyle='None')
        
            marker, scale = gen_arrow_head_marker(180 * math.atan2(v.dir.y, v.dir.x))
            plt.scatter(v.pos.x, v.pos.y, marker=marker, s=(markerSize*scale)**2, c="green")
       
        
    
    if (not showDir):
        plt.scatter(x, y, s=markerSize**2)
    
    

    plt.axis([0, xSize, 0, ySize])
    plt.xticks(np.arange(0, xSize + 1, 16))
    plt.yticks(np.arange(0, ySize + 1, 16))
    plt.gca().set_aspect('equal')
    plt.show()