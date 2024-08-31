import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
#from p5 import setup, draw, size, background, run

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

arr = []
with open("data.txt", "r") as file1:
    for line in file1.readlines():
        f_list = [float(i) for i in line.split(" ")]
        #arr.append(Vector2D(f_list[0], f_list[1]))
        arr.append(Boid(f_list[0], f_list[1], f_list[2], f_list[3]))

    # for x in arr:
    #     print(x)

    x, y, dx, dy = [], [], [], []
    for v in arr:
        x.append(v.pos.x)
        y.append(v.pos.y)
        dx.append(v.dir.x)
        dy.append(v.dir.y)

        plt.arrow(v.pos.x, v.pos.y, v.dir.x, v.dir.y, width = 1, head_starts_at_zero = True)

    plt.plot(x, y, "o")
    plt.show()