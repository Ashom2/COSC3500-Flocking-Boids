import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class Vector2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "(" + str(self.x) + ", " + str(self.y) + ")"

arr = []
with open("data.txt", "r") as file1:
    for line in file1.readlines():
        f_list = [float(i) for i in line.split(" ")]
        arr.append(Vector2D(f_list[0], f_list[1]))

    # for x in arr:
    #     print(x)

    x, y = [], []
    for v in arr:
        x.append(v.x)
        y.append(v.y)
        plt.arrow(v.x, v.y, 2, 2, width = 1, head_starts_at_zero = True)

    #plt.plot(x, y, "o")
    plt.show()