#!/usr/bin/env python
import math
import numpy
import matplotlib.pyplot as plt

a = 1
b = 2
A = 3
B = 3
numPoints = 200
buff      = 10

def waypoint_gen():
    t = []
    x = []
    y = []
    for i in range(numPoints+buff):
        t.append(i*(2.0*math.pi)/(numPoints))
    for i in range(numPoints+buff):
        x.append(A*math.cos(a*t[i]))
        y.append(B*math.sin(b*t[i]))
    # for i in range(numPoints+buff):
    #     print(round(t[i],3),(round(x[i],3),round(y[i],3)))
    plt.plot(x,y)
    plt.show()

waypoint_gen()
