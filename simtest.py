from matplotlib import projections
import numpy as np
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

def fo(x, y):
    return np.sin(np.sqrt(x ** 2 + y ** 2))

f = open("testing_data/test3/my_pos_file3.txt", "r")
coord = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

for line in f:
    x,y,z = line.split(" ")
    add = np.array([[float(x), float(y), float(z)]])
    #print(add)
    coord = np.concatenate((coord, add))
 
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.autoscale(enable=True, axis='both', tight=True)

# Data for a three-dimensional line
# zline = np.linspace(0, 15, 1000)
# xline = np.sin(zline)
# yline = np.cos(zline)
# ax.plot3D(xline, yline, zline, 'gray')

# Data for three-dimensional scattered points
zdata = coord[2]
xdata = coord[0]
ydata = coord[1]
ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')

plt.show()