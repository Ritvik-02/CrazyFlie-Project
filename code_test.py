# import matplotlib.pyplot as plt 
# import numpy as np

# plt.ion() 
# plt.title(" Sample interactive plot") 

# def get_points():
#     coords = []
#     for i in range(10):
#         add = np.random.randint(0,10,2)
#         coords.append(add)
#     return coords

# for i in range(10):
#     coords = get_points()
#     for j in range(len(coords)):
#         plt.scatter(coords[j][0], coords[j][1])
#     plt.pause(0.5)
# plt.show(block=True)

import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d


map1 = plt.figure()
ax = plt.axes(projection='3d')
ax.set_title('Locally Calculated Position')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.ion()

for i in range(10):
    xyz=np.array(np.random.random((10,3)))
    x=xyz[:,0]
    y=xyz[:,1]
    z=xyz[:,2]

    ax.scatter(x,y,z)
    plt.pause(1)

plt.show(block=True)