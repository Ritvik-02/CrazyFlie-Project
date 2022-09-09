from arena import *
import time
import numpy as np

# setup library
scene = Scene(host="mqtt.arenaxr.org", scene="first", namespace="goradia3")

def main():
    # make a box
    box = Box(object_id="my_box", position=Position(0,0,0), scale=Scale(0.5,0.5,0.5))
    # add the box
    scene.add_object(box)

    f = open("testing_data/test3/position3.txt", "r")
    coord = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
    for line in f:
        x,y,z = line.split(" ")
        add = np.array([[float(x), float(y), float(z)]])
        coord = np.concatenate((coord, add))

    for temp in range(1):
        i = 0
        #print("start")
        while(i < len(coord)):
            #print(i)
            time.sleep(0.05)
            xc = coord[i,0]*10 
            yc = coord[i,1]*10 #0.5 is the size of object
            zc = coord[i,2]*10
            #print(xc, " ", yc, " ", zc)
            scene.update_object(box, position=Position(xc,zc,yc))
            i += 5 #7 is a little smoother/faster

# add and start tasks
scene.run_once(main)
scene.run_tasks()