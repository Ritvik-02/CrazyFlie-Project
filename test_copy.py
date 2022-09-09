# Same as v5, now with yaw pitch roll from both the stabilizer (gyro) and our own lighthouse angles 
# graphed and orientation matrix also graphed. Notes: The yaw and pitch measurements match up pretty well, 
# but our roll measurements seem to have some trouble when rolling too far away from the 
# lighthouse (facing the opposite direction), which is expected.
from audioop import avg
from cProfile import label
from curses import raw
import logging
import math
import time
import numpy as np
from threading import Event

from cflib.crazyflie.mem import LighthouseMemHelper
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import mpl_toolkits.mplot3d
from matplotlib import pyplot as plt

import position_cal as pc
import pos_grapher as pg
import kalman_testing as kal
#import estimator_kalman as ek


# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

global bs0_rot_mat
global bs0_origin

global bs1_rot_mat
global bs1_origin

global prev_pos
global curr_pos
global valid_sensors

global sensor_timestamps
global stabilizer_timestamps
global stab_y
global stab_r
global stab_p

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    stabilizer_timestamps.append(timestamp)
    stab_y.append(data['stabilizer.yaw'])
    stab_r.append(data['stabilizer.roll'])
    stab_p.append(data['stabilizer.pitch'])
    #ypr_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))

# callback for the first 3 sensors
def log_raw_callback1(timestamp, data, logconf):
    # TODO: write the raw angles of the first three sensors into the raw_angles_file
    global prev_pos
    global curr_pos
    global valid_sensors
    currsensor = 0
    sensor_timestamps.append(timestamp)
    angles = []
    for key in data:
        angles.append(data[key])
        if(len(angles) == 4):
            pos = pc.intersectLines(pc.parseData(angles,bs0_rot_mat,bs1_rot_mat),bs0_origin,bs1_origin)
            if not np.array_equal(pos,prev_pos[currsensor]):
                valid_sensors[currsensor] = 1
            else:
                valid_sensors[currsensor] = 1
            curr_pos[currsensor] = pos
            currsensor+=1
            angles = []

# callback for the last sensor
def log_raw_callback(timestamp, data, logconf):
    sensor_num = int(logconf.name[-1])

    raw_angles_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    angles = []
    for key in data:
        angles.append(data[key])
    pos = pc.intersectLines(pc.parseData(angles,bs0_rot_mat,bs1_rot_mat),bs0_origin,bs1_origin)

    global prev_pos
    global curr_pos
    global valid_sensors

    # check to see if the position is valid
    # if the previous is exactly the same as the current position, then the sensor is not valid
    if not np.array_equal(pos,prev_pos[sensor_num]):
        valid_sensors[sensor_num] = 1
    else:
        valid_sensors[sensor_num] = 1 # set this to 0 if you want to ignore the sensor when invalid data is received

    curr_pos[sensor_num] = pos
    
    avg_pos = [0,0,0]
    tot_valid = np.sum(valid_sensors) 
    for i in range(0,len(curr_pos)):
        ar = curr_pos[i]
        if valid_sensors[i]:
            if(tot_valid == 4):
                my_sensor_coords.append(ar)
            avg_pos = np.add(avg_pos,ar)
    
    if(tot_valid == 4):
        avg_pos = np.divide(avg_pos,tot_valid)
        global my_pos_coords
        #print(avg_pos)
        my_pos_coords.append(avg_pos)
        my_pos_file.write('%s %s %s\n' % (avg_pos[0],avg_pos[1],avg_pos[2]))
    valid_sensors = [0,0,0,0]
    prev_pos = curr_pos

def log_state_callback(timestamp, data, logconf):
    state_timestamps.append(timestamp)
    state_y.append(data['stateEstimate.yaw'])
    state_r.append(data['stateEstimate.roll'])
    state_p.append(data['stateEstimate.pitch'])
    ypr_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))

def kalman_fil_calc(timestamp, data, logconf):
    sensor_num = int(logconf.name[-1])

    raw_angles_file.write('[%d][%s]: %s \n' % (timestamp, logconf.name, data))
    angles = []
    for key in data:
        angles.append(data[key])
    pos = pc.intersectLines(pc.parseData(angles,bs0_rot_mat,bs1_rot_mat),bs0_origin,bs1_origin)

    global prev_pos
    global curr_pos
    global valid_sensors

    # check to see if the position is valid
    # if the previous is exactly the same as the current position, then the sensor is not valid
    if not np.array_equal(pos,prev_pos[sensor_num]):
        valid_sensors[sensor_num] = 1
    else:
        valid_sensors[sensor_num] = 1 # set this to 0 if you want to ignore the sensor when invalid data is received

    curr_pos[sensor_num] = pos
    
    avg_pos = [0,0,0]
    tot_valid = np.sum(valid_sensors) 
    for i in range(0,len(curr_pos)):
        ar = curr_pos[i]
        if valid_sensors[i]:
            if(tot_valid == 4):
                my_sensor_coords.append(ar)
            avg_pos = np.add(avg_pos,ar)
    
    if(tot_valid == 4):
        avg_pos = np.divide(avg_pos,tot_valid)
        global my_pos_coords
        #print(avg_pos)
        my_pos_coords.append(avg_pos)
        my_pos_file.write('%s %s %s\n' % (avg_pos[0],avg_pos[1],avg_pos[2]))
    valid_sensors = [0,0,0,0]
    prev_pos = curr_pos

def log_pos_callback(timestamp, data, logconf):
    pos_x = data['lighthouse.x']
    pos_y = data['lighthouse.y']
    pos_z = data['lighthouse.z']
    #pos_x, pos_y, pos_z = kal.log_pos_callback(timestamp, kal.lg_stateEsti, logconf)
    
    global their_pos_coords
    their_pos_coords.append((pos_x,pos_y,pos_z))
    pos_file.write('%s %s %s\n' % (pos_x,pos_y,pos_z))

def simple_log_async(scf, logconfs):
    cf = scf.cf
    for conf in logconfs:
        cf.log.add_config(conf)
    logconfs[0].data_received_cb.add_callback(log_stab_callback)
    logconfs[1].data_received_cb.add_callback(log_pos_callback)
    logconfs[2].data_received_cb.add_callback(log_raw_callback1)
    logconfs[3].data_received_cb.add_callback(log_raw_callback)
    logconfs[4].data_received_cb.add_callback(log_state_callback)
    for conf in logconfs:
        conf.start()
    print("start")
    # time.sleep(2.5) # how long to collect data for
    # for conf in logconfs:
    #     conf.stop()
    print("stop")

class ReadMem:
    def __init__(self, uri):
        self._event = Event()

        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            helper = LighthouseMemHelper(scf.cf)

            helper.read_all_geos(self._geo_read_ready)
            self._event.wait()

            self._event.clear()

            helper.read_all_calibs(self._calib_read_ready)
            self._event.wait()

    def _geo_read_ready(self, geo_data):
        for id, data in geo_data.items():
            #print('---- Geometry for base station', id + 1)
            data.dump()
            if(id == 0):
                global bs0_origin
                bs0_origin = np.array(data.origin)
                global bs0_rot_mat
                bs0_rot_mat = np.array(data.rotation_matrix)
            else:
                global bs1_origin
                bs1_origin = np.array(data.origin)
                global bs1_rot_mat
                bs1_rot_mat = np.array(data.rotation_matrix)
            #print()
        self._event.set()

    def _calib_read_ready(self, calib_data):
        for id, data in calib_data.items():
            #print('---- Calibration data for base station', id + 1)
            data.dump()
            #print()
        self._event.set()


if __name__ == '__main__':
    # plotting
    map1 = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_title('Locally Calculated Position')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    my_pos_coords = []
    their_pos_coords = []
    my_sensor_coords = []
    my_pred_coords = []

    stabilizer_timestamps = []
    sensor_timestamps = []
    state_timestamps = []
    stab_y = []
    stab_r = []
    stab_p = []
    state_y = []
    state_r = []
    state_p = []

    #raw_angles file
    raw_angles_file = open("raw_angles.txt","a")
    raw_angles_file.truncate(0)

    #my position file
    my_pos_file = open("my_pos_file.txt","a")
    my_pos_file.truncate(0)

    #yaw-pitch-row file
    ypr_file = open("yaw_pitch_row.txt","a")
    ypr_file.truncate(0)

    #my ypr file
    my_ypr_file = open("my_ypr_file.txt","a")
    my_ypr_file.truncate(0)

    my_pred_file = open("my_pred_file.txt","a")
    my_pred_file.truncate(0)

    #position file
    pos_file = open("position.txt","a")
    pos_file.truncate(0)

    prev_pos = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
    curr_pos = [[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
    valid_sensors = [0,0,0,0]

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # ground truth
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    lg_pos = LogConfig(name='position', period_in_ms=10)
    lg_pos.add_variable('lighthouse.x', 'float')
    lg_pos.add_variable('lighthouse.y', 'float')
    lg_pos.add_variable('lighthouse.z', 'float')

    # calibrated lh1 angles
    lg_lh_sen0 = LogConfig(name='lighthouse0', period_in_ms=10)
    lg_lh_sen0.add_variable('lighthouse.angle0x', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle0y', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle1x', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle1y', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle0x_1', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle0y_1', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle1x_1', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle1y_1', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle0x_2', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle0y_2', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle1x_2', 'FP16')
    lg_lh_sen0.add_variable('lighthouse.angle1y_2', 'FP16')
    
    lg_lh_sen3 = LogConfig(name='lighthouse3', period_in_ms=10)
    lg_lh_sen3.add_variable('lighthouse.angle0x_3', 'float')
    lg_lh_sen3.add_variable('lighthouse.angle0y_3', 'float')
    lg_lh_sen3.add_variable('lighthouse.angle1x_3', 'float')
    lg_lh_sen3.add_variable('lighthouse.angle1y_3', 'float')

    # state estimator for kalman
    lg_stateEsti = LogConfig(name='StateEstimator', period_in_ms=100)
    lg_stateEsti.add_variable('stateEstimate.roll', 'float')
    lg_stateEsti.add_variable('stateEstimate.pitch', 'float')
    lg_stateEsti.add_variable('stateEstimate.yaw', 'float')

    ReadMem(uri)

    # three_sensor_pos = []
    # yaw_from_bs = []
    # pitch_from_bs = []
    # roll_from_bs = []
    # timestamps = []

    my_pos_coords = []

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # set method to cross-beam method to get lighthouse.x,y,z
        cf = scf.cf
        param_name = "lighthouse.method" 
        param_value = 0 # Estimation Method: 0:CrossingBeam, 1:Sweep in EKF (extended Kalman Filter) (default: 1)
        cf.param.set_value(param_name, param_value)
        
        logconfs = [lg_stab, lg_pos, lg_lh_sen0, lg_lh_sen3, lg_stateEsti]
        
        for conf in logconfs:
            cf.log.add_config(conf)
        logconfs[0].data_received_cb.add_callback(log_stab_callback)
        logconfs[1].data_received_cb.add_callback(log_pos_callback)
        logconfs[2].data_received_cb.add_callback(log_raw_callback1)
        logconfs[3].data_received_cb.add_callback(log_raw_callback)
        logconfs[4].data_received_cb.add_callback(log_state_callback)
        print("start")
        for conf in logconfs:
            conf.start()
        time.sleep(2.5)
        # for conf in logconfs:
        #     conf.stop()

    # plt.ion()
    #maybe start loop here
    print("starting loop now")
    #for num in range(4):
        # print(num)
    #map1.canvas.draw()
    #map1.canvas.flush_events()
    
    
    print(len(my_pos_coords))
    # if(len(my_pos_coords) != 0):
    #     x = my_pos_coords[:][0]
    #     y = my_pos_coords[:][1]
    #     z = my_pos_coords[:][2]
    #     ax.scatter3D(x,y,z)
    # plt.pause(0.5)
    # for conf in logconfs:
    #     conf.start()
    count = 0
    for i in range(100000000):
        count += 1
    # for conf in logconfs:
    #     conf.stop()

    print(len(my_pos_coords))
    #maybe end loop here
    print("just ended loop")

    plt.show(block=True)

    raw_angles_file.close()
    my_pos_file.close()
    ypr_file.close()
    my_ypr_file.close()
    my_pred_file.close()
    pos_file.close()