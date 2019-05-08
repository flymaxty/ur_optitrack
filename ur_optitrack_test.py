# -*- coding: utf-8 -*-

"""
Created on Mon Jun 11 17:16:28 2018

@author: Ye Tian
"""

import sys
import time
import math
import random
import signal
import math3d
import threading
from datetime import datetime
import numpy as np
import pandas as pd
from urx import Robot
from NatNetClient import NatNetClient

test_count = 10
rigid_body_dict = {}
mutex = threading.Lock()

start_pos = math3d.Transform()

def trace(*args):
    print("".join(map(str,args)))

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    trace( "Received frame", frameNumber)

def receiveRigidBodyPackageFrame( frameNumber, rigidBodyList ):

    mutex.acquire()
    rigid_body_dict.clear()
    for rigidBody in rigidBodyList:
        if rigidBody['tracking'] == 1:
            trans_matrix = math3d.Transform()
            quaternion = math3d.UnitQuaternion(rigidBody['rot'][3], rigidBody['rot'][0], rigidBody['rot'][1], rigidBody['rot'][2])
            trans_matrix.set_pos(rigidBody['pos'])
            trans_matrix.set_orient(quaternion.orientation)
            rigid_body_dict[rigidBody['name']] = trans_matrix
    mutex.release()

def signal_handler(signum, frame):
    print("signal hit")
    streamingClient.stop()
    sys.exit()

def  reset_transform():
    return math3d.Transform()

def reset_arm():
    global start_pos
    if start_pos.pos.x != 0.0:
        print('reset arm')
        arm.set_pose(start_pos)
        time.sleep(2)

def get_current_pose():
    global rigid_body_dict

    tcp_pose = arm.get_pose()

    mutex.acquire()
    while ('cs_rasc_tcp' in rigid_body_dict) == False:
        print("no data")
        time.sleep(0.01)
    opti_pose = rigid_body_dict['cs_rasc_tcp']
    mutex.release()

    return tcp_pose, opti_pose

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    arm = Robot("192.168.1.22")

    streamingClient = NatNetClient()
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyPackageListener = receiveRigidBodyPackageFrame 

    # run optitrack thread
    streamingClient.run()
    time.sleep(2)
    trans = math3d.Transform()

    start_pos = arm.get_pose()

    # x error
    x_error = []
    count = test_count
    while count>0:
        trans = reset_transform()

        tcp1, opti1 = get_current_pose()
        delta = random.uniform(0.002, 0.005)
        trans.pos.x = delta if count%2 else -delta
        arm.movex_tool("movel", trans, acc=0.4, vel=0.4)
        time.sleep(2)
        tcp2, opti2 = get_current_pose()
        tcp_error = tcp2.pos.dist(tcp1.pos)
        opti_error = opti2.pos.dist(opti1.pos)

        x_error.append([delta, tcp_error, opti_error])

        print("x", delta)
        print("tcp_error: ", (tcp_error))
        print("opti_error: ", (opti_error))
        print("=======================")

        count = count - 1

    reset_arm()

    # y error
    y_error = []
    count = test_count
    while count>0:
        trans = reset_transform()

        tcp1, opti1 = get_current_pose()
        delta = random.uniform(0.002, 0.005)
        trans.pos.y = delta if count%2 else -delta
        arm.movex_tool("movel", trans, acc=0.4, vel=0.4)
        time.sleep(2)
        tcp2, opti2 = get_current_pose()
        tcp_error = tcp2.pos.dist(tcp1.pos)
        opti_error = opti2.pos.dist(opti1.pos)

        y_error.append([delta, tcp_error, opti_error])

        print("y", delta)
        print("tcp_error: ", (tcp_error))
        print("opti_error: ", (opti_error))
        print("=======================")

        count = count - 1

    reset_arm()

    # z error
    z_error = []
    count = test_count
    while count>0:
        trans = reset_transform()

        tcp1, opti1 = get_current_pose()
        delta = random.uniform(0.002, 0.005)
        trans.pos.z = delta if count%2 else -delta
        arm.movex_tool("movel", trans, acc=0.4, vel=0.4)
        time.sleep(2)
        tcp2, opti2 = get_current_pose()
        tcp_error = tcp2.pos.dist(tcp1.pos)
        opti_error = opti2.pos.dist(opti1.pos)

        z_error.append([delta, tcp_error, opti_error])

        print("z", delta)
        print("tcp_error: ", (tcp_error))
        print("opti_error: ", (opti_error))
        print("=======================")

        count = count - 1

    reset_arm()

    # x angle error
    rx_error = []
    count = test_count
    while count>0:
        trans = reset_transform()

        tcp1, opti1 = get_current_pose()
        delta = random.uniform(0, math.pi/18.0)
        trans.orient.rotate_x(delta if count%2 else -delta)
        arm.movex_tool("movel", trans, acc=0.4, vel=0.4)
        time.sleep(2)
        tcp2, opti2 = get_current_pose()
        tcp_error = tcp2.orient.ang_dist(tcp1.orient)
        opti_error = opti2.orient.ang_dist(opti1.orient)

        rx_error.append([delta, tcp_error, opti_error])

        print("x angle ", math.degrees(delta))
        print("tcp_error: ", math.degrees(tcp_error))
        print("opti_error: ", math.degrees(opti_error))
        print("=======================")

        count = count - 1

    reset_arm()

    # y angle error
    ry_error = []
    count = test_count
    while count>0:
        trans = reset_transform()

        tcp1, opti1 = get_current_pose()
        delta = random.uniform(0, math.pi/18.0)
        trans.orient.rotate_y(delta if count%2 else -delta)
        arm.movex_tool("movel", trans, acc=0.4, vel=0.4)
        time.sleep(2)
        tcp2, opti2 = get_current_pose()
        tcp_error = tcp2.orient.ang_dist(tcp1.orient)
        opti_error = opti2.orient.ang_dist(opti1.orient)

        ry_error.append([delta, tcp_error, opti_error])

        print("y angle ", math.degrees(delta))
        print("tcp_error: ", math.degrees(tcp_error))
        print("opti_error: ", math.degrees(opti_error))
        print("=======================")

        count = count - 1

    reset_arm()

    # z angle error
    rz_error = []
    count = test_count
    while count>0:
        trans = reset_transform()

        tcp1, opti1 = get_current_pose()
        delta = random.uniform(0, math.pi/18.0)
        trans.orient.rotate_z(delta if count%2 else -delta)
        arm.movex_tool("movel", trans, acc=0.4, vel=0.4)
        time.sleep(2)
        tcp2, opti2 = get_current_pose()
        tcp_error = tcp2.orient.ang_dist(tcp1.orient)
        opti_error = opti2.orient.ang_dist(opti1.orient)

        rz_error.append([delta, tcp_error, opti_error])

        print("z angle ", math.degrees(delta))
        print("tcp_error: ", math.degrees(tcp_error))
        print("opti_error: ", math.degrees(opti_error))
        print("=======================")

        count = count - 1

    reset_arm()

    print("generating csv data...", end='')
    count = 0
    error_data = []
    while count < test_count:
        error_data.append(x_error[count]+y_error[count]+z_error[count]+rx_error[count]+ry_error[count]+rz_error[count])
        count += 1
    csv_data = pd.DataFrame(error_data, columns = ['x_design', 'x_tcp', 'x_opti', 'y_design', 'y_tcp', 'y_opti', 'z_design', 'z_tcp', 'z_opti', 'rx_design', 'rx_tcp', 'rx_opti', 'ry_design', 'ry_tcp', 'ry_opti', 'rz_design', 'rz_tcp', 'rz_opti'])
    file_name = "ur_opti_test_" + datetime.now().strftime("%Y%m%d%H%M%S") + ".csv"
    csv_data.to_csv(file_name, index=False)
    print('done', end='\n')

    streamingClient.stop()
    sys.exit()