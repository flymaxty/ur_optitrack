# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 16:09:55 2019

@author: Ye Tian
"""

import sys
import time
import signal
from datetime import datetime

import math3d
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from NatNetClient import NatNetClient

frame_rate = 120
record_seconds = 45
frame_count = 640 # frame_rate * record_seconds

rigid_list = []

def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    pass # print( "Received frame", frameNumber, timestamp )

def receiveRigidBodyPackageFrame( frameNumber, rigidBodyList ):
    global frame_count, xx, yy, zz, dd, csv_data, rigid_list

    all_tracked = True
    for rigidBody in rigidBodyList:
        if rigidBody['tracking']==0:
            all_tracked = False
            break;

    if all_tracked == False:
        print("not all rigid_body on tracking, skip frame", frameNumber, end='\n')
        return

    # get timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d/%H:%M:%S.%f")

    for rigidBody in rigidBodyList:
        rigid_data = [timestamp, rigidBody['name'], rigidBody['id'], rigidBody['pos'][0], rigidBody['pos'][1], rigidBody['pos'][2], rigidBody['rot'][0], rigidBody['rot'][1], rigidBody['rot'][2], rigidBody['rot'][3]]
        rigid_list.append(rigid_data)
    
    if frame_count > 0:
        frame_count -= 1
        print('Unrecorded frame:', str(frame_count).zfill(10), end='\r')
    elif frame_count == 0:
        print('Unrecorded frame:', str(frame_count).zfill(10), end='\n')

def signal_handler(signum, frame):
    print("signal hit")
    streamingClient.stop()
    sys.exit()

def randrange(n, vmin, vmax):
    '''
    Helper function to make an array of random numbers having shape (n, )
    with each number distributed Uniform(vmin, vmax).
    '''
    return (vmax - vmin)*np.random.rand(n) + vmin

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    streamingClient = NatNetClient()
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyPackageListener = receiveRigidBodyPackageFrame 

    try:
        frame_count = int(sys.argv[1])
    except ValueError:
        print("unaccepted value")
    except IndexError:
        pass

    print('Start recording...')
    print('Frame Count: ', frame_count)
    streamingClient.run()
    
    start_time = datetime.now()

#    while(True):
#        current_time = datetime.now()
#        delta_seconds = current_time - start_time
#        if(delta_seconds.total_seconds() >= record_seconds):
#            break;
#        time.sleep(0.25)
    
    while(True):
        if(frame_count <= 0):
            break;
        time.sleep(0.01)
    
    streamingClient.stop()
    time.sleep(1)
    print('recording end')

    print("============== CSV Data ==============")
    csv_data = pd.DataFrame(rigid_list, columns = ['timestamp', 'rigid_name', 'rigid_id', 'pos0', 'pos1', 'pos2', 'rot0', 'rot1', 'rot2', 'rot3'])
    print(csv_data)

    file_name = datetime.now().strftime("%Y%m%d%H%M%S") + ".csv"
    csv_data.to_csv(file_name, index=False)
