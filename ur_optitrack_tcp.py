# -*- coding: utf-8 -*-

"""
Created on Mon Jun 11 17:16:28 2018

@author: Ye Tian
"""

import sys
import time
import signal
import math3d
import numpy as np
from urx import Robot
from NatNetClient import NatNetClient

recv4 = False
global_to_tcp_T = math3d.Transform()
target_to_tcp_T = math3d.Transform()
target_to_object_T = math3d.Transform()

target_to_object_T.pos.y = 0.06
tcp_target_orient = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
target_to_object_T.set_orient(math3d.Orientation(tcp_target_orient))

def trace(*args):
    print("".join(map(str,args)))

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    trace( "Received frame", frameNumber)

def receiveRigidBodyPackageFrame( frameNumber, rigidBodyList ):
    global target_to_object_T, global_to_tcp_T, target_to_tcp_T, recv4
    # trace( "Received frame for rigid body ", frameNumber)

    for rigidBody in rigidBodyList:
        if(rigidBody['name'] == 'ur_facebow_fixed_big_ball'):
            recv4 = True
            trans_matrix = math3d.Transform()
            quaternion = math3d.UnitQuaternion(rigidBody['rot'][3], rigidBody['rot'][0], rigidBody['rot'][1], rigidBody['rot'][2])
            trans_matrix.set_pos(rigidBody['pos'])
            trans_matrix.set_orient(quaternion.orientation)
            global_to_tcp_T = trans_matrix.get_inverse()

            # trace("tcp_to_global_T", trans_matrix.matrix)
            # trace("global_to_tcp_T", global_to_tcp_T.matrix)
        elif(rigidBody['name'] == 'cal_design_facebow_small'):
            if(recv4 == True):
                trans_matrix = math3d.Transform()
                quaternion = math3d.UnitQuaternion(rigidBody['rot'][3], rigidBody['rot'][0], rigidBody['rot'][1], rigidBody['rot'][2])
                trans_matrix.set_pos(rigidBody['pos'])
                trans_matrix.set_orient(quaternion.orientation)
                object_to_global_T = trans_matrix

                # trace("A", global_to_tcp_T)
                # trace("B", global_to_object_T.matrix)
                # trace("C", target_to_object_T.matrix)
                temp_matrix = global_to_tcp_T.matrix * object_to_global_T.matrix * target_to_object_T.matrix

                target_to_tcp_T = math3d.Transform()
                target_to_tcp_T.set_pos(temp_matrix[:3,3].A1)
                # target_to_tcp_T.pos.x = target_to_tcp_T.pos.x * -1.0;
                # target_to_tcp_T.pos.y = 0
                # target_to_tcp_T.pos.z = 0
                orient = math3d.Orientation(temp_matrix[:3,:3].A1)
                target_to_tcp_T.set_orient(orient)

                # trace("object_to_global_T", object_to_global_T.matrix)

def signal_handler(signum, frame):
    print("signal hit")
    streamingClient.stop()
    sys.exit()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    arm = Robot("192.168.1.22")

    streamingClient = NatNetClient()
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyPackageListener = receiveRigidBodyPackageFrame 

    # run optitrack thread
    streamingClient.run()

    while True:
        print("set tcp pose\n", target_to_tcp_T.matrix)
        # b = arm.get_pose()
        # target_to_tcp_T.pos.x = 0
        # target_to_tcp_T.pos.y = 0
        # target_to_tcp_T.pos.z = 0
        arm.movex_tool("movep", target_to_tcp_T, acc=0.2, vel=0.2, wait=False, threshold=0.3)
        # time.sleep(0.1)