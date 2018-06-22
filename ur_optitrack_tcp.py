# -*- coding: utf-8 -*-

"""
Created on Mon Jun 11 17:16:28 2018

@author: Ye Tian
"""

import sys
import signal
import math3d
import numpy as np
from urx import Robot
from NatNetClient import NatNetClient

recv4 = False
global_to_tcp_T = math3d.Transform()
target_to_tcp_T = math3d.Transform()
target_to_object_T = math3d.Transform()

target_to_object_T.pos.z = 0.06
tcp_target_orient = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
target_to_object_T.set_orient(math3d.Orientation(tcp_target_orient))

def trace(*args):
    pass # print("".join(map(str,args)))

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    trace( "Received frame", frameNumber)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global target_to_object_T, global_to_tcp_T, target_to_tcp_T, counter, recv4
    trace( "Received frame for rigid body", id , position, rotation)
    if(id == 4):
        recv4 = True
        trans_matrix = math3d.Transform()
        quaternion = math3d.UnitQuaternion(rotation[3], rotation[0], rotation[1], rotation[2])
        trans_matrix.set_pos(position)
        trans_matrix.set_orient(quaternion.orientation)
        global_to_tcp_T = trans_matrix.get_inverse()

        trace("tcp_to_global_T", trans_matrix.matrix)
        trace("global_to_tcp_T", global_to_tcp_T.matrix)
    elif(id == 3):
        if(recv4 == True):
            trans_matrix = math3d.Transform()
            quaternion = math3d.UnitQuaternion(rotation[3], rotation[0], rotation[1], rotation[2])
            trans_matrix.set_pos(position)
            trans_matrix.set_orient(quaternion.orientation)
            object_to_global_T = trans_matrix

            # trace("A", global_to_tcp_T)
            # trace("B", global_to_object_T.matrix)
            # trace("C", target_to_object_T.matrix)
            temp_matrix = global_to_tcp_T.matrix * object_to_global_T.matrix * target_to_object_T.matrix

            target_to_tcp_T = math3d.Transform()
            target_to_tcp_T.set_pos(temp_matrix[:3,3].A1)
            orient = math3d.Orientation(temp_matrix[:3,:3].A1)
            target_to_tcp_T.set_orient(orient)

            trace("object_to_global_T", object_to_global_T.matrix)

if __name__ == '__main__':
    # arm = Robot("192.168.0.133")
    arm = Robot("192.168.0.200")

    # init optitrack stream
    streamingClient = NatNetClient()

    # setup callback
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    # run optitrack thread
    streamingClient.run()

    try:
        while True:
            print("set tcp pose\n", target_to_tcp_T.matrix)
            arm.movel_tool(target_to_tcp_T, acc=0.2, vel=0.2)
    except KeyboardInterrupt:
        streamingClient.stopAll()
        arm.stop()
        arm.close()
        print("-------------------")
        print("program stop")
        sys.exit()