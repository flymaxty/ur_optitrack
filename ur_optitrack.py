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

# counter = 0
global_to_arm_T = np.empty([4,4])

tcp_to_target_T = math3d.Transform()
tcp_to_target_T.pos.x += 0.0001
tcp_target_orient = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
tcp_to_target_T.set_orient(math3d.Orientation(tcp_target_orient))

def trace(*args):
    pass # print("".join(map(str,args)))

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    trace( "Received frame", frameNumber)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global tcp_to_target_T, global_to_arm_T, tcp_to_arm_T, counter
    trace( "Received frame for rigid body", id , position, rotation)
    if(id == 1):
        trans_matrix = math3d.Transform()
        quaternion = math3d.UnitQuaternion(rotation[3], rotation[0], rotation[1], rotation[2])
        trans_matrix.set_pos(position)
        trans_matrix.set_orient(quaternion.orientation)
        global_to_arm_T = trans_matrix.get_inverse()

        trace("arm_to_global_T", trans_matrix.matrix)
        trace("global_to_arm_T", global_to_arm_T.matrix)
    elif(id == 2):
        # frame counter, exec each 10 frame
        # if(counter < 2):
        #     counter += 1
        #     return
        # counter = 0

        trans_matrix = math3d.Transform()
        quaternion = math3d.UnitQuaternion(rotation[3], rotation[0], rotation[1], rotation[2])
        trans_matrix.set_pos(position)
        trans_matrix.set_orient(quaternion.orientation)
        obj_to_global_T = trans_matrix

        temp_matrix = global_to_arm_T.matrix * obj_to_global_T.matrix * tcp_to_target_T.matrix

        tcp_to_arm_T = math3d.Transform()
        tcp_to_arm_T.set_pos(temp_matrix[:3,3].A1)
        orient = math3d.Orientation(temp_matrix[:3,:3].A1)
        tcp_to_arm_T.set_orient(orient)

        trace("obj_to_global_T", obj_to_global_T.matrix)

if __name__ == '__main__':
    # arm = Robot("192.168.0.133")
    arm = Robot("192.168.0.200")
    tcp_to_arm_T = arm.get_pose()

    # init optitrack stream
    streamingClient = NatNetClient()

    # setup callback
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    # run optitrack thread
    streamingClient.run()
    try:
        while True:
            print("set tcp pose\n", tcp_to_arm_T.matrix)
            arm.set_pose(tcp_to_arm_T, acc=0.2, vel=0.2)
    except KeyboardInterrupt:
        streamingClient.stopAll()
        arm.stop()
        arm.close()
        print("-------------------")
        print("program stop")
        sys.exit()