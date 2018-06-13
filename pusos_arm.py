# -*- coding: utf-8 -*-

"""
Created on Mon Jun 11 17:16:28 2018

@author: Ye Tian
"""

from NatNetClient import NatNetClient
import numpy as np
from urx import Robot
import math3d

counter = 0
arm_base_T = np.empty([4,4])

def exit(signum, frame):
    print('You choose to stop me.')
    exit()

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    print( "Received frame", frameNumber)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global arm_base_T, counter
    print( "Received frame for rigid body", id , position, rotation)
    if(id == 1):
        trans_matrix = math3d.Transform()
        quaternion = math3d.UnitQuaternion(rotation[0], rotation[1], rotation[2], rotation[3])
        trans_matrix.set_pos(position)
        trans_matrix.set_orient(quaternion.orientation)
        print("trans_matrix", trans_matrix.matrix)
        arm_base_T = trans_matrix.get_inverse().matrix
        print("arm_base_T", arm_base_T)

        # exit(0)
    elif(id == 2):
        while(counter < 30):
            counter += 1
            return
        counter = 0
        tcp_posA = np.array(position)
        tcp_posA = np.append(tcp_posA, 1)
        tcp_posA = tcp_posA.reshape(4,1)
        tcp_pos = arm_base_T * tcp_posA
        print("tcp", tcp_pos)

        current_tcp = arm.get_pose()
        current_tcp.pos.x = tcp_pos[0] * (-1.0)
        current_tcp.pos.y = tcp_pos[1]
        current_tcp.pos.z = tcp_pos[2] * (-1.0)
        arm.set_pose(current_tcp, acc=0.1, vel=0.05)
        pass

arm = Robot("192.168.0.200")

# This will create a new NatNet client
streamingClient = NatNetClient()

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
streamingClient.run()