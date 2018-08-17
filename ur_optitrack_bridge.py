# -*- coding: utf-8 -*-

"""
Created on Mon Jun 11 17:16:28 2018

@author: Ye Tian
"""

import sys
import json
import signal
import math3d
import numpy as np
from urx import Robot
import paho.mqtt.client as mqtt
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

def on_mqtt_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("iqr/cs-ras/arm_request")

def on_mqtt_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

mqtt_client = mqtt.Client("ur_optitrack_bridge")
mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_message = on_mqtt_message

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    trace( "Received frame", frameNumber)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame( id, position, rotation ):
    global target_to_object_T, global_to_tcp_T, target_to_tcp_T, counter, recv4, mqtt_client
    trace( "Received frame for rigid body", id , position, rotation)
    if(id == 1):
        trans_matrix = math3d.Transform()
        quaternion = math3d.UnitQuaternion(rotation[3], rotation[0], rotation[1], rotation[2])
        trans_matrix.set_pos(position)
        trans_matrix.set_orient(quaternion.orientation)

        json_value = json.dumps({'type': 'tcp', 'matrix' :trans_matrix.array.flatten().tolist()})
        mqtt_client.publish('iqr/cs-ras/optitrack_data', json_value)
    elif(id == 3):
        trans_matrix = math3d.Transform()
        quaternion = math3d.UnitQuaternion(rotation[3], rotation[0], rotation[1], rotation[2])
        trans_matrix.set_pos(position)
        trans_matrix.set_orient(quaternion.orientation)

        json_value = json.dumps({'type': 'pointer', 'matrix' :trans_matrix.array.flatten().tolist()})
        mqtt_client.publish('iqr/cs-ras/optitrack_data', json_value)
    elif(id == 6):
        trans_matrix = math3d.Transform()
        quaternion = math3d.UnitQuaternion(rotation[3], rotation[0], rotation[1], rotation[2])
        trans_matrix.set_pos(position)
        trans_matrix.set_orient(quaternion.orientation)

        json_value = json.dumps({'type': 'fork', 'matrix' :trans_matrix.array.flatten().tolist()})
        mqtt_client.publish('iqr/cs-ras/optitrack_data', json_value)

if __name__ == '__main__':

    # init mqtt
    mqtt_client.connect('127.0.0.1', 1883)

    # init optitrack stream
    streamingClient = NatNetClient()

    # setup callback
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyListener = receiveRigidBodyFrame

    print("Start streaming...")

    # run optitrack thread
    streamingClient.run()