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

arm = Robot("192.168.0.153")

def trace(*args):
    print("".join(map(str,args)))

def exit(signum, frame):
    print('UR bridge exit...')
    arm.stop()
    arm.close()
    mqtt_client.disconnect()

def on_mqtt_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("iqr/cs-ras/topic/arm_request")

def on_mqtt_message(client, userdata, msg):
    tcp_pos = math3d.Transform()
    print(msg.topic+" "+str(msg.payload))
    if(msg.topic == "iqr/cs-ras/topic/arm_request"):
        data_json = json.loads(msg.payload)
        data_type = data_json["type"] 
        if(data_type == "movel_tool"):
            data_list = data_json["matrix"]
            data_list[3] = data_list[3] / 1000.0
            data_list[7] = data_list[7] / 1000.0
            data_list[11] = data_list[11] / 1000.0
            matrix = math3d.Transform(np.array(data_list).reshape(4,4))
            # orient = math3d.Orientation()
            # matrix.set_orient(orient)
            accel = data_json["accel"]
            velocity = data_json["velocity"]
            print("data", data_list[3])
            print("movel_tool, before\n", arm.get_pose(), "after\n", matrix)
            arm.movel_tool(matrix, acc=accel, vel=velocity, wait=False)
        elif(data_type == "stop"):
            arm.stop()
    else:
        trace("Unkown topic")

mqtt_client = mqtt.Client("iqr/cs-ras/node/python_ur_bridge")
mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_message = on_mqtt_message

if __name__ == '__main__':
    # set signal
    signal.signal(signal.SIGINT, exit)
    signal.signal(signal.SIGTERM, exit)

    # init mqtt
    mqtt_client.connect('127.0.0.1', 1883)

    # loop forever
    # print("Start listening...")
    # mqtt_client.loop_forever()

    while(True):
        trace("get pose", arm.get_pose())
        mqtt_client.loop()

    print('Done')