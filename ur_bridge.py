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
    arm.close()
    mqtt_client.disconnect()

def on_mqtt_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("iqr/cs-ras/topic/arm_request")

def on_mqtt_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    if(msg.topic == "iqr/cs-ras/topic/arm_request"):
        data_json = json.loads(msg.payload)
        if(data_json["type"] == "movel_tool"):
            data_list = data_json["matrix"]
            matrix = math3d.Transform(np.array(data_list).reshape(4,4))
            print("movel_tool, before\n", arm.get_pose(), "after\n", matrix)
            arm.movel_tool(matrix, acc=0.4, vel=0.4)
        else:
            trace("Unkown data")
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
    mqtt_client.loop_forever()

    print('Done')