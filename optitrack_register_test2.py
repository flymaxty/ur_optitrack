# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 15:28:52 2019

@author: Ye Tian
"""

import sys
import json
import math
import math3d
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use('TkAgg') 

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def convert_to_transform(data):
    quaternion = math3d.UnitQuaternion(data['rot3'], data['rot0'], data['rot1'], data['rot2'])
    transform = math3d.Transform()
    transform.set_pos((data['pos0'], data['pos1'], data['pos2']))
    transform.set_orient(quaternion.orientation)
    return transform
    
if __name__ == '__main__':
    print("Optitrack Register Test v1.1.0")

    print('Read csv file', sys.argv[1], '...', end='')
    data_csv = pd.read_csv(sys.argv[1])
    print('Done', end='\n')

    xx = []
    yy = []
    zz = []
    euclidean_error = []
    angle_error = []

    transform_T = math3d.Transform()

    print('Start analysis...', end='')
    grouped_data = data_csv.set_index(['timestamp'])

    # collect frames to get register transform
    register_count = 0
    register_t1_list = []
    register_t2_list = []
    for timestamp, row in grouped_data.groupby(level=0):
        t1 = convert_to_transform(row.iloc[0])
        t2 = convert_to_transform(row.iloc[1])

        if register_count >= 100:
            print("get transform matrix")
            t1_average = math3d.Transform(np.array(register_t1_list).mean(axis=0).reshape(4,4))
            t2_average = math3d.Transform(np.array(register_t2_list).mean(axis=0).reshape(4,4))
            transform_T = t1_average.inverse * t2_average
            break
        else:
            register_count += 1
            register_t1_list.append(t1.array.flatten().tolist())
            register_t2_list.append(t2.array.flatten().tolist())

    for timestamp, row in grouped_data.groupby(level=0):
        t1 = convert_to_transform(row.iloc[0])
        t2 = convert_to_transform(row.iloc[1])

        xx.append(row.iloc[0]['pos0']*1000.0)
        yy.append(row.iloc[0]['pos1']*1000.0)
        zz.append(row.iloc[0]['pos2']*1000.0)

        t2_calculated = t1 * transform_T
        euclidean_error.append(t2_calculated.pos.dist(t2.pos)*1000.0)
        angle_error.append(t2_calculated.orient.ang_dist(t2.orient))

        # print(set_transform)
        # print(t1.inverse * t2)
        # print(t1.dist(t2), t1.get_orient().ang_dist(t2.get_orient()))
    print('Done', end='\n')

    print("Euclidean mean error: ", np.array(euclidean_error).mean())
    print("Euclidean std: ", np.std(np.array(euclidean_error), ddof=1))
    print("Angle mean error:", np.array(angle_error).mean())
    print("Angle std:", np.std(angle_error, ddof=1))
    
    print('Ploting figure...', end='')
    fig = plt.figure()

    point_sub = fig.add_subplot(2, 3, 1, projection='3d')
    d = point_sub.scatter(xx, yy, zz, c=euclidean_error, marker='.')
    point_sub.set_title('Point Cloud(Euclidean error)')
    point_sub.set_xlabel('X')
    point_sub.set_ylabel('Y')
    point_sub.set_zlabel('Z')
    # point_sub.set_xlim([-600, 600])
    # point_sub.set_ylim([-100, 100])
    # point_sub.set_zlim([-600, 600])
    plt.colorbar(d, ax = point_sub)

    euclidean_error_boxplot = fig.add_subplot(2, 3, 2)
    euclidean_error_boxplot.set_title('Euclidean error(mm)')
    euclidean_error_boxplot.boxplot(euclidean_error)

    euclidean_error_hist = fig.add_subplot(2, 3, 3)
    euclidean_error_hist.set_title('Euclidean error(mm)')
    euclidean_error_hist.hist(x=euclidean_error, bins=50)

    point_sub2 = fig.add_subplot(2, 3, 4, projection='3d')
    d = point_sub2.scatter(xx, yy, zz, c=angle_error, marker='.')
    point_sub2.set_title('Point Cloud(Angle error)')
    point_sub2.set_xlabel('X')
    point_sub2.set_ylabel('Y')
    point_sub2.set_zlabel('Z')
    # point_sub.set_xlim([-600, 600])
    # point_sub.set_ylim([-100, 100])
    # point_sub.set_zlim([-600, 600])
    plt.colorbar(d, ax = point_sub2)

    euclidean_distance_boxplot = fig.add_subplot(2, 3, 5)
    euclidean_distance_boxplot.set_title('Angle error(rad)')
    euclidean_distance_boxplot.boxplot(angle_error)

    euclidean_distance_hist = fig.add_subplot(2, 3, 6)
    euclidean_distance_hist.set_title('Angle error(rad)')
    euclidean_distance_hist.hist(x=angle_error, bins=50)

    print('Done', end='\n')
    plt.show()