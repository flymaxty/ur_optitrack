# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 15:28:52 2019

@author: Ye Tian
"""

import sys
import json
import numpy as np
import math3d
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
    print("Optitrack Distance Test v1.1.0")

    print('Read csv file', sys.argv[1], '...', end='')
    data_csv = pd.read_csv(sys.argv[1])
    print('Done', end='\n')

    xx = []
    yy = []
    zz = []
    register_error = []
    euclidean_distance = []
    angle_distance = []
    set_transform = False
    transform_T = math3d.Transform()

    print('Start analysis...', end='')
    grouped_data = data_csv.set_index(['timestamp'])
    for timestamp, row in grouped_data.groupby(level=0):
        t1 = convert_to_transform(row.iloc[0])
        t2 = convert_to_transform(row.iloc[1])

        xx.append(row.iloc[0]['pos0']*1000.0)
        yy.append(row.iloc[0]['pos1']*1000.0)
        zz.append(row.iloc[0]['pos2']*1000.0)

        euclidean_distance.append(t1.pos.dist(t2.pos)*1000.0)
        angle_distance.append(t1.orient.ang_dist(t2.orient))

        # print(set_transform)
        # print(t1.inverse * t2)
        # print(t1.dist(t2), t1.get_orient().ang_dist(t2.get_orient()))
    print('Done', end='\n')
    
    print('Ploting figure...', end='')
    fig = plt.figure()

    euclidean_distance_error = np.array(euclidean_distance) - 100.0

    euclidean_error_boxplot = fig.add_subplot(2, 2, 1)
    euclidean_error_boxplot.set_title('Euclidean error(mm)')
    euclidean_error_boxplot.boxplot(euclidean_distance_error)

    euclidean_error_hist = fig.add_subplot(2, 2, 2)
    euclidean_error_hist.set_title('Euclidean error(mm)')
    euclidean_error_hist.hist(x=euclidean_distance_error, bins=50)

    euclidean_distance_boxplot = fig.add_subplot(2, 2, 3)
    euclidean_distance_boxplot.set_title('Euclidean Distance(mm)')
    euclidean_distance_boxplot.boxplot(euclidean_distance)

    euclidean_distance_hist = fig.add_subplot(2, 2, 4)
    euclidean_distance_hist.set_title('Euclidean Distance(mm)')
    euclidean_distance_hist.hist(x=euclidean_distance, bins=50)

    # point_sub = fig.add_subplot(1, 3, 1, projection='3d')
    # d = point_sub.scatter(xx, yy, zz, c=euclidean_distance_error, marker='.')
    # point_sub.set_title('Point Cloud')
    # point_sub.set_xlabel('X')
    # point_sub.set_ylabel('Y')
    # point_sub.set_zlabel('Z')
    # # point_sub.set_xlim([-600, 600])
    # # point_sub.set_ylim([-100, 100])
    # # point_sub.set_zlim([-600, 600])
    # plt.colorbar(d, ax = point_sub)

    print('Done', end='\n')
    plt.show()
    # print(read_object_data("object_data.json", "cal_design_big", "cal_design_small"))
