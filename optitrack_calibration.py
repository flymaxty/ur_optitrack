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

def read_object_data(object_file_name, object_1, object_2) :
    print(f"object file name: {object_file_name}")
    print(f"transform objects: {object_1}, {object_2}")
    
    #read json file
    with open(object_file_name, 'r') as object_file:
        data = json.load(object_file)

    # get object points data    
    for obj in data['objects']:
        if obj['id'] == object_1:
            object1_data = obj['points']
        elif obj['id'] == object_2:
            object2_data = obj['points']
            
    # get transform between object 1 and 2
    for tf in data['transform']:
        if tf['object_1']==object_1 and tf['object_2']==object_2:
            object_tf = tf['tf']
    
    return object1_data, object2_data, object_tf
    
if __name__ == '__main__':
    print("Optitrack Calibration v1.1.0")

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

        euclidean_distance.append(t1.pos.dist(t2.pos)*1000.0-100.0)
        angle_distance.append(t1.get_orient().ang_dist(t2.get_orient()))

        if set_transform == False:
            print("FFFF", t2.pos.dist(t1.pos), ', ', t1.get_orient().ang_dist(t2.get_orient()))
            transform_T = t1.inverse * t2
            # transform_T = math3d.Transform()
            # transform_T.pos.y = -0.1
            set_transform = True
            register_error.append(0)
        else:
            t1_calculated = t1 * transform_T
            t1_error = t1_calculated.pos.dist(t2.pos)*1000.0
            # t2_error = t2_calculated.orient.ang_dist(t2.orient)
            register_error.append(t1_error)
            # print(t2_error)

        # print(set_transform)
        # print(t1.inverse * t2)
        # print(t1.dist(t2), t1.get_orient().ang_dist(t2.get_orient()))
    print('Done', end='\n')
    
    print('Ploting figure...', end='')
    fig = plt.figure()

    point_sub = fig.add_subplot(2, 4, 1, projection='3d')
    d = point_sub.scatter(xx, yy, zz, c=register_error, marker='.')
    point_sub.set_title('Point Cloud')
    point_sub.set_xlabel('X')
    point_sub.set_ylabel('Y')
    point_sub.set_zlabel('Z')
    # point_sub.set_xlim([-600, 600])
    point_sub.set_ylim([-100, 100])
    # point_sub.set_zlim([-600, 600])
    plt.colorbar(d, ax = point_sub)

    euclidean_distance_boxplot = fig.add_subplot(2, 4, 2)
    euclidean_distance_boxplot.set_title('Euclidean Distance(mm)')
    euclidean_distance_boxplot.boxplot(euclidean_distance)

    euclidean_distance_hist = fig.add_subplot(2, 4, 6)
    euclidean_distance_hist.set_title('Euclidean Distance(mm)')
    euclidean_distance_hist.hist(x=euclidean_distance, bins=50)

    angle_distance_boxplot = fig.add_subplot(2, 4, 3)
    angle_distance_boxplot.set_title('Angle Distance(°)')
    angle_distance_boxplot.boxplot(angle_distance)

    angle_distance_hist = fig.add_subplot(2, 4, 7)
    angle_distance_hist.set_title('Angle Distance(°)')
    angle_distance_hist.hist(x=angle_distance, bins=50)

    register_error_hist = fig.add_subplot(2, 4, 5)
    register_error_hist.set_title('Register Error Distance(mm)')
    register_error_hist.hist(x=register_error, bins=50)

    relationship_1_hist = fig.add_subplot(2, 4, 8)
    e = relationship_1_hist.scatter(angle_distance, register_error, c=register_error, marker='.')
    relationship_1_hist.set_title('Relationship(Angle-RegisterError)')
    relationship_1_hist.set_xlabel('Angle Distance(°)')
    relationship_1_hist.set_ylabel('Register Error(mm)')
    plt.colorbar(e, ax = relationship_1_hist)
    relationship_1_hist.set_xlim([0, 0.002])

    relationship_2_hist = fig.add_subplot(2, 4, 4)
    # e = relation_error_hist.scatter(angle_distance, euclidean_distance, c=register_error, marker='.')
    f = relationship_2_hist.scatter(euclidean_distance, register_error, c=register_error, marker='.')
    relationship_2_hist.set_title('Relationship(Distance-RegisterError)')
    relationship_2_hist.set_xlabel('Euclidean Distance(mm)')
    relationship_2_hist.set_ylabel('Register Error(mm)')
    plt.colorbar(f, ax = relationship_2_hist)
    

    print("register_error mean: ", np.mean(register_error), end='\n')
    print('Done', end='\n')
    plt.show()
    # print(read_object_data("object_data.json", "cal_design_big", "cal_design_small"))
