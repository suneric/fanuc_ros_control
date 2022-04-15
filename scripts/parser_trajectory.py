#!/usr/bin/env python3

import pandas as pd
import numpy as np


def predefined_trajectory(file):
    df = pd.read_csv(file)
    joint_names = ["J1"," J2"," J3"," J4"," J5"," J6"]
    joints = df[joint_names]*(3.14/180.0)
    return joints

def validate_joints(joints):
    jt1 = joints["J1"]-0.5
    jt2 = joints[" J2"]-0.75
    jt3 = joints[" J3"]-1.0
    jt4 = joints[" J4"]
    jt5 = joints[" J5"]
    jt6 = joints[" J6"]
    # meanJT = np.mean(jt)
    # minJT = np.min(jt)
    # maxJT = np.max(jt)
    # print(meanJT,minJT,maxJT)
    jts = []
    for i in range(len(jt1)):
        jts.append([jt1[i],jt2[i],jt3[i],jt4[i],jt5[i],jt6[i]])
    return jts

if __name__ == '__main__':
    jt_list = predefined_trajectory("../data/progOutput.csv")
    jt_list = validate_joints(jt_list)
    print(jt_list)
