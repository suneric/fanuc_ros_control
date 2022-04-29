#!/usr/bin/env python

import pandas as pd
import numpy as np


def predefined_trajectory(file):
    df = pd.read_csv(file)
    # print(df.columns, df.shape[0], df.loc[[0]]["J1"])
    trajectory_list = []
    count = df.shape[0]
    column_names = ["J1", " J2", " J3", " J4", " J5", " J6",
                    " SPEED_J1", " SPEED_J2", " SPEED_J3", " SPEED_J4", " SPEED_J5", " SPEED_J6",
                    " ACCEL_J1", " ACCEL_J2", " ACCEL_J3", " ACCEL_J4", " ACCEL_J5", " ACCEL_J6"]
    record = df[column_names].values
    ids = df[" MOVE_ID"].values
    times = df[" TIME_S"].values
    for i in range(count):
        t = times[i]
        id = int(ids[i])
        trajectory = record[i]*(3.1415926/180)
        # print(id, t, trajectory)
        trajectory_list.append([id, t, trajectory])
    return trajectory_list

if __name__ == '__main__':
    trajectory_list = predefined_trajectory("../data/progOutput.csv")
    print(len(trajectory_list), trajectory_list[-2])
