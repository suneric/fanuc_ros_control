#!/usr/bin/env python3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from industrial_msgs.msg import RobotStatus
from sensor_msgs.msg import JointState

import rospy
import sys
import pandas as pd
import os
sys.path.append('..')
sys.path.append('.')

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


"""
# The robot mode captures the operating mode of the robot.  When in
# manual, remote motion is not possible.

# Estop status: True if robot is e-stopped.  The drives are disabled
# and motion is not possible.  The e-stop condition must be acknowledged
# and cleared before any motion can begin.

# Drive power status: True if drives are powered.  Motion commands will
# automatically enable the drives if required.  Drive power is not requred
# for possible motion

# Motion enabled: True if robot motion is possible.

# Motion status: True if robot is in motion, otherwise false

# Error status: True if there is an error condition on the robot. Motion may
# or may not be affected (see motion_possible)

# Error code: Vendor specific error code (non zero indicates error)
"""


class FanucTrajectortPlayer:
    def __init__(self, trajectory, max_iter=1):
        self.trajectory = trajectory
        self.max_iter = max_iter
        self.count = 0
        self.robot_mode = 0
        self.robot_motion_possible = 0
        self.robot_motion_status = 0
        self.robot_error = 0
        self.traj_pub = rospy.Publisher('joint_path_command', JointTrajectory, queue_size=1)
        self.rs_sub = rospy.Subscriber('/robot_status', RobotStatus, self.robot_status_cb)

    def robot_status_cb(self, data):
        self.robot_mode = data.mode.val
        self.robot_motion_possible = data.motion_possible.val
        self.robot_error = data.in_error.val
        self.robot_motion_status = data.in_motion.val
        self.robot_stop = data.e_stopped.val

    def is_robot_ready(self):
        print("mode: {}, possible: {}, status: {}, error: {}".format(self.robot_mode, self.robot_motion_possible, self.robot_motion_status,self.robot_error))
        if self.robot_mode == 2 and self.robot_motion_possible and not self.robot_motion_status and not self.robot_error:
            return True
        else:
            return False

    def initial(self):
        jt = JointTrajectory()
        jt.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        jt.header.stamp = rospy.Time.now()
        # initial
        jpt = JointTrajectoryPoint()
        jpt.positions = [0]*6
        jpt.velocities = [0]*6
        jpt.time_from_start = rospy.Duration(1.0)
        jt.points.append(jpt)
        self.traj_pub.publish(jt)

    def execute_traj(self, trajectory):
        jt = JointTrajectory()
        jt.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        jt.header.stamp = rospy.Time.now()
        for joint in trajectory:
            jpt = JointTrajectoryPoint()
            jpt.positions = joint[2][:6]
            jpt.velocities = joint[2][6:12]
            jpt.accelerations = joint[2][12:]
            jpt.time_from_start = rospy.Duration(joint[1])
            jt.points.append(jpt)
        return jt

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.is_robot_ready():
                if self.count >= self.max_iter:
                    print("max attempts reached.")
                    # self.initial()
                    break;

                jt = self.execute_traj(self.trajectory)
                self.traj_pub.publish(jt)
                self.count += 1
                print("execute trajectory", self.count)

            rate.sleep()

if __name__ == '__main__':
    # read trajectory
    dir = os.path.dirname(os.path.realpath(__file__))
    traj_file = os.path.join(dir,'../data/progOutput.csv')
    trajectory = predefined_trajectory(traj_file)
    print("Length of trajectory: {}".format(len(trajectory)))

    rospy.init_node('fanuc_traj', anonymous=True)
    robot = FanucTrajectortPlayer(trajectory, max_iter=2)
    try:
        robot.run()
    except rospy.ROSInterruptException:
        pass
