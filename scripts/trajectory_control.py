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

def test_trajectory():
    joints_list = [[0,0,0,0,0,0],
                   [-1,0,-1.0,0,0,0],
                   [0,0.5,0,0,0,0],
                   [0.3,0,0,0,-1.3,0],
                   [0,0,0,0,0,0]
                   ]
    return joints_list

def predefined_trajectory(file):
    df = pd.read_csv(file)
    joint_names = ["J1"," J2"," J3"," J4"," J5"," J6"]
    joints = df[joint_names]*(3.14/180.0)
    return joints

def validate_joints(joints):
    jt1 = joints["J1"]
    jt2 = joints[" J2"]
    jt3 = joints[" J3"]
    jt4 = joints[" J4"]
    jt5 = joints[" J5"]
    jt6 = joints[" J6"]
    jts = []
    for i in range(len(jt1)):
        jts.append([jt1[i],jt2[i],jt3[i],jt4[i],jt5[i],jt6[i]])
    return jts



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
    def __init__(self, joints_list, max_iter=1):
        self.joints_list = joints_list
        self.max_iter = max_iter
        self.count = 0
        self.robot_mode = 0
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
        #print("mode: {}, status: {}, error: {}".format(self.robot_mode,self.robot_motion_status,self.robot_error))
        if self.robot_mode == 2 and not self.robot_motion_status and not self.robot_error:
            return True
        else:
            return False

    def initial(self,time = 20):
        jt = JointTrajectory()
        jt.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        jt.header.stamp = rospy.Time.now()
        # initial
        jpt = JointTrajectoryPoint()
        jpt.positions = [0]*6
        jpt.velocities = [0]*6
        jpt.time_from_start = rospy.Duration(time)
        jt.points.append(jpt)
        self.traj_pub.publish(jt)

    def execute_traj(self, joint_list, time = 20):
        jt = JointTrajectory()
        jt.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
        jt.header.stamp = rospy.Time.now()
        for joints in joint_list:
            jpt = JointTrajectoryPoint()
            jpt.positions = joints
            jpt.velocities = [0]*6
            jpt.time_from_start = rospy.Duration(time)
            jt.points.append(jpt)
        return jt

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.is_robot_ready():
                if self.count >= self.max_iter:
                    print("max attempts reached.")
                    break;

                jt = self.execute_traj(self.joints_list,time=20)
                self.traj_pub.publish(jt)
                self.count += 1
                print("execute trajectory", self.count)

            rate.sleep()

if __name__ == '__main__':
    # read trajectory
    dir = os.path.dirname(os.path.realpath(__file__))
    traj_file = os.path.join(dir,'../data/progOutput.csv')
    jt_list = predefined_trajectory(traj_file)
    jt_list = validate_joints(jt_list)
    print("trajectory length",len(jt_list))

    rospy.init_node('fanuc_traj', anonymous=True)
    robot = FanucTrajectortPlayer(jt_list, max_iter=1)
    try:
        robot.run()
    except rospy.ROSInterruptException:
        pass
