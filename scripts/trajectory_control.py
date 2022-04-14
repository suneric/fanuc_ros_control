from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
import sys


def traj(time):
    jt = JointTrajectory()
    jt.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    jt.header.stamp = rospy.Time.now()

    joints_list = [[0,0,0,0,0,0],
                   [0,0.1,0,0,0,0],
                   [-0.1,0.1,0,0,0,0],
                   [0.1,0.1,0,0,0,0],
                   [0,0,0,0,0,0]]
    for joints in joints_list:
        jpt = JointTrajectoryPoint()
        jpt.positions = joints
        jpt.velocities = [0]*6
        jpt.time_from_start = rospy.Duration(time)
        jt.points.append(jpt)

    return jt

def traj_pub():
    pub = rospy.Publisher('joint_path_command', JointTrajectory, queue_size=1)
    rospy.init_node('fanuc_traj', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        key_input = input("plear enter space to send trajectory:\n")
        if key_input == '':
            t = traj(20)
            pub.publish(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        traj_pub()
    except rospy.ROSInterruptException:
        pass
