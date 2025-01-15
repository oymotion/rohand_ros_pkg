#!/usr/bin/env python3
import rospy
import threading
import time
import sys
from select import select
import termios
import tty
from sensor_msgs.msg import JointState

FRAME_ID_PREFIX = 'rohand_'

PUB_RATE = 10 

STEPS = 1  # Moving steps for full range


class ROHandTeleopNode:
    def __init__(self):
        rospy.init_node('rohand_teleop_node')
        self.bus_mutex = threading.Lock()

        rospy.loginfo("node %s init.." % rospy.get_name())

        # 获取参数
        self.hand_id_ = rospy.get_param('~hand_id', 2)
        rospy.loginfo("hand_id: %d" % (self.hand_id_))
        rospy.loginfo("'q' to exit, 'a', 'z', 's', 'x', 'd', 'c', 'f', 'v', 'g', 'b', 'h', 'n' to move fingers")

        # 创建并初始化发布者成员属性pub_joint_states_
        self.joint_states_publisher_ = rospy.Publisher("target_joint_states", JointState, queue_size=10)

        # 初始化数据
        self.angles_ = [36.0, 174.0, 174.0, 174.0, 178.0, 0.0]
        self.pub_rate = rospy.Rate(PUB_RATE)

        self.pub_thread_ = threading.Thread(target=self._thread_pub)
        self.pub_thread_.start()

    def _thread_pub(self):
        while not rospy.is_shutdown():
            joint_states = JointState()

            joint_states.header.stamp = rospy.Time.now()
            joint_states.header.frame_id = FRAME_ID_PREFIX + str(self.hand_id_)
            joint_states.name = ['thumb', 'index', 'middle', 'ring', 'little', 'thumb_rotation']

            # Position
            joint_states.position = self.angles_ 

            # Speed
            joint_states.velocity = [65535.0, 65535.0, 65535.0, 65535.0, 65535.0, 65535.0]

            # TODO: force
            joint_states.effort = []

            # 更新 header
            joint_states.header.stamp = rospy.Time.now()

            # 发布关节数据
            self.joint_states_publisher_.publish(joint_states)

            self.pub_rate.sleep()

    def update_angles(self, angles):
        self.angles_ = angles
        rospy.loginfo("angles: {0}".format(self.angles_))


def get_key(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rospy.init_node('rohand_teleop_node')
    INCREASE_KEYS = ['a', 's', 'd', 'f', 'g', 'h']
    DECREASE_KEYS = ['z', 'x', 'c', 'v', 'b', 'n']
    
    MIN_JOINT_ANGLES = [-12.69, 91.00, 89.58, 89.47, 88.05, 0.00]
    MAX_JOINT_ANGLES = [36.76, 180.91, 178.52, 179.04, 177.33, 90.00]

    setting = saveTerminalSettings()

    node = ROHandTeleopNode()  # 新建一个节点
    angles = [36.76, 180.91, 178.52, 179.04, 177.33, 0.0]

    while not rospy.is_shutdown():
        key = get_key(setting, 1 / PUB_RATE)

        if key == 'q':
            break

        try:
            index = INCREASE_KEYS.index(key)
        except ValueError:
            index = -1 
 
        if index >= 0:
            rospy.loginfo('key {0} pressed'.format(key))
            angles[index] = min(angles[index] + (MAX_JOINT_ANGLES[index] - MIN_JOINT_ANGLES[index]) / STEPS, MAX_JOINT_ANGLES[index])
            node.update_angles(angles)
        else:
            try:
                index = DECREASE_KEYS.index(key)
            except ValueError:
                index = -1 
        
            if index >= 0:
                rospy.loginfo('key {0} pressed'.format(key))
                angles[index] = max(angles[index] - (MAX_JOINT_ANGLES[index] - MIN_JOINT_ANGLES[index]) / STEPS, MIN_JOINT_ANGLES[index])
                node.update_angles(angles)

    rospy.shutdown()  # 关闭rospy

    restoreTerminalSettings(setting)


if __name__ == '__main__':
    main()