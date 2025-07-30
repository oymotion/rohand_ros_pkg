#!/usr/bin/env python3

import threading
import time
import sys
import tty
import termios
from select import select

import rospy
from sensor_msgs.msg import JointState


FRAME_ID_PREFIX = 'rohand_'

PUB_RATE = 10  # Hz

STEPS = 1  # Moving steps for full range


class ROHandTeleopNode:
    def __init__(self):
        rospy.init_node('rohand_teleop_node')  # Initialize ROS node

        self.bus_mutex = threading.Lock()

        rospy.loginfo("node %s init.." % rospy.get_name())

        # Declare parameters
        self.hand_id_ = rospy.get_param('hand_id', 2)
        rospy.loginfo("hand_id: %d" % (self.hand_id_))
        rospy.loginfo("'q' to exit, 'a', 'z', 's', 'x', 'd', 'c', 'f', 'v', 'g', 'b', 'h', 'n' to move fingers")

        # Create publisher for joint states
        self.joint_states_publisher_ = rospy.Publisher('~/target_joint_states', JointState, queue_size=10)

        # Initialize data
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
            joint_states.position = self.angles_
            joint_states.velocity = [65535.0, 65535.0, 65535.0, 65535.0, 65535.0, 65535.0]
            joint_states.effort = []

            # Update header
            joint_states.header.stamp = rospy.Time.now()

            # Publish joint data
            self.joint_states_publisher_.publish(joint_states)

            self.pub_rate.sleep()

    def update_angles(self, angles):
        self.angles_ = angles
        rospy.loginfo("angles: {0}".format(self.angles_))


def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    INCREASE_KEYS = ['a', 's', 'd', 'f', 'g', 'h']
    DECREASE_KEYS = ['z', 'x', 'c', 'v', 'b', 'n']

    MIN_JOINT_ANGLES = [-12.69, 91.00, 89.58, 89.47, 88.05, 0.00]
    MAX_JOINT_ANGLES = [36.76, 180.91, 178.52, 179.04, 177.33, 90.00]

    setting = saveTerminalSettings()

    node = ROHandTeleopNode()  # Create a node instance
    angles = [36.76, 180.91, 178.52, 179.04, 177.33, 0.0]


    rate = rospy.Rate(PUB_RATE)

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

        rate.sleep()

    restoreTerminalSettings(setting)

    rospy.signal_shutdown("shutdown")

if __name__ == '__main__':
    main()

