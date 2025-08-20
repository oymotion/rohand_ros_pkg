#!/usr/bin/env python3

import time
import traceback
import serial
import threading
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
from common.rohand_serial_protocol_v2 import *

FRAME_ID_PREFIX = "rohand_"
PUB_RATE = 30  # Hz


class ROHandSerialNode:
    def __init__(self):
        rospy.init_node("rohand_node")  # Initialize ROS node

        rospy.loginfo("node %s init.." % rospy.get_name())

        # Declare parameters
        self.port_name_ = rospy.get_param('port_name', '/dev/ttyUSB0')
        self.baudrate_ = rospy.get_param('baudrate', 115200)
        self.hand_ids_ = rospy.get_param('hand_ids', [2])
        rospy.loginfo("port: %s, baudrate: %d, hand_ids: %s" % (self.port_name_, self.baudrate_, str(self.hand_ids_)))

        self.position_ = []
        self.velocity_ = []
        self.effort_ = []
        self.data_time_ = time.time() - 1000  # Out of date
        self.status_ = []

        serial_port = serial.Serial(
            port=self.port_name_,
            baudrate=self.baudrate_,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=3,
        )

        if not serial_port.is_open:
            serial_port.open()

        rospy.loginfo(f"serial port '{self.port_name_}' opened: {serial_port.is_open}")

        # Create subscribers and publishers
        self.joint_states_subscriber_ = rospy.Subscriber(
            "~/target_joint_states", JointState, self._joint_states_callback
        )

        self.joint_states_publisher_ = rospy.Publisher(
            "~/current_joint_states", JointState, queue_size=10
        )

        self.finger_states_publihser_ = rospy.Publisher(
            "~/finger_state", UInt8MultiArray, queue_size=10
        )

        self.pub_rate = rospy.Rate(PUB_RATE)

        # Initialize bus context
        rohand_ctx = OHandContext(serial=serial_port, timeout=200)
        self.rohand_protocol = OHandProtocol(rohand_ctx)

        # Check protocol version for each hand
        for i in range(10):
            matched_cnt = 0

            if rospy.is_shutdown():
                rospy.loginfo("Shutting down node...")
                break

            for hand_id in self.hand_ids_:
                try:
                    err, remote_err, major, minor = self.rohand_protocol.get_protocol_version(hand_id)
                except Exception as exc:
                    traceback.print_exc()
                    rospy.logerr(f"exception occurred when calling get_protocol_version(): {exc}")
                    continue

                if err != HAND_RESP_SUCCESS:
                    rospy.logerr(f"get_protocol_version() returned an error: {err}")
                    continue

                if major == PROTOCOL_VERSION_MAJOR:
                    matched_cnt += 1
                else:
                    rospy.logerr(
                        f"major protocol version of rohand '{hand_id}' is '{major}', expected '{PROTOCOL_VERSION_MAJOR}'"
                    )
                    raise Exception("Protocol version NOT matched")

            if matched_cnt == len(self.hand_ids_):
                break

            time.sleep(1.0)

        if matched_cnt != len(self.hand_ids_):
            raise Exception("Get protocol version failed")

        self.thread_ = threading.Thread(target=self._thread_pub)
        self.thread_.start()

    def _joint_states_callback(self, msg):
        rospy.loginfo("I heard: %s" % msg)

        try:
            hand_id = int(msg.header.frame_id.replace(FRAME_ID_PREFIX, ""))
        except ValueError as e:
            hand_id = -1

        rospy.loginfo("hand_id: %d" % hand_id)

        try:
            index = self.hand_ids_.index(hand_id)
        except ValueError as e:
            index = -1

        if index >= 0:
            # Send to OHand and read
            err, remote_err, position, angle, current, force, status = self.rohand_protocol.set_custom(
                hand_id, speed=msg.velocity, angle=msg.position, get_flag=SUB_CMD_GET_ANGLE | SUB_CMD_GET_STATUS
            )

            if err == HAND_RESP_SUCCESS:
                self.position_ = angle if angle is not None else []
                self.velocity_ = []  # TODO: Calculate speed according to position diff and time
                self.effort_ = force if force is not None else []
                self.data_time_ = time.time()
                self.status_ = status

    def _thread_pub(self):
        while not rospy.is_shutdown():
            for hand_id in self.hand_ids_:
                joint_states = JointState()
                finger_states = UInt8MultiArray()

                joint_states.header.stamp = rospy.Time.now()
                joint_states.header.frame_id = FRAME_ID_PREFIX + str(hand_id)
                joint_states.name = ["thumb", "index", "middle", "ring", "little", "thumb_rotation"]

                if time.time() - self.data_time_ > 0.5 / PUB_RATE:
                    err, remote_err, position, angle, current, force, status = self.rohand_protocol.set_custom(
                        hand_id, get_flag=SUB_CMD_GET_ANGLE | SUB_CMD_GET_STATUS
                    )

                    if err != HAND_RESP_SUCCESS:
                        continue

                    joint_states.position = angle if angle is not None else []
                    joint_states.velocity = []  # TODO: Calculate speed according to position diff and time
                    joint_states.effort = force if force is not None else []

                    finger_states.data = status

                else:
                    # Use still fresh data
                    joint_states.position = self.position_
                    joint_states.velocity = self.velocity_
                    joint_states.effort = self.effort_
                    finger_states.data = self.status_

                joint_states.header.stamp = rospy.Time.now()

                # Publish joint data
                self.joint_states_publisher_.publish(joint_states)

                # Publish finger state data
                self.finger_states_publihser_.publish(finger_states)

            self.pub_rate.sleep()


if __name__ == '__main__':
    node = ROHandSerialNode()  # Create a node instance

    rospy.spin()  # Keep the node running, waiting for any exit signal (Ctrl+C)

    rospy.signal_shutdown("shutdown")  # Shut down ROS when done

