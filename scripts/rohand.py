#!/usr/bin/env python3

import threading
import time
import rospy
from sensor_msgs.msg import JointState

from pymodbus.client import ModbusSerialClient
from pymodbus import ModbusException

from common.roh_registers_v1 import *

FRAME_ID_PREFIX = 'rohand_'


class ROHandNode:
    def __init__(self):
        rospy.init_node('rohand_node')
        self.bus_mutex = threading.Lock()

        rospy.loginfo("node %s init.." % rospy.get_name())

        self.port_name_ = rospy.get_param('~port_name', "/dev/ttyUSB0")
        self.baudrate_ = rospy.get_param('~baudrate', 115200)
        self.hand_ids_ = rospy.get_param('~hand_ids', [2])
        rospy.loginfo("port: %s, baudrate: %d, hand_ids: %s" % (self.port_name_, self.baudrate_, str(self.hand_ids_)))

        # 创建并初始化发布者成员属性pub_joint_states_
        self.joint_states_subscriber_ = rospy.Subscriber("target_joint_states", JointState, self._joint_states_callback, queue_size=10)

        # 创建并初始化发布者成员属性pub_joint_states_
        self.joint_states_publisher_ = rospy.Publisher("current_joint_states", JointState, queue_size=10)

        # 初始化数据
        # self._init_joint_states()
        self.pub_rate = rospy.Rate(30)

        # Initialize modbus
        self.modbus_client_ = ModbusSerialClient(port=self.port_name_, baudrate=self.baudrate_)
        self.modbus_client_.connect()

        for i in range(10):
            matched_cnt = 0

            for hand_id in self.hand_ids_:
                try:
                    rr = self.modbus_client_.read_holding_registers(ROH_PROTOCOL_VERSION, count=1, slave=hand_id)
                except ModbusException as exc:
                    rospy.logerr(f"ERROR: exception in pymodbus, {exc}")
                    continue

                if rr.isError():
                    rospy.logerr(f"ERROR: pymodbus read request, {rr}")
                    continue

                if (rr.registers[0] >> 8) == MODBUS_PROTOCOL_VERSION_MAJOR:
                    matched_cnt += 1
                else:
                    rospy.logerr("ERROR: major protocol version of rohand {0} is {1}, expected {2}".format(hand_id, rr.registers[0] >> 8, MODBUS_PROTOCOL_VERSION_MAJOR))
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
            hand_id = int(msg.header.frame_id.replace(FRAME_ID_PREFIX, ''))
        except ValueError as e:
            hand_id = 0

        rospy.loginfo("hand_id: %d" % hand_id)

        if hand_id in self.hand_ids_:
            # Set speed
            values = []

            for i in range(len(msg.velocity)):
                values.append(int(msg.velocity[i]))

            err_occurred = False
            self.bus_mutex.acquire()

            try:
                wr = self.modbus_client_.write_registers(address=ROH_FINGER_SPEED0, values=values, slave=hand_id)
            except Exception as exc:
                err_occurred = True
                rospy.logerr(f"ERROR: exception in pymodbus, {exc}")

            self.bus_mutex.release()

            if err_occurred:
                return

            if wr.isError():
                rospy.logerr(f"ERROR: pymodbus write_register returned an error: ({wr})")
                return

            # 设置目标位置
            values = []

            for i in range(len(msg.position)):
                value = int(msg.position[i] * 100)  # scale
                if value < 0:
                    value += 65536
                values.append(value)

            err_occurred = False
            self.bus_mutex.acquire()

            try:
                wr = self.modbus_client_.write_registers(address=ROH_FINGER_ANGLE_TARGET0, values=values, slave=hand_id)
            except Exception as exc:
                err_occurred = True
                rospy.logerr(f"ERROR: exception in pymodbus, {exc}")

            self.bus_mutex.release()

            if err_occurred:
                return

            if wr.isError():
                rospy.logerr(f"ERROR: pymodbus write_register returned an error: ({wr})")
                return

    def _thread_pub(self):
        last_update_time = time.time()

        while not rospy.is_shutdown():
            delta_time = time.time() - last_update_time
            last_update_time = time.time()

            for hand_id in self.hand_ids_:
                joint_states = JointState()

                joint_states.header.stamp = rospy.Time.now()
                joint_states.header.frame_id = FRAME_ID_PREFIX + str(hand_id)
                joint_states.name = ['thumb', 'index', 'middle', 'ring', 'little', 'thumb_rotation']

                # 读取当前位置
                try:
                    self.bus_mutex.acquire()
                    rr = self.modbus_client_.read_holding_registers(ROH_FINGER_ANGLE0, count=6, slave=hand_id)
                    self.bus_mutex.release()
                except ModbusException as exc:
                    rospy.logerr(f"ERROR: exception in pymodbus {exc}")
                    time.sleep(1.0)
                    continue

                if rr.isError():
                    rospy.logerr(f"ERROR: pymodbus read_holding_registers() returned an error: ({rr})")
                else:
                    for i in range(len(rr.registers)):
                        value = rr.registers[i]
                        if value > 32767:
                            value -= 65536
                        joint_states.position.append(value / 100)    # scale

                # TODO：读取当前速度
                joint_states.velocity = []

                # TODO: Read current forces
                try:
                    self.bus_mutex.acquire()
                    rr = self.modbus_client_.read_holding_registers(ROH_FINGER_FORCE0, count=6, slave=hand_id)
                    self.bus_mutex.release()
                except ModbusException as exc:
                    rospy.logerr(f"ERROR: exception in pymodbus {exc}")
                    time.sleep(1.0)
                    continue

                if rr.isError():
                    rospy.logerr(f"ERROR: pymodbus read_holding_registers() returned an error: ({rr})")
                else:
                    for i in range(len(rr.registers)):
                        value = rr.registers[i]
                        joint_states.effort.append(value)

                # 更新 header
                joint_states.header.stamp = rospy.Time.now()

                # 发布关节数据
                self.joint_states_publisher_.publish(joint_states)

            self.pub_rate.sleep()


def main(args=None):
    rospy.init_node('rohand_node')    
    node = ROHandNode()  # 新建一个节点

    rospy.spin()  # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.modbus_client_.close()
    rospy.shutdown()  # 关闭rospy
    
if __name__ == '__main__':
    main()