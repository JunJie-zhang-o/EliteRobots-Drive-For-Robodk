# -*- coding: UTF-8 -*-
# Copyright 2015-2022 - RoboDK Inc. - https://robodk.com/
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
#
#
# This is a Python module that allows driving a KUKA IIWA robot.
# This Python module can be run directly in console mode to test its functionality.
# This module allows communicating with a robot through the command line.
# The same commands we can input manually are used by RoboDK to drive the robot from the PC.
# RoboDK Drivers are located in /RoboDK/api/Robot/ by default. Drivers can be PY files or EXE files.
#
# Drivers are modular. They are not part of the RoboDK executable but they must be placed in C:/RoboDK/api/robot/, then, linked in the Connection parameters menu:
#   1. right click a robot in RoboDK, then, select "Connect to robot".
#   2. In the "More options" menu it is possible to update the location and name of the driver.
# Driver linking is automatic for currently available drivers.
# More information about robot drivers available here:
#   https://robodk.com/doc/en/Robot-Drivers.html#RobotDrivers
#
# Alternatively to the standard programming methods (where a program is generated, then, transferred to the robot and executed) it is possible to run a program simulation directly on the robot
# The robot movement in the simulator is then synchronized with the real robot.
# Programs generated from RoboDK can be run on the robot by right clicking the program, then selecting "Run on robot".
#   Example:
#   https://www.youtube.com/watch?v=pCD--kokh4s
#
# Example of an online programming project:
#   https://robodk.com/blog/online-programming/
#
# It is possible to control the movement of a robot from the RoboDK API (for example, from a Python or C# program using the RoboDK API).
# The same code is used to simulate and optionally move the real robot.
#   Example:
#   https://robodk.com/offline-programming
#
#   To establish connection from RoboDK API:
#   https://robodk.com/doc/en/PythonAPI/robolink.html#robolink.Item.ConnectSafe
#
# Example of a quick manual test in console mode:
#  User entry: CONNECT 192.168.123.1 7000
#  Response:   SMS:Response from the robot or failure to connect
#  Response:   SMS:Ready
#  User entry: MOVJ 10 20 30 40 50 60 70
#  Response:   SMS:Working...
#  Response:   SMS:Ready
#  User entry: CJNT
#  Response:   SMS:Working...
#  Response:   JNTS: 10 20 30 40 50 60 70
#
# ---------------------------------------------------------------------------------
from ast import While
from ipaddress import ip_interface
import socket
import struct
import sys
import re
import json
import time
import threading
from io import BytesIO

# ---------------------------------------------------------------------------------
# Set the minimum number of degrees of freedom that are expected
nDOFs_MIN = 7

# Set the driver version
DRIVER_VERSION = "RoboDK Driver for Elite Robot v1.0.1"

# ---------------------------------------------------------------------------------

MSG_CJNT = 1
MSG_SETTOOL = 2
MSG_SPEED = 3
MSG_ROUNDING = 4

MSG_MOVEJ = 10
MSG_MOVEL = 11
MSG_MOVEC = 12
MSG_MOVEL_SEARCH = 13

MSG_POPUP = 20
MSG_PAUSE = 21
MSG_RUNPROG = 22
MSG_SETDO = 23
MSG_WAITDI = 24
MSG_GETDI = 25
MSG_SETAO = 26
MSG_GETAI = 27

MSG_MONITOR = 127
MSG_ACKNOWLEDGE = 128

MSG_DISCONNECT = 999


def Robot_Disconnect():
    global ROBOT
    ROBOT.disconnect()


# ----------- communication class for Elite EC robots -------------
# This class handles communication between this driver (PC) and the robot
class ComRobot:
    """Robot class for programming Elite EC robots"""
    LAST_MSG = None  # Keep a copy of the last message received
    CONNECTED = False  # Connection status is known at all times

    # This is executed when the object is created
    def __init__(self):
        self.BUFFER_SIZE = 512  # bytes
        self.TIMEOUT = 5 * 60  # seconds: it must be enough time for a movement to complete
        # self.TIMEOUT = 10 # seconds
        self.sock = None
        self.sock2 = None

        # destructor

    def __del__(self):
        self.disconnect()

    # Disconnect from robot
    def disconnect(self):
        self.CONNECTED = False
        if self.sock:
            try:
                self.sock.close()

            except OSError:
                return False
        return True

    # Connect to robot
    def connect(self, ip, port=30001):
        global ROBOT_MOVING
        global bGetJoints
        global IPAdd
        global CS_state
        global curr_joints

        self.disconnect()
        print_message('Connecting to robot %s:%i' % (ip, port))

        IPAdd = ip
        UpdateStatus(ROBOTCOM_READY)
        #CS_state = 1
        self.CS_getjoints(ip)

        # while CS_state !=99:
        #     print('cs_state ',CS_state)
        #     time.sleep(0.1)

        self.CONNECTED = True
        ROBOT_MOVING = False
        print("CS机器人连接状态", self.CONNECTED)
        # self.send_line(DRIVER_VERSION)
        # flag,received = ROBOT.SendCmd('get_joint_pos')
        #          # received = ROBOT.recv_array()

        # print_joints(curr_joints)

        #bGetJoints = True
        # print('CS机器人处于',status,' 模式')
        sys.stdout.flush()
        return True

    def CS_sendcmd(self, IPAadd, cmd):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(36000)
            s.connect((IPAdd, 30001))
            try:
                # print(sendStr)
                s.sendall(bytes(cmd, "utf-8"))
            except Exception as e:
                return (False, None, None)

            s.close()
        #threading.Thread(target = self.v_robotmove).start()
        t1 = threading.Thread(target=self.v_robotmove, args=())
        t1.start()

    def v_robotmove(self):
        count = 0
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((IPAdd, 30001))
            bFinish = False
            while bFinish == False:
                __buf = s.recv(4096)
                # print(len(__buf))
                if len(__buf) <= 0:
                    time.sleep(0.01)

                while(len(__buf) > 0):
                    data_length = struct.unpack(">i", __buf[0:4])[0]
                    data_type = struct.unpack("B", __buf[4:5])[0]
                    #print(data_length, data_type)
                    data, __buf = __buf[0:data_length], __buf[data_length:]

                    if data_type == 16:
                        # MESSAGE_TYPE_ROBOT_STATE = 16
                        #bFinish = True
                        sub_type = 0

                        is_prg_running = struct.unpack("B", data[23:24])[0]
                        if is_prg_running == 0:
                            bFinish = True

                        data = data[5:data_length]
                        # 去掉前5个字节的报文头
                        while sub_type != 1:
                            #ROBOT_STATE_PACKAGE_TYPE_JOINT_DATA = 1
                            sub_length = struct.unpack(">i", data[0:4])[0]
                            sub_type = struct.unpack("B", data[4:5])[0]
                            data1, data = data[0:sub_length], data[sub_length:]

                        curr_data_add = 5
                        joints = [0, 0, 0, 0, 0, 0]
                        for i in range(0, 6):
                            joints[i] = struct.unpack(
                                ">d", data1[curr_data_add:curr_data_add+8])[0]
                            joints[i] = round(joints[i] / 3.1415926 * 180.0, 1)
                            curr_data_add = curr_data_add+(119-62)

                        count = count + 1
                        if (count % 1) == 0:
                            print_joints(joints)
                            # 每2个周期刷新一次模型

                time.sleep(0.01)
            s.close()
            UpdateStatus(ROBOTCOM_READY)

    def CS_getjoints(self, IPAdd):
        global curr_joints
        global out_joints
        global CS_state
        bFinish = False

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(36000)
            s.connect((IPAdd, 30001))
            while bFinish == False:
                __buf = s.recv(4096)
                # print(len(__buf))
                if len(__buf) <= 0:
                    time.sleep(0.01)

                while(len(__buf) > 0):
                    data_length = struct.unpack(">i", __buf[0:4])[0]
                    data_type = struct.unpack("B", __buf[4:5])[0]
                    #print(data_length, data_type)
                    data, __buf = __buf[0:data_length], __buf[data_length:]

                    if data_type == 16:
                        # MESSAGE_TYPE_ROBOT_STATE = 16
                        bFinish = True
                        sub_type = 0

                        data = data[5:data_length]
                        # 去掉前5个字节的报文头
                        while sub_type != 1:
                            #ROBOT_STATE_PACKAGE_TYPE_JOINT_DATA = 1
                            sub_length = struct.unpack(">i", data[0:4])[0]
                            sub_type = struct.unpack("B", data[4:5])[0]
                            data1, data = data[0:sub_length], data[sub_length:]

                        curr_data_add = 5
                        joints = [0, 0, 0, 0, 0, 0]
                        for i in range(0, 6):
                            joints[i] = struct.unpack(
                                ">d", data1[curr_data_add:curr_data_add+8])[0]
                            curr_joints[i] = round(
                                joints[i] / 3.1415926 * 180.0, 1)
                            curr_data_add = curr_data_add+(119-62)
            s.close()
        print_joints(curr_joints)

    # Send a line to the robot through the communication port (TCP/IP)

    def send_b(self, msg):
        try:
            sent = self.sock.send(msg)
            if sent == 0:
                return False
            return True
        except ConnectionAbortedError as e:
            self.CONNECTED = False
            print(str(e))
            return False

    # Receive a line from the robot through the communication port (TCP/IP)
    def recv_b(self, buffer_size):
        bytes_io = BytesIO()
        try:
            for i in range(buffer_size):
                bytes_io.write(self.sock.recv(1))
            b_data = bytes_io.getvalue()
        except ConnectionAbortedError as e:
            self.CONNECTED = False
            print(str(e))
            return None

        if b_data == b'':
            return None

        # self.LAST_MSG = b_data.decode('ascii')
        return b_data

    def sendCMD(self, sock, cmd, params=None, id=1):
        if (not params):
            params = []
        else:
            params = json.dumps(params)
        sendStr = "{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(
            cmd, params, id) + "\n"
        try:
            # print(sendStr)

            sock.sendall(bytes(sendStr, "utf-8"))
            ret = sock.recv(1024)
           # print(str(ret))

            jdata = json.loads(str(ret, "utf-8"))
            if ("result" in jdata.keys()):
                return (True, json.loads(jdata["result"]), jdata["id"])
            elif ("error" in jdata.keys()):
                print(sock.error["code"])
                print(sock.error["message"])
                return (False, json.loads(jdata["error"]), jdata["id"])
            else:
                return (False, None, None)
        except Exception as e:
            return (False, None, None)

    def sendCMD1(self, cmd, params=None, id=1):
        try:
            # print(sendStr)
            self.sock.sendall(bytes(cmd, "utf-8"))
        except Exception as e:
            return (False, None, None)

    def send_line(self, string=None):
        """Sends a string of characters with a \\n"""
        string = string.replace('\n', '<br>')
        if sys.version_info[0] < 3:
            return self.send_b(bytes(string + '\0'))  # Python 2.x only
        else:
            # Python 3.x only
            return self.send_b(bytes(string + '\0', 'utf-8'))

    def recv_line(self):
        """Receives a string. It reads until a null terminated string"""
        string = b''
        chari = self.recv_b(1)
        while chari != b'\0':  # read until null terminated
            string = string + chari
            chari = self.recv_b(1)
        return str(string.decode('utf-8'))  # python 2 and python 3 compatible

    def send_int(self, num):
        """Sends an int (32 bits)"""
        if isinstance(num, float):
            num = round(num)
        elif not isinstance(num, int):
            num = num[0]
        return self.send_b(struct.pack('>i', num))

    def recv_int(self):
        """Receives an int (32 bits)"""
        buffer = self.recv_b(4)
        num = struct.unpack('>i', buffer)
        return num[0]

    def recv_double(self):
        """Receives an double (64 bits)"""
        buffer = self.recv_b(8)
        num = struct.unpack('>d', buffer)
        return num[0]

    def recv_acknowledge(self):
        while True:
            stat_ack = self.recv_int()
            if stat_ack == MSG_MONITOR:
                jnts_moving = self.recv_array()
                print_joints(jnts_moving, True)

            elif stat_ack == MSG_ACKNOWLEDGE:
                return True

            else:
                print_message("Unexpected response from the robot")
                UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
                self.disconnect()
                return False

    def send_array(self, values):
        """Sends an array of doubles"""
        if not isinstance(values, list):  # if it is a Mat() with joints
            values = (values.tr()).rows[0]
        n_values = len(values)
        if not self.send_int(n_values):
            return False

        if n_values > 0:
            buffer = b''
            for i in range(n_values):
                buffer = buffer + struct.pack('>d', values[i])
            return self.send_b(buffer)

        return True

    def recv_array(self):
        """Receives an array of doubles"""
        n_values = self.recv_int()
        # print_message('n_values: %i' % n_values)
        values = []
        if n_values > 0:
            buffer = self.recv_b(8 * n_values)
            values = list(struct.unpack('>' + str(n_values) + 'd', buffer))
            # print('values: ' + str(values))
        return values

    def SendCmd(self, cmd, values=None):
        """Send a command. Returns True if success, False otherwise."""
        # print('SendCmd(cmd=' + str(cmd) + ', values=' + str(values) if values else '' + ')')
        # Skip the command if the robot is not connected
        if not self.CONNECTED:
            UpdateStatus(ROBOTCOM_NOT_CONNECTED)
            return False

        ret, result, id = self.sendCMD(self.sock, cmd, values)

        return (True, result)

    def get_joint_pos(self):

        # data = self.sock.recv(1538)
        # curr_data_add = 63
        # ########## 机器人actual_joint
        # #joints=[0,0,0,0,0,0]
        # is_robot_running = struct.unpack("B", data[23:24])[0]
        # # 获取机器人task 是否运行

        # for i in range(0,6):
        #     joints[i] = struct.unpack(">d", data[curr_data_add:curr_data_add+8])[0]
        #     joints[i] = joints[i] /3.1415926 *180.0
        #     curr_data_add =curr_data_add+(119-62)

        # return[is_robot_running,joints]

        __buf = self.sock.recv(4096)
        # if len(more) <= 0:
        #     time.sleep(0.01)
        #     continue
        # print(struct.unpack(">icicQ7?", data[:25]))
       # __buf = __buf
        while(len(__buf) > 0):
            data_length = struct.unpack(">i", __buf[0:4])[0]
            data_type = struct.unpack("B", __buf[4:5])[0]
           # print(data_type)
            data, __buf = __buf[0:data_length], __buf[data_length:]

            if data_type == 16:
                curr_data_add = 0

                joints = [0, 0, 0, 0, 0, 0]
                t1 = struct.unpack(">Q", data[10:18])[0]
                is_prg_running = struct.unpack("B", data[23:24])[0]

                curr_data_add = 63
                for i in range(0, 6):
                    joints[i] = struct.unpack(
                        ">d", data[curr_data_add:curr_data_add+8])[0]
                    joints[i] = round(joints[i] / 3.1415926 * 180.0, 1)
                    curr_data_add = curr_data_add+(119-62)

                # print(len(data),data_len1,len(__buf))
                return [is_prg_running, joints]

    def loop(self):
        global bGetJoints
        global bPrintJoints

        global CS_state
        global curr_joints
        global out_joints
        global IPAdd

        MESSAGE_TYPE_ROBOT_STATE = 16
        ROBOT_STATE_PACKAGE_TYPE_ROBOT_MODE_DATA = 0
        ROBOT_STATE_PACKAGE_TYPE_JOINT_DATA = 1
        ROBOT_STATE_PACKAGE_TYPE_CARTESIAN_INFO = 4
        ROBOT_STATE_PACKAGE_TYPE_CONFIGURATION_DATA = 6

        while True:
            if CS_state == 1:
                # 获取真实机器人joints
                bFinish = False

                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(36000)
                    s.connect((IPAdd, 30001))

                    while bFinish == False:
                        __buf = s.recv(4096)
                        # print(len(__buf))
                        if len(__buf) <= 0:
                            time.sleep(0.01)

                        while(len(__buf) > 0):
                            data_length = struct.unpack(">i", __buf[0:4])[0]
                            data_type = struct.unpack("B", __buf[4:5])[0]
                            #print(data_length, data_type)
                            data, __buf = __buf[0:data_length], __buf[data_length:]

                            if data_type == MESSAGE_TYPE_ROBOT_STATE:
                                # MESSAGE_TYPE_ROBOT_STATE = 16
                                bFinish = True
                                sub_type = 0

                                data = data[5:data_length]
                                # 去掉前5个字节的报文头
                                while sub_type != ROBOT_STATE_PACKAGE_TYPE_JOINT_DATA:
                                    #ROBOT_STATE_PACKAGE_TYPE_JOINT_DATA = 1
                                    sub_length = struct.unpack(
                                        ">i", data[0:4])[0]
                                    sub_type = struct.unpack("B", data[4:5])[0]
                                    data1, data = data[0:sub_length], data[sub_length:]

                                curr_data_add = 5
                                joints = [0, 0, 0, 0, 0, 0]
                                for i in range(0, 6):
                                    curr_joints[i] = struct.unpack(
                                        ">d", data1[curr_data_add:curr_data_add+8])[0]
                                    joints[i] = round(
                                        joints[i] / 3.1415926 * 180.0, 1)
                                    curr_data_add = curr_data_add+(119-62)
                    s.close()
                print_joints(curr_joints)
                CS_state = 99

            if CS_state == 2:
                # 发送 out_joints到机器人
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(36000)
                    s.connect((IPAdd, 30001))

                    out_script = 'def pcscript():\n'
                    out_script = out_script + \
                        '  movej('+str(out_joints)+',v=0.1)\n'
                    out_script = out_script + 'end\n'

                    try:
                        # print(sendStr)
                        self.sock.sendall(bytes(out_script, "utf-8"))
                    except Exception as e:
                        return (False, None, None)

                    s.close()

                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((IPAdd, 30001))
                    bFinish = False
                    while bFinish == False:
                        __buf = s.recv(4096)
                        # print(len(__buf))
                        if len(__buf) <= 0:
                            time.sleep(0.01)

                        while(len(__buf) > 0):
                            data_length = struct.unpack(">i", __buf[0:4])[0]
                            data_type = struct.unpack("B", __buf[4:5])[0]
                            #print(data_length, data_type)
                            data, __buf = __buf[0:data_length], __buf[data_length:]

                            if data_type == MESSAGE_TYPE_ROBOT_STATE:
                                # MESSAGE_TYPE_ROBOT_STATE = 16
                                #bFinish = True
                                sub_type = 0

                                is_prg_running = struct.unpack(
                                    "B", data[23:24])[0]
                                if is_prg_running == 0:
                                    bFinish = True

                                data = data[5:data_length]
                                # 去掉前5个字节的报文头
                                while sub_type != ROBOT_STATE_PACKAGE_TYPE_JOINT_DATA:
                                    #ROBOT_STATE_PACKAGE_TYPE_JOINT_DATA = 1
                                    sub_length = struct.unpack(
                                        ">i", data[0:4])[0]
                                    sub_type = struct.unpack("B", data[4:5])[0]
                                    data1, data = data[0:sub_length], data[sub_length:]

                                curr_data_add = 5
                                joints = [0, 0, 0, 0, 0, 0]
                                for i in range(0, 6):
                                    curr_joints[i] = struct.unpack(
                                        ">d", data1[curr_data_add:curr_data_add+8])[0]
                                    joints[i] = round(
                                        joints[i] / 3.1415926 * 180.0, 1)
                                    curr_data_add = curr_data_add+(119-62)
                                print_joints(curr_joints)

                        time.sleep(0.01)
                    s.close()
                    CS_state = 99

            if CS_state == 3:
                # 发送 out_joints到机器人
                time.sleep(0.1)

            if CS_state == 4:
                # 断开连接
                time.sleep(0.1)

            #print_message('state ',CS_state)
            time.sleep(0.1)

    def checkjoints(refjoints, newjoints):
        delta = 10
        for i in range(0, 6):
            if abs(refjoints[i] - newjoints[i]) > delta:
                return False
        return True


# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# Generic RoboDK driver for a specific Robot class
ROBOT = ComRobot()
ROBOT_IP = "192.168.230.138"  # IP of the robot
ROBOT_PORT = 30001  # Communication port of the robot
ROBOT_MOVING = False
bGetJoints = False
bPrintJoints = False
joints = [0, 0, 0, 0, 0, 0]
pre_joints = [0, 0, 0, 0, 0, 0]

# ------------ robot connection -----------------
# Establish connection with the robot


def RobotConnect():
    global ROBOT
    global ROBOT_IP
    global ROBOT_PORT
    return ROBOT.connect(ROBOT_IP, ROBOT_PORT)


# Disconnect from the robot
def RobotDisconnect():
    global ROBOT
    ROBOT.disconnect()
    return True


# -----------------------------------------------------------------------------
# Generic RoboDK driver tools

# Note, a simple print() will flush information to the log window of the robot connection in RoboDK
# Sending a print() might not flush the standard output unless the buffer reaches a certain size

def print_message(message):
    """print_message will display a message in the log window (and the connexion status bar)"""
    print("SMS:" + message)
    sys.stdout.flush()  # very useful to update RoboDK as fast as possible


def show_message(message):
    """show_message will display a message in the status bar of the main window"""
    print("SMS2:" + message)
    sys.stdout.flush()  # very useful to update RoboDK as fast as possible


def print_joints(joints, is_moving=False):
    # if len(joints) > 6:
    #    joints = joints[0:6]
    if is_moving:
        # Display the feedback of the joints when the robot is moving
        if ROBOT_MOVING:
            print("JNTS_MOVING " + " ".join(format(x, ".5f")
                  for x in joints))  # if joints is a list of float
            # print("JNTS_MOVING " + joints)
    else:
        print("JNTS " + " ".join(format(x, ".5f")
              for x in joints))  # if joints is a list of float
        # print("JNTS " + joints)
    sys.stdout.flush()  # very useful to update RoboDK as fast as possible


# ---------------------------------------------------------------------------------
# Constant values to display status using UpdateStatus()
ROBOTCOM_UNKNOWN = -1000
ROBOTCOM_CONNECTION_PROBLEMS = -3
ROBOTCOM_DISCONNECTED = -2
ROBOTCOM_NOT_CONNECTED = -1
ROBOTCOM_READY = 0
ROBOTCOM_WORKING = 1
ROBOTCOM_WAITING = 2

# Last robot status is saved
STATUS = ROBOTCOM_DISCONNECTED


# UpdateStatus will send an appropriate message to RoboDK which will result in a specific coloring
# for example, Ready will be displayed in green, Waiting... will be displayed in Yellow and other messages
# will be displayed in red
def UpdateStatus(set_status=None):
    global STATUS
    if set_status is not None:
        STATUS = set_status

    if STATUS == ROBOTCOM_CONNECTION_PROBLEMS:
        print_message("Connection problems")
    elif STATUS == ROBOTCOM_DISCONNECTED:
        print_message("Disconnected")
    elif STATUS == ROBOTCOM_NOT_CONNECTED:
        print_message("Not connected")
    elif STATUS == ROBOTCOM_READY:
        print_message("Ready")
    elif STATUS == ROBOTCOM_WORKING:
        print_message("Working...")
    elif STATUS == ROBOTCOM_WAITING:
        print_message("Waiting...")
    else:
        print_message("Unknown status")


# Sample set of commands that can be provided by RoboDK of through the command line
def TestDriver():

    # RunCommand("CONNECT 192.168.0.100 10000")
    RunCommand("CONNECT")
    RunCommand("RUNPROG -1 SetForceConditionOnce(12)")
    RunCommand("DISCONNECT")


# -------------------------- Main driver loop -----------------------------
# Read STDIN and process each command (infinite loop)
# IMPORTANT: This must be run from RoboDK so that RoboDK can properly feed commands through STDIN
# This driver can also be run in console mode providing the commands through the console input
def RunDriver():
    for line in sys.stdin:
        RunCommand(line)


# Each line provided through command line or STDIN will be processed by RunCommand
def RunCommand(cmd_line):
    global ROBOT_IP
    global ROBOT_PORT
    global ROBOT
    global ROBOT_MOVING
    global bGetJoints
    global bPrintJoints
    global pre_joints

    global curr_joints
    global out_joints
    global CS_state
    global IPAdd

    # strip a line of words into a list of numbers
    def line_2_values(line):
        values = []
        for word in line:
            try:
                number = float(word)
                values.append(number)
            except ValueError:
                pass
        return values

    cmd_words = cmd_line.split(' ')  # [''] if len == 0
    cmd = cmd_words[0]
    cmd_values = line_2_values(cmd_words[1:])  # [] if len <= 1
    n_cmd_values = len(cmd_values)
    n_cmd_words = len(cmd_words)
    received = None

    if cmd_line == "":
        # Skip if no command is provided
        return

    elif cmd_line.startswith("CONNECT"):
        # Connect to robot provided the IP and the port
        if n_cmd_words >= 2:
            ROBOT_IP = cmd_words[1]
        if n_cmd_words >= 3:
            ROBOT_PORT = int(cmd_words[2])
        received = RobotConnect()

    elif cmd_line.startswith("MOVJ"):
        UpdateStatus(ROBOTCOM_WORKING)
        # Activate the monitor feedback
        ROBOT_MOVING = True
        # Execute a joint move. RoboDK provides j1,j2,...,j6,j7,x,y,z,w,p,r

        out_joints = [0, 0, 0, 0, 0, 0]
        for i in range(0, 6):
            out_joints[i] = cmd_values[i]/180*3.1415926

        out_script = 'def pcscript():\n'
        out_script = out_script + \
            '  movej('+str(out_joints)+',v=10/180*3.14159)\n'
        out_script = out_script + 'end\n'
        ROBOT.CS_sendcmd(IPAdd, out_script)
        # ROBOT.sendCMD1(out_script)
        # CS_state = 2
        # while CS_state != 99:
        #     time.sleep(0.1)

        # UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= nDOFs_MIN and cmd_line.startswith("MOVLSEARCH"):
        UpdateStatus(ROBOTCOM_WORKING)
        # Activate the monitor feedback
        ROBOT_MOVING = True
        # Execute a linear move. RoboDK provides j1,j2,...,j6,x,y,z,w,p,r
        out_joints = []
        for i in range(0, 6):
            out_joints.append(cmd_values[i])

        params = {"targetPos": out_joints, "speed_type": 0, "speed": 10}
        if ROBOT.SendCmd('moveByLine', params):
            while(True):
                # 获取机器人状态
                suc, result = ROBOT.SendCmd('getRobotState')
                flag, received = ROBOT.SendCmd('get_joint_pos')
                # received = ROBOT.recv_array()
                print_joints(received)
                if (result == 0):
                    break

    elif n_cmd_values >= nDOFs_MIN and cmd_line.startswith("MOVL"):
        UpdateStatus(ROBOTCOM_WORKING)
        # Activate the monitor feedback
        ROBOT_MOVING = True
        # Execute a linear move. RoboDK provides j1,j2,...,j6,j7,x,y,z,w,p,r

        out_joints = []
        for i in range(0, 6):
            out_joints.append(cmd_values[i]/180*3.1415926)

        out_script = 'def pcscript():\n'
        out_script = out_script + \
            '  movel(get_forward_kin('+str(out_joints)+'),v=0.1)\n'
        out_script = out_script + 'end\n'

        ROBOT.CS_sendcmd(IPAdd, out_script)
        # ROBOT.sendCMD1(out_script)
        # CS_state = 2
        # while CS_state != 99:
        #     time.sleep(0.1)

        UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 2 * (nDOFs_MIN + 6) and cmd_line.startswith("MOVC"):
        UpdateStatus(ROBOTCOM_WORKING)
        # Activate the monitor feedback
        ROBOT_MOVING = True
        # Execute a circular move. RoboDK provides j1,j2,...,j6,x,y,z,w,p,r
        xyzwpr12 = cmd_values[-12:]
        if ROBOT.SendCmd(MSG_MOVEC, xyzwpr12):
            # Wait for command to be executed
            if ROBOT.recv_acknowledge():
                # Notify that we are done with this command
                UpdateStatus(ROBOTCOM_READY)

    elif cmd_line.startswith("CJNT"):
        UpdateStatus(ROBOTCOM_WORKING)
        # Retrieve the current position of the robot
        ROBOT.CS_getjoints(IPAdd)
        UpdateStatus(ROBOTCOM_READY)

        # print_joints(CS_state)

    elif n_cmd_values >= 1 and cmd_line.startswith("SPEED"):
        UpdateStatus(ROBOTCOM_WORKING)
        # First value is linear speed in mm/s
        # IMPORTANT! We should only send one "Ready" per instruction
        speed_values = [-1, -1, -1, -1]
        for i in range(max(4, len(cmd_values))):
            speed_values[i] = cmd_values[i]

        # speed_values[0] = speed_values[0] # linear speed in mm/s
        # speed_values[1] = speed_values[1] # joint speed in mm/s
        # speed_values[2] = speed_values[2] # linear acceleration in mm/s2
        # speed_values[3] = speed_values[3] # joint acceleration in deg/s2

        if ROBOT.SendCmd(MSG_SPEED, speed_values):
            # Wait for command to be executed
            if ROBOT.recv_acknowledge():
                # Notify that we are done with this command
                UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 6 and cmd_line.startswith("SETTOOL"):
        UpdateStatus(ROBOTCOM_WORKING)
        # Set the Tool reference frame provided the 6 XYZWPR cmd_values by RoboDK
        if ROBOT.SendCmd(MSG_SETTOOL, cmd_values):
            # Wait for command to be executed
            if ROBOT.recv_acknowledge():
                # Notify that we are done with this command
                UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 1 and cmd_line.startswith("PAUSE"):
        UpdateStatus(ROBOTCOM_WAITING)
        # Run a pause
        if ROBOT.SendCmd(MSG_PAUSE, cmd_values[0]):
            # Wait for command to be executed
            if ROBOT.recv_acknowledge():
                # Notify that we are done with this command
                UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 1 and cmd_line.startswith("SETROUNDING"):
        # Set the rounding/smoothing value. Also known as ZoneData in ABB or CNT for Fanuc
        if ROBOT.SendCmd(MSG_ROUNDING, cmd_values[0]):
            # Wait for command to be executed
            if ROBOT.recv_acknowledge():
                # Notify that we are done with this command
                UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 2 and cmd_line.startswith("SETDO"):
        UpdateStatus(ROBOTCOM_WORKING)
        # dIO_id = cmd_values[0]
        # dIO_value = cmd_values[1]
        if ROBOT.SendCmd(MSG_SETDO, cmd_values[0:2]):
            # Wait for command to be executed
            if ROBOT.recv_acknowledge():
                # Notify that we are done with this command
                UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 2 and cmd_line.startswith("WAITDI"):
        UpdateStatus(ROBOTCOM_WORKING)
        # dIO_id = cmd_values[0]
        # dIO_value = cmd_values[1]
        if ROBOT.SendCmd(MSG_WAITDI, cmd_values[0:2]):
            # Wait for command to be executed
            if ROBOT.recv_acknowledge():
                # Notify that we are done with this command
                UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 1 and n_cmd_words >= 3 and cmd_line.startswith("RUNPROG"):
        UpdateStatus(ROBOTCOM_WORKING)
        # Program ID is extracted automatically if the program name is Program ID
        program_id = cmd_values[0]
        code = cmd_words[2]  # "Program%i" % program_id
        m = re.search(r'^(?P<program_name>.*)\((?P<args>.*)\)', code)
        code_dict = m.groupdict()
        program_name = code_dict['program_name']
        args = code_dict['args'].replace(' ', '').split(',')
        print('program_name: ' + program_name)
        print('args: ' + str(args))

        ROBOT.SendCmd(MSG_RUNPROG)
        ROBOT.send_int(program_id)
        ROBOT.send_line(program_name)
        for a in args:
            ROBOT.send_line(a)

        # Wait for the program call to complete
        if ROBOT.recv_acknowledge():
            # Notify that we are done with this command
            UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_words >= 2 and cmd_line.startswith("POPUP "):
        UpdateStatus(ROBOTCOM_WORKING)
        message = cmd_line[6:]
        ROBOT.send_line(message)
        # Wait for command to be executed
        if ROBOT.recv_acknowledge():
            # Notify that we are done with this command
            UpdateStatus(ROBOTCOM_READY)

    elif cmd_line.startswith("DISCONNECT"):
        # Disconnect from robot
        # ROBOT.SendCmd(MSG_DISCONNECT)
        # ROBOT.recv_acknowledge()
        ROBOT.disconnect()
        UpdateStatus(ROBOTCOM_DISCONNECTED)

    elif cmd_line.startswith("STOP") or cmd_line.startswith("QUIT"):
        # Stop the driverç
        # bGetJoints = False
        # ROBOT.SendCmd('stop')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(36000)
            s.connect((IPAdd, 30001))

            out_script = 'def pcscript():\n'
            out_script = out_script + '  stopj(3)\n'
            out_script = out_script + 'end\n'

            try:
                # print(sendStr)
                s.sendall(bytes(out_script, "utf-8"))
            except Exception as e:
                return (False, None, None)

            s.close()

        # ROBOT.disconnect()
        UpdateStatus(ROBOTCOM_READY)
        # UpdateStatus(ROBOTCOM_DISCONNECTED)
        # quit(0)  # Stop the driver

    elif cmd_line.startswith("t"):
        # Call custom procedure for quick testing
        TestDriver()

    else:
        print("Unknown command: " + str(cmd_line))

    if received is not None:
        UpdateStatus(ROBOTCOM_READY)
    # Stop monitoring feedback
    ROBOT_MOVING = False


def RunMain():
    """Call Main procedure"""

    # Flush version
    print_message("RoboDK Driver v1.0 for Elite Robot controllers")

    # It is important to disconnect the robot if we force to stop the process
    import atexit

    atexit.register(RobotDisconnect)

    # Flush Disconnected message
    print_message(DRIVER_VERSION)
    UpdateStatus()

    # Run the driver from STDIN
    RunDriver()

    # Test the driver with a sample set of commands
    # TestDriver()


curr_joints = [0, 0, 0, 0, 0, 0]
# 获取真实机器人当前角度
out_joints = [0, 0, 0, 0, 0, 0]
# 把robodk里的机器人角度发给真实机器人运动
CS_state = 0
# 控制loop里的状态

if __name__ == "__main__":
    """Call Main procedure"""
    # Important, leave the Main procedure as RunMain
    #threading.Thread(target = ROBOT.loop, daemon = True).start()
    RunMain()
