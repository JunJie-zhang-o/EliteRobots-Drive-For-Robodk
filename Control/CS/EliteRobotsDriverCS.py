# -*- coding: UTF-8 -*-
# Copyright 2015-2024 - RoboDK Inc. - https://robodk.com/
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
# This is a Python module that allows driving a EliteRobots CS robot.
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
import socket
import struct
import sys
import threading
import time
import select
import math

from io import BytesIO

tcp_pose = [0, 0, 0, 0, 0, 0]
speed_values = [-1, -1, -1, -1]
rounding_value = -1
# ---------------------------------------------------------------------------------
# Set the minimum number of degrees of freedom that are expected
nDOFs_MIN = 6

# Set the driver version
DRIVER_VERSION = "RoboDK Driver for Elite Robots CS v2.0.0"

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


def is_angle_close(angle1, angle2, angle_abs_tol=0.1):
    error = angle1 - angle2
    if error < -math.pi:
        return math.isclose(error + 2 * math.pi, 0, abs_tol=angle_abs_tol)
    elif error >= math.pi:
        return math.isclose(error - 2 * math.pi, 0, abs_tol=angle_abs_tol)
    else:
        return math.isclose(error, 0, abs_tol=angle_abs_tol)


DEFAULT_TIMEOUT = 10.0

ROBOT_STATE_TYPE = 16


class RobotDataConfig:

    def __init__(self):
        self.name = [
            "total_message_len",
            "total_message_type",
            "mode_sub_len",
            "mode_sub_type",
            "timestamp",
            "reserver",
            "reserver",
            "is_robot_power_on",
            "is_emergency_stopped",
            "is_robot_protective_stopped",
            "is_program_running",
            "is_program_paused",
            "get_robot_mode",
            "get_robot_control_mode",
            "get_target_speed_fraction",
            "get_speed_scaling",
            "get_target_speed_fraction_limit",
            "get_robot_speed_mode",
            "is_robot_system_in_alarm",
            "is_in_package_mode",
            "reverse",
            "joint_sub_len",
            "joint_sub_type",
            "actual_joint0",
            "target_joint0",
            "actual_velocity0",
            "target_pluse0",
            "actual_pluse0",
            "zero_pluse0",
            "current0",
            "voltage0",
            "temperature0",
            "torques0",
            "mode0",
            "reverse0",
            "actual_joint1",
            "target_joint1",
            "actual_velocity1",
            "target_pluse1",
            "actual_pluse1",
            "zero_pluse1",
            "current1",
            "voltage1",
            "temperature1",
            "torques1",
            "mode1",
            "reverse1",
            "actual_joint2",
            "target_joint2",
            "actual_velocity2",
            "target_pluse2",
            "actual_pluse2",
            "zero_pluse2",
            "current2",
            "voltage2",
            "temperature2",
            "torques2",
            "mode2",
            "reverse2",
            "actual_joint3",
            "target_joint3",
            "actual_velocity3",
            "target_pluse3",
            "actual_pluse3",
            "zero_pluse3",
            "current3",
            "voltage3",
            "temperature3",
            "torques3",
            "mode3",
            "reverse3",
            "actual_joint4",
            "target_joint4",
            "actual_velocity4",
            "target_pluse4",
            "actual_pluse4",
            "zero_pluse4",
            "current4",
            "voltage4",
            "temperature4",
            "torques4",
            "mode4",
            "reverse4",
            "actual_joint5",
            "target_joint5",
            "actual_velocity5",
            "target_pluse5",
            "actual_pluse5",
            "zero_pluse5",
            "current5",
            "voltage5",
            "temperature5",
            "torques5",
            "mode5",
            "reverse5",
            "cartesial_sub_len",
            "cartesial_sub_type",
            "tcp_x",
            "tcp_y",
            "tcp_z",
            "rot_x",
            "rot_y",
            "rot_z",
            "offset_px",
            "offset_py",
            "offset_pz",
            "offset_rotx",
            "offset_roty",
            "offset_rotz",
            "configuration_sub_len",
            "configuration_sub_type",
            "limit_min_joint_x0",
            "limit_max_joint_x0",
            "limit_min_joint_x1",
            "limit_max_joint_x1",
            "limit_min_joint_x2",
            "limit_max_joint_x2",
            "limit_min_joint_x3",
            "limit_max_joint_x3",
            "limit_min_joint_x4",
            "limit_max_joint_x4",
            "limit_min_joint_x5",
            "limit_max_joint_x5",
            "max_velocity_joint_x0",
            "max_acc_joint_x0",
            "max_velocity_joint_x1",
            "max_acc_joint_x1",
            "max_velocity_joint_x2",
            "max_acc_joint_x2",
            "max_velocity_joint_x3",
            "max_acc_joint_x3",
            "max_velocity_joint_x4",
            "max_acc_joint_x4",
            "max_velocity_joint_x5",
            "max_acc_joint_x5",
            "default_velocity_joint",
            "default_acc_joint",
            "default_tool_velocity",
            "default_tool_acc",
            "eq_radius",
            "dh_a_joint_x0",
            "dh_a_joint_x1",
            "dh_a_joint_x2",
            "dh_a_joint_x3",
            "dh_a_joint_x4",
            "dh_a_joint_x5",
            "dh_d_joint_d0",
            "dh_d_joint_d1",
            "dh_d_joint_d2",
            "dh_d_joint_d3",
            "dh_d_joint_d4",
            "dh_d_joint_d5",
            "dh_alpha_joint_x0",
            "dh_alpha_joint_x1",
            "dh_alpha_joint_x2",
            "dh_alpha_joint_x3",
            "dh_alpha_joint_x4",
            "dh_alpha_joint_x5",
            "reserver0",
            "reserver1",
            "reserver2",
            "reserver3",
            "reserver4",
            "reserver5",
            "board_version",
            "control_box_type",
            "robot_type",
            "robot_struct",
            "masterboard_sub_len",
            "masterboard_sub_type",
            "digital_input_bits",
            "digital_output_bits",
            "standard_analog_input_domain0",
            "standard_analog_input_domain1",
            "tool_analog_input_domain",
            "standard_analog_input_value0",
            "standard_analog_input_value1",
            "tool_analog_input_value",
            "standard_analog_output_domain0",
            "standard_analog_output_domain1",
            "tool_analog_output_domain",
            "standard_analog_output_value0",
            "standard_analog_output_value1",
            "tool_analog_output_value",
            "bord_temperature",
            "robot_voltage",
            "robot_current",
            "io_current",
            "bord_safe_mode",
            "is_robot_in_reduced_mode",
            "get_operational_mode_selector_input",
            "get_threeposition_enabling_device_input",
            "masterboard_safety_mode",
            "additional_sub_len",
            "additional_sub_type",
            "is_freedrive_button_pressed",
            "reserve",
            "is_freedrive_io_enabled",
            "is_dynamic_collision_detect_enabled",
            "reserver",
            "tool_sub_len",
            "tool_sub_type",
            "tool_analog_output_domain",
            "tool_analog_input_domain",
            "tool_analog_output_value",
            "tool_analog_input_value",
            "tool_voltage",
            "tool_output_voltage",
            "tool_current",
            "tool_temperature",
            "tool_mode",
            "safe_sub_len",
            "safe_sub_type",
            "safety_crc_num",
            "safety_operational_mode",
            "reserver",
            "current_elbow_position_x",
            "current_elbow_position_y",
            "current_elbow_position_z",
            "elbow_radius",
            "tool_comm_sub_len",
            "tool_comm_sub_type",
            "is_enable",
            "baudrate",
            "parity",
            "stopbits",
            "tci_modbus_status",
            "tci_usage",
            "reserved0",
            "reserved1",
        ]
        self.fmt = ">IBIBQ???????BBdddB??IIBdddiiiffffBidddiiiffffBidddiiiffffBidddiiiffffBidddiiiffffBidddiiiffffBiIBddddddddddddIBdddddddddddddddddddddddddddddddddddddddddddddddddddddIIIIIBIIBBBdddBBBdddffffB???BIB????BIBBBddfBffBIBIbBddddIB?III?Bff"


class RobotHeader:
    __slots__ = [
        "type",
        "size",
    ]

    @staticmethod
    def unpack(buf):
        rmd = RobotHeader()
        (rmd.size, rmd.type) = struct.unpack_from(">iB", buf)
        return rmd


class RobotData:

    @staticmethod
    def unpack(buf, config):
        data = RobotData()
        unpack = struct.unpack_from(config.fmt, buf)
        for i in range(len(config.name)):
            name = config.name[i]
            data.__dict__[name] = unpack[i]
        return data


class ReadAlarm:

    @staticmethod
    def unpack(buf):
        data_length = struct.unpack(">i", buf[0:4])[0]

        data, buf = buf[0:data_length], buf[data_length:]
        msg_type = data[14]

        if msg_type == 10:

            script_line = struct.unpack(">i", data[15:19])[0]
            script_column = struct.unpack(">i", data[19:23])[0]

            b = bytearray(data[23:data_length - 1])
            msg = b.decode()
            print_message("Error line:" + str(script_line))
            print_message("Error column:" + str(script_column))
            print_message("Error info:" + msg)
            ROBOT.CS_getjoints()

        if msg_type == 6:

            error_code = struct.unpack(">i", data[15:19])[0]
            sub_error_code = struct.unpack(">i", data[19:23])[0]
            level = struct.unpack(">i", data[23:27])[0]
            dtype = struct.unpack(">i", data[27:31])[0]
            if dtype == 6:

                joint = struct.unpack(">i", data[31:35])[0]
                print_message("Error: E" + str(error_code) + "S" + str(sub_error_code) + ":J" + str(joint))
                ROBOT.CS_getjoints()

        return True


# ----------- Communication class for EliteRobots CS robots -------------
class ComRobot:
    """Robot class for programming Elite CS robots"""

    LAST_MSG = None
    CONNECTED = False

    def __init__(self):
        self.BUFFER_SIZE = 512
        self.TIMEOUT = 5 * 60
        self.sock = None
        config = RobotDataConfig()
        self.__data_config = config

    def __del__(self):
        self.disconnect()

    def disconnect(self):

        self.CONNECTED = False
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                return False
        return True

    def connect(self, ip, port=30001):
        global ROBOT_MOVING
        global IPAdd
        global HostIP

        self.disconnect()
        print_message("Connecting to robot %s:%i" % (ip, port))

        IPAdd = ip

        self.CONNECTED = True
        ROBOT_MOVING = False

        try:
            ROBOT.CS_getjoints()
        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
            self.CONNECTED = False
            ROBOT_MOVING = False
            print_message("Connect Failed")
            return False
        else:
            if ROBOT_DOF > nDOFs_MIN:
                print_message("ERROR: Current not supports synchronize external axes!")
                print_message("ERROR: Try again after removing sync with external axes!")
            else:
                UpdateStatus(ROBOTCOM_READY)

            return True

    def newconnect(self, ip, port=30001):
        try:
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.__sock.settimeout(DEFAULT_TIMEOUT)
            self.hostname = ip
            self.port = port
            self.__sock.connect((self.hostname, self.port))
            self.__buf = bytes()
        except (socket.timeout, socket.error):
            self.__sock = None
            raise

    def get_data(self):
        return self.__recv()

    def __recv(self):
        try:
            self.__recv_to_buffer(DEFAULT_TIMEOUT)
        except:
            return None
        while len(self.__buf) > 5:
            head = RobotHeader.unpack(self.__buf)
            if head.size <= len(self.__buf):

                if head.type != ROBOT_STATE_TYPE:
                    if head.type == 20:
                        ReadAlarm.unpack(self.__buf)
                    self.__buf = self.__buf[head.size:]
                    continue
                data = RobotData.unpack(self.__buf, self.__data_config)
                self.__buf = self.__buf[head.size:]
                return data
            else:
                break
        return None

    def __recv_to_buffer(self, timeout):
        readable, _, xlist = select.select([self.__sock], [], [self.__sock], timeout)
        if len(readable):
            more = self.__sock.recv(4096)

            if len(more) == 0:

                return None

            self.__buf = self.__buf + more
            return True

        if (len(xlist) or len(readable) == 0) and timeout != 0:

            return None

        return False

    def tcp_compare(self, tcp_pose):
        new = ComRobot()
        new.newconnect(IPAdd)

        retry = True
        __status = True

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:

                tcp_offset = [
                    data.offset_px,
                    data.offset_py,
                    data.offset_pz,
                    data.offset_rotx,
                    data.offset_roty,
                    data.offset_rotz,
                ]
                for num in range(len(tcp_pose) - 3):
                    if not math.isclose(tcp_offset[num], tcp_pose[num], abs_tol=0.1):
                        __status = False

                for num in range(len(tcp_pose) - 3, len(tcp_pose)):
                    if not is_angle_close(tcp_offset[num], tcp_pose[num], angle_abs_tol=0.1):
                        __status = False

                if __status == False:
                    if retry == True:
                        __status = True

                else:

                    return True
            time.sleep(0.1)

    def tcp_check(self, tcp_pose):
        new = ComRobot()
        new.newconnect(IPAdd)

        __status = True

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:

                tcp_offset = [
                    data.offset_px,
                    data.offset_py,
                    data.offset_pz,
                    data.offset_rotx,
                    data.offset_roty,
                    data.offset_rotz,
                ]
                for num in range(len(tcp_pose) - 3):
                    if not math.isclose(tcp_offset[num], tcp_pose[num], abs_tol=0.1):
                        __status = False

                for num in range(len(tcp_pose) - 3, len(tcp_pose)):
                    if not is_angle_close(tcp_offset[num], tcp_pose[num], angle_abs_tol=0.1):
                        __status = False

                if __status == False:
                    return False

                else:

                    return True

    def do_check(self, dIO_id, dIO_value):
        new = ComRobot()
        new.newconnect(IPAdd)

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:

                if len(bin(data.digital_output_bits)) - 2 < int(dIO_id) + 1:
                    continue
                else:
                    if bin(data.digital_output_bits)[-int(dIO_id) - 1] == str(dIO_value):
                        return True

            time.sleep(0.1)

    def ao_check(self, aIO_id, aIO_domain, aIO_value):
        new = ComRobot()
        new.newconnect(IPAdd)

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:

                if aIO_id == 0:
                    if aIO_domain == 0:

                        if data.standard_analog_output_domain0 == aIO_domain:
                            if math.isclose(
                                    data.standard_analog_output_value0,
                                (aIO_value * 16 + 4) / 1000,
                                    abs_tol=0.1,
                            ):

                                return True
                    elif aIO_domain == 1:

                        if data.standard_analog_output_domain0 == aIO_domain:
                            if math.isclose(
                                    data.standard_analog_output_value0,
                                    aIO_value * 10,
                                    abs_tol=0.1,
                            ):
                                return True
                    elif aIO_domain == -1:

                        if data.standard_analog_output_domain0 == 0:
                            if math.isclose(
                                    data.standard_analog_output_value0,
                                (aIO_value * 16 + 4) / 1000,
                                    abs_tol=0.1,
                            ):
                                return True
                        elif data.standard_analog_output_domain0 == 1:
                            if math.isclose(
                                    data.standard_analog_output_value0,
                                    aIO_value * 10,
                                    abs_tol=0.1,
                            ):
                                return True

                elif aIO_id == 1:
                    if aIO_domain == 0:

                        if data.standard_analog_output_domain1 == aIO_domain:
                            if math.isclose(
                                    data.standard_analog_output_value1,
                                (aIO_value * 16 + 4) / 1000,
                                    abs_tol=0.1,
                            ):
                                return True
                    elif aIO_domain == 1:

                        if data.standard_analog_output_domain1 == aIO_domain:
                            if math.isclose(
                                    data.standard_analog_output_value1,
                                    aIO_value * 10,
                                    abs_tol=0.1,
                            ):
                                return True
                    elif aIO_domain == -1:

                        if data.standard_analog_output_domain1 == 0:
                            if math.isclose(
                                    data.standard_analog_output_value1,
                                (aIO_value * 16 + 4) / 1000,
                                    abs_tol=0.1,
                            ):
                                return True
                        elif data.standard_analog_output_domain1 == 1:
                            if math.isclose(
                                    data.standard_analog_output_value1,
                                    aIO_value * 10,
                                    abs_tol=0.1,
                            ):
                                return True

            time.sleep(0.1)

    def di_wait(self, dIO_id, dIO_value):
        new = ComRobot()
        new.newconnect(IPAdd)

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:

                if len(bin(data.digital_input_bits)) - 2 < int(dIO_id) + 1:
                    if dIO_value == 0:
                        return True
                    else:
                        continue
                else:
                    if bin(data.digital_input_bits)[-int(dIO_id) - 1] == str(dIO_value):
                        return True

            time.sleep(0.1)

    def CS_getjoints(self):
        new = ComRobot()
        new.newconnect(IPAdd)

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:
                actual_joints = [
                    data.actual_joint0,
                    data.actual_joint1,
                    data.actual_joint2,
                    data.actual_joint3,
                    data.actual_joint4,
                    data.actual_joint5,
                ]
                actual_joints_degrees = [
                    math.degrees(actual_joints[0]),
                    math.degrees(actual_joints[1]),
                    math.degrees(actual_joints[2]),
                    math.degrees(actual_joints[3]),
                    math.degrees(actual_joints[4]),
                    math.degrees(actual_joints[5]),
                ]
                for i in range(ROBOT_DOF - nDOFs_MIN):

                    actual_joints_degrees.append(0)
                print_joints(actual_joints_degrees)

                return True

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

        if b_data == b"":
            return None

        return b_data

    def send_line(self, string=None):
        """Sends a string of characters with a \\n"""
        string = string.replace("\n", "<br>")
        if sys.version_info[0] < 3:
            return self.send_b(bytes(string + "\0"))
        else:

            return self.send_b(bytes(string + "\0", "utf-8"))

    def recv_line(self):
        """Receives a string. It reads until a null terminated string"""
        string = b""
        chari = self.recv_b(1)
        while chari != b"\0":
            string = string + chari
            chari = self.recv_b(1)
        return str(string.decode("utf-8"))

    def send_int(self, num):
        """Sends an int (32 bits)"""
        if isinstance(num, float):
            num = round(num)
        elif not isinstance(num, int):
            num = num[0]
        return self.send_b(struct.pack(">i", num))

    def recv_int(self):
        """Receives an int (32 bits)"""
        buffer = self.recv_b(4)
        num = struct.unpack(">i", buffer)
        return num[0]

    def recv_double(self):
        """Receives an double (64 bits)"""
        buffer = self.recv_b(8)
        num = struct.unpack(">d", buffer)
        return num[0]

    def SendMOV(self, out_script, out_pose):
        global STATUS
        global ROBOT_MOVING

        if rounding_value != -1:
            new_abs = 0.1 + rounding_value / 1000
        else:
            new_abs = 0.1

        new = ComRobot()
        new.newconnect(IPAdd)

        __status = True

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:

                if not data.is_program_running:
                    ROBOT_MOVING = False

                    before_tcp = [
                        data.tcp_x,
                        data.tcp_y,
                        data.tcp_z,
                        data.rot_x,
                        data.rot_y,
                        data.rot_z,
                    ]
                    break

                else:
                    ROBOT_MOVING = True
            time.sleep(0.1)

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:

                if not data.is_program_running:
                    if not data.is_program_paused:
                        ROBOT_MOVING = False

                        robot_mode = data.get_robot_mode
                        if robot_mode == 7:
                            ROBOT.SendCmd(out_script)
                        else:
                            print_message("Error, please check the robot status.")
                            if robot_mode == 3:
                                print_message("Please turn on the power and release brakes first!")
                            elif robot_mode == 5:
                                print_message("Please release brakes first!")
                            return False
                        break

                else:
                    ROBOT_MOVING = True
            time.sleep(0.1)

        while True:
            data = new.get_data()
            if data == None:
                continue
            else:

                if not data.is_program_running:
                    if not data.is_program_paused:

                        ROBOT_MOVING = False
                        after_tcp = [
                            data.tcp_x,
                            data.tcp_y,
                            data.tcp_z,
                            data.rot_x,
                            data.rot_y,
                            data.rot_z,
                        ]

                        for num in range(len(after_tcp) - 3):
                            if not math.isclose(after_tcp[num], before_tcp[num], abs_tol=0.1):
                                return True

                        for num in range(len(after_tcp) - 3):
                            if not math.isclose(after_tcp[num], out_pose[num], abs_tol=new_abs):
                                __status = False
                                if not data.is_program_paused:
                                    ROBOT.SendCmd(out_script)
                                break
                        if __status == True:
                            return True

                else:
                    ROBOT_MOVING = True
            time.sleep(0.1)

    def SendPopup(self, outscript):
            ROBOT.SendCmd(outscript)

            new = ComRobot()
            new.newconnect(IPAdd)

            while True:
                data = new.get_data()
                if data == None:
                    continue
                else:
                    if not data.is_program_running:
                        if not data.is_program_paused:
                                return True

                time.sleep(0.1)

    def send_array(self, values):
        """Sends an array of doubles"""
        if not isinstance(values, list):
            values = (values.tr()).rows[0]
        n_values = len(values)
        if not self.send_int(n_values):
            return False

        if n_values > 0:
            buffer = b""
            for i in range(n_values):
                buffer = buffer + struct.pack(">d", values[i])
            return self.send_b(buffer)

        return True

    def recv_array(self):
        """Receives an array of doubles"""
        n_values = self.recv_int()

        values = []
        if n_values > 0:
            buffer = self.recv_b(8 * n_values)
            values = list(struct.unpack(">" + str(n_values) + "d", buffer))

        return values

    def SendCmd(self, cmd, values=None):
        """Send a command. Returns True if success, False otherwise."""

        if not self.CONNECTED:
            UpdateStatus(ROBOTCOM_NOT_CONNECTED)
            return False

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(36000)
            s.connect((IPAdd, 30001))
            try:

                s.sendall(bytes(cmd, "utf-8"))
            except Exception as e:
                return False

            s.close()
        return True


# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
# Generic RoboDK driver for a specific Robot class
ROBOT = ComRobot()
ROBOT_IP = "192.168.230.138"
ROBOT_PORT = 30001
ROBOT_MOVING = False


# ------------ Robot connection -----------------
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
    sys.stdout.flush()


def show_message(message):
    """show_message will display a message in the status bar of the main window"""
    print("SMS2:" + message)
    sys.stdout.flush()


def print_joints(joints, is_moving=False):

    if is_moving:

        if ROBOT_MOVING:
            print("JNTS_MOVING: " + " ".join(format(x, ".3f") for x in joints))

    else:
        print("JNTS: " + " ".join(format(x, ".6f") for x in joints))

    sys.stdout.flush()


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

    RunCommand("CONNECT")
    RunCommand("SPEED 250")
    RunCommand("SETTOOL -0.025 -41.046 50.920 60.000 -0.000 90.000")
    RunCommand("CJNT")
    RunCommand("MOVJ -5.362010 46.323420 20.746290 74.878840 -50.101680 61.958500")
    RunCommand("MOVEL 0 0 0 0 0 0 -5.362010 50.323420 20.746290 74.878840 -50.101680 61.958500")
    RunCommand("PAUSE 1000")
    RunCommand("DISCONNECT")


# -------------------------- Main driver loop -----------------------------
# Read STDIN and process each command (infinite loop)
# IMPORTANT: This must be run from RoboDK so that RoboDK can properly feed commands through STDIN
# This driver can also be run in console mode providing the commands through the console input
def RunDriver():
    for line in sys.stdin:
        RunCommand(line)


def RunCommand(cmd_line):
    global ROBOT_IP
    global ROBOT_PORT
    global ROBOT_DOF
    global ROBOT
    global ROBOT_MOVING

    global IPAdd

    global tcp_pose
    global speed_values
    global rounding_value

    def line_2_values(line):
        values = []
        for word in line:
            try:
                number = float(word)
                values.append(number)
            except ValueError:
                pass
        return values

    print(cmd_line)

    cmd_words = cmd_line.split(" ")
    cmd = cmd_words[0]
    cmd_values = line_2_values(cmd_words[1:])
    n_cmd_values = len(cmd_values)
    n_cmd_words = len(cmd_words)
    received = None

    if cmd_line == "":

        return

    elif cmd_line.startswith("CONNECT"):

        if n_cmd_words >= 2:
            ROBOT_IP = cmd_words[1]
        if n_cmd_words >= 3:
            ROBOT_PORT = int(cmd_words[2])
        if n_cmd_words >= 4:
            ROBOT_DOF = int(cmd_words[3])
        received = RobotConnect()

    elif n_cmd_values >= nDOFs_MIN and cmd_line.startswith("MOVJ"):
        UpdateStatus(ROBOTCOM_WORKING)

        ROBOT_MOVING = True

        out_pose = []
        for cmd_value in cmd_values[-6:-3]:
            out_pose.append(cmd_value / 1000)
        for cmd_value in cmd_values[-3:]:
            out_pose.append(math.radians(cmd_value))

        out_joints = []
        for cmd_value in cmd_values[0:6]:

            out_joints.append(math.radians(cmd_value))

        out_script = "def pcscript():\n"
        out_script = out_script + "  movej(" + str(out_joints)

        if speed_values[3] != -1:
            out_script = out_script + ",a=d2r(" + str(speed_values[3]) + ")"

        if speed_values[1] != -1:
            out_script = out_script + ",v=d2r(" + str(speed_values[1]) + ")"

        if rounding_value != -1:
            out_script = out_script + ",r=" + str(rounding_value / 1000)

        out_script = out_script + ")\n"
        out_script = out_script + "end\n"

        try:
            if ROBOT.tcp_check(tcp_pose):
                if ROBOT.SendMOV(out_script, out_pose):
                    UpdateStatus(ROBOTCOM_READY)
            else:
                print_message("TCP in RoboDK is different from Robot, please check the TCP setting first")

        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)

    elif n_cmd_values >= nDOFs_MIN and cmd_line.startswith("MOVLSEARCH"):

        UpdateStatus(ROBOTCOM_WORKING)

        ROBOT_MOVING = True

        out_pose = []
        for cmd_value in cmd_values[-6:-3]:
            out_pose.append(cmd_value / 1000)
        for cmd_value in cmd_values[-3:]:
            out_pose.append(math.radians(cmd_value))

        out_script = "def pcscript():\n"
        out_script = out_script + "  movel(" + str(out_pose)

        if speed_values[2] != -1:
            out_script = out_script + ",a=" + str(speed_values[2] / 1000)

        if speed_values[0] != -1:
            out_script = out_script + ",v=" + str(speed_values[0] / 1000)

        if rounding_value != -1:
            out_script = out_script + ",r=" + str(rounding_value / 1000)

        out_script = out_script + ")\n"
        out_script = out_script + "end\n"

        try:
            if ROBOT.tcp_check(tcp_pose):
                if ROBOT.SendMOV(out_script, out_pose):
                    UpdateStatus(ROBOTCOM_READY)
            else:
                print_message("TCP in RoboDK is different from Robot, please check the TCP setting first")

        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)

    elif n_cmd_values >= nDOFs_MIN and cmd_line.startswith("MOVL"):

        UpdateStatus(ROBOTCOM_WORKING)

        ROBOT_MOVING = True

        out_pose = []
        for cmd_value in cmd_values[-6:-3]:
            out_pose.append(cmd_value / 1000)
        for cmd_value in cmd_values[-3:]:
            out_pose.append(math.radians(cmd_value))

        out_script = "def pcscript():\n"
        out_script = out_script + "  movel(" + str(out_pose)

        if speed_values[2] != -1:
            out_script = out_script + ",a=" + str(speed_values[2] / 1000)

        if speed_values[0] != -1:
            out_script = out_script + ",v=" + str(speed_values[0] / 1000)

        if rounding_value != -1:
            out_script = out_script + ",r=" + str(rounding_value / 1000)

        out_script = out_script + ")\n"
        out_script = out_script + "end\n"

        try:
            if ROBOT.tcp_check(tcp_pose):
                if ROBOT.SendMOV(out_script, out_pose):
                    UpdateStatus(ROBOTCOM_READY)
            else:
                print_message("TCP in RoboDK is different from Robot, please check the TCP setting first")

        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)

    elif n_cmd_values >= 2 * (nDOFs_MIN + 6) and cmd_line.startswith("MOVC"):

        UpdateStatus(ROBOTCOM_WORKING)

        ROBOT_MOVING = True

        pose_via = []
        for cmd_value in cmd_values[-12:-9]:
            pose_via.append(cmd_value / 1000)
        for cmd_value in cmd_values[-9:-6]:
            pose_via.append(math.radians(cmd_value))

        pose_to = []
        for cmd_value in cmd_values[-6:-3]:
            pose_to.append(cmd_value / 1000)
        for cmd_value in cmd_values[-3:]:
            pose_to.append(math.radians(cmd_value))

        out_script = "def pcscript():\n"
        out_script = out_script + "  movec(" + str(pose_via) + "," + str(pose_to)

        if speed_values[2] != -1:
            out_script = out_script + ",a=" + str(speed_values[2] / 1000)

        if speed_values[0] != -1:
            out_script = out_script + ",v=" + str(speed_values[0] / 1000)

        if rounding_value != -1:
            out_script = out_script + ",r=" + str(rounding_value / 1000)

        out_script = out_script + ",mode=1)\n"
        out_script = out_script + "end\n"

        try:
            if ROBOT.tcp_check(tcp_pose):
                if ROBOT.SendMOV(out_script, pose_to):
                    UpdateStatus(ROBOTCOM_READY)
            else:
                print_message("TCP in RoboDK is different from Robot, please check the TCP setting first")

        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)

    elif cmd_line.startswith("CJNT"):
        UpdateStatus(ROBOTCOM_WORKING)

        try:
            ROBOT.CS_getjoints()
        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
        else:
            UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 1 and cmd_line.startswith("SPEED"):

        UpdateStatus(ROBOTCOM_WORKING)

        for i in range(max(4, len(cmd_values))):
            speed_values[i] = cmd_values[i]

        UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 6 and cmd_line.startswith("SETTOOL"):

        UpdateStatus(ROBOTCOM_WORKING)

        tcp_pose = []
        for i in range(0, 3):
            tcp_pose.append(cmd_values[i] / 1000)
        for i in range(3, 6):
            tcp_pose.append(math.radians(cmd_values[i]))

        out_script = "sec pcscript():\n"
        out_script = out_script + "  set_tcp(" + str(tcp_pose)

        out_script = out_script + ")\n"
        out_script = out_script + "end\n"

        try:
            ROBOT.SendCmd(out_script)
        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
        else:

            if ROBOT.tcp_compare(tcp_pose):

                UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 1 and cmd_line.startswith("PAUSE"):
        UpdateStatus(ROBOTCOM_WAITING)

        pausetime = cmd_values[0] / 1000
        if pausetime >= 0:
            time.sleep(cmd_values[0] / 1000)
            UpdateStatus(ROBOTCOM_READY)
        else:
            out_script = "def pcscript():\n"
            out_script = (out_script + "  popup(s='Pause',title='Message From RoboDK',blocking=True)\n")
            out_script = out_script + "end\n"

            try:
                ROBOT.SendPopup(out_script)
            except:
                UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
            else:
                UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 1 and cmd_line.startswith("SETROUNDING"):

        UpdateStatus(ROBOTCOM_WORKING)

        rounding_value = cmd_values[0]
        UpdateStatus(ROBOTCOM_READY)

    elif n_cmd_values >= 2 and cmd_line.startswith("SETDO"):
        UpdateStatus(ROBOTCOM_WORKING)
        dIO_id = -1
        dIO_value = -1

        if cmd_words[3] not in [str(i) for i in range(16)]:
            print_message("Please check the IO Name:")
            print_message("The index of the output, integer type data: [0:15]")

        else:
            dIO_id = cmd_words[3]

        if (cmd_words[4].replace("\n", "").replace("\r", "") == "1" or cmd_words[4].replace("\n", "").replace("\r", "") == "True"):
            dIO_value = 1
        elif (cmd_words[4].replace("\n", "").replace("\r", "") == "0" or cmd_words[4].replace("\n", "").replace("\r", "") == "False"):
            dIO_value = 0
        else:
            print_message("Please check the IO Value:")
            print_message("Signal level, boolean type data, True or False")

        out_script = []
        if dIO_id != -1 and dIO_value != -1:
            out_script = "sec pcscript():\n"

            out_script = (out_script + "  set_standard_digital_out(" + dIO_id + "," + str(bool(dIO_value)) + ")\n")
            out_script = out_script + "end\n"

            try:
                ROBOT.SendCmd(out_script)
            except:
                UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
            else:
                if ROBOT.do_check(dIO_id, dIO_value):
                    UpdateStatus(ROBOTCOM_READY)

        else:
            UpdateStatus(ROBOTCOM_UNKNOWN)

    elif n_cmd_values >= 2 and cmd_line.startswith("SETAO"):
        UpdateStatus(ROBOTCOM_WORKING)
        aIO_id = -1
        aIO_domain = -1
        aIO_value = -1

        if cmd_words[3] == "0" or cmd_words[3] == "analog_out[0]":
            aIO_id = 0
        elif cmd_words[3] == "1" or cmd_words[3] == "analog_out[1]":
            aIO_id = 1
        else:
            print_message("Please check the IO Name:")
            print_message("The index of the output, integer type data: [0:1]")

        if cmd_words[4].replace("\n", "").replace("\r", "")[-2:].lower() == "ma":
            aIO_domain = 0
            try:
                current = float(cmd_words[4].replace("\n", "").replace("\r", "")[:-2])
            except:
                print_message("Please check the IO Value:")
                print_message("Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0.")

            else:
                if current < 4 or current > 20:
                    print_message("Please check the IO Value:")
                    print_message("current mode (4-20mA)")

                else:
                    aIO_value = (current - 4) / 16
        elif cmd_words[4].replace("\n", "").replace("\r", "")[-1:].lower() == "v":
            aIO_domain = 1
            try:
                voltage = float(cmd_words[4].replace("\n", "").replace("\r", "")[:-1])
            except:
                print_message("Please check the IO Value:")
                print_message("Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0.")

            else:
                if voltage < 0 or voltage > 10:
                    print_message("Please check the IO Value:")
                    print_message("voltage mode (0-10V)")

                else:
                    aIO_value = voltage / 10
        else:
            try:
                if float(cmd_words[4]) < 0:
                    aIO_value = 0
                elif float(cmd_words[4]) > 1:
                    aIO_value = 1
                else:
                    aIO_value = float(cmd_words[4])
            except:
                print("Please check the IO Value:")
                print("Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0.")

        out_script = []
        if aIO_id != -1 and aIO_value != -1:
            out_script = "sec pcscript():\n"

            if aIO_domain == 0 or aIO_domain == 1:
                out_script = (out_script + "  set_standard_analog_output_domain(" + str(aIO_id) + "," + str(aIO_domain) + ")\n")

            out_script = (out_script + "  set_standard_analog_out(" + str(aIO_id) + "," + str(aIO_value) + ")\n")
            out_script = out_script + "end\n"

            try:
                ROBOT.SendCmd(out_script)
            except:
                UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
            else:
                if ROBOT.ao_check(aIO_id, aIO_domain, aIO_value):
                    UpdateStatus(ROBOTCOM_READY)

        else:
            UpdateStatus(ROBOTCOM_UNKNOWN)

    elif n_cmd_values >= 2 and cmd_line.startswith("WAITDI"):
        UpdateStatus(ROBOTCOM_WORKING)
        dIO_id = -1
        dIO_value = -1

        if cmd_words[3] not in [str(i) for i in range(16)]:
            print_message("Please check the IO Name:")
            print_message("The index of the input, integer type data: [0:15]")

        else:
            dIO_id = cmd_words[3]

        if (cmd_words[4].replace("\n", "").replace("\r", "") == "1" or cmd_words[4].replace("\n", "").replace("\r", "") == "True"):
            dIO_value = 1
        elif (cmd_words[4].replace("\n", "").replace("\r", "") == "0" or cmd_words[4].replace("\n", "").replace("\r", "") == "False"):
            dIO_value = 0
        else:
            print_message("Please check the IO Value:")
            print_message("Signal level, boolean type data, True or False")

        if dIO_id != -1 and dIO_value != -1:

            try:
                ROBOT.di_wait(dIO_id, dIO_value)
            except:
                UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
            else:
                UpdateStatus(ROBOTCOM_READY)

        else:
            UpdateStatus(ROBOTCOM_UNKNOWN)

    elif n_cmd_words >= 2 and cmd_line.startswith("POPUP "):
        UpdateStatus(ROBOTCOM_WORKING)
        message = cmd_line[6:].replace("\n", "").replace("\r", "")

        out_script = "def pcscript():\n"
        out_script = (out_script + "  popup(s=" + repr(message) + ",title='Message From RoboDK',blocking=True)\n")
        out_script = out_script + "end\n"

        try:
            ROBOT.SendPopup(out_script)
        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
        else:
            UpdateStatus(ROBOTCOM_READY)

    elif cmd_line.startswith("DISCONNECT"):

        try:
            ROBOT.disconnect()
        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)
        else:
            UpdateStatus(ROBOTCOM_DISCONNECTED)

    elif cmd_line.startswith("STOP") or cmd_line.startswith("QUIT"):

        out_script = "def pcscript():\n"
        out_script = out_script + "  stopj(5)\n"
        out_script = out_script + "end\n"

        try:
            ROBOT.SendCmd(out_script)
        except:
            UpdateStatus(ROBOTCOM_CONNECTION_PROBLEMS)

        else:

            UpdateStatus(ROBOTCOM_READY)

    elif cmd_line.startswith("t"):

        TestDriver()

    else:
        print("Unknown command: " + str(cmd_line))

    ROBOT_MOVING = False


def RunMain():
    """Call Main procedure"""

    # It is important to disconnect the robot if we force to stop the process
    import atexit

    atexit.register(RobotDisconnect)

    # Flush Disconnected message
    print_message(DRIVER_VERSION)
    UpdateStatus()

    # Run the driver from STDIN
    RunDriver()

    # Test the driver with a sample set of commands
    TestDriver()


if __name__ == "__main__":
    """Call Main procedure"""

    # Important, leave the Main procedure as RunMain
    RunMain()
