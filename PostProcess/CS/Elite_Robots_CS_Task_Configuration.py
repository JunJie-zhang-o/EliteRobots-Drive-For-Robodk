# Copyright 2015 - RoboDK Inc. - https://robodk.com/
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ----------------------------------------------------
# This file is a POST PROCESSOR for Robot Offline Programming to generate programs
#
# To edit/test this POST PROCESSOR script file:
# Select "Program"->"Add/Edit Post Processor", then select your post or create a new one.
# You can edit this file using any text editor or Python editor. Using a Python editor allows to quickly evaluate a sample program at the end of this file.
# Python should be automatically installed with RoboDK
#
# You can also edit the POST PROCESSOR manually:
#    1- Open the *.py file with Python IDLE (right click -> Edit with IDLE)
#    2- Make the necessary changes
#    3- Run the file to open Python Shell: Run -> Run module (F5 by default)
#    4- The "test_post()" function is called automatically
# Alternatively, you can edit this file using a text editor and run it with Python
#
# To use a POST PROCESSOR file you must place the *.py file in "C:/RoboDK/Posts/"
# To select one POST PROCESSOR for your robot in RoboDK you must follow these steps:
#    1- Open the robot panel (double click a robot)
#    2- Select "Parameters"
#    3- Select "Unlock advanced options"
#    4- Select your post as the file name in the "Robot brand" box
#
# To delete an existing POST PROCESSOR script, simply delete this file (.py file)
#
# ----------------------------------------------------
# More information about RoboDK Post Processors and Offline Programming here:
#     https://robodk.com/help#PostProcessor
#     https://robodk.com/doc/en/PythonAPI/postprocessor.html
# ----------------------------------------------------

# ----------------------------------------------------
# Description
#
#
# Supported Controllers
#
# ----------------------------------------------------
import xml.etree.ElementTree as ET
import uuid
import math
import re
import keyword
import builtins


from robodk.robomath import *
from robodk.robodialogs import *
from robodk.robofileio import *

max_string_len = 15


def name_add_num(varStr):
    if len(varStr) < max_string_len - 1:
        if "_" in varStr:
            varStrList = varStr.split("_")
            if varStrList[-1].isdecimal():
                num = str(int(varStrList[-1]) + 1)
                varStr = "_".join(varStrList[:-1]) + "_" + num
            else:
                varStr = varStr + "_1"
        else:
            varStr = varStr + "_1"
    elif len(varStr) == max_string_len - 1:
        if "_" in varStr:
            varStrList = varStr.split("_")
            if varStrList[-1].isdecimal():
                num = str(int(varStrList[-1]) + 1)
                varStr = "_".join(varStrList[:-1]) + "_" + num
            else:
                varStr = varStr[:-1] + "_1"
        else:
            varStr = varStr[:-1] + "_1"
    else:
        varStr = varStr[:max_string_len]
        if "_" in varStr:
            varStrList = varStr.split("_")
            if varStrList[-1].isdecimal():
                num = str(int(varStrList[-1]) + 1)
                varStr = varStr[: -len(num) - 1] + "_" + num
            else:
                varStr = varStr[:-2] + "_1"
        else:
            varStr = varStr[:-2] + "_1"
    return varStr


EliteKeywords = [
    "def",
    "sec",
    "end",
    "len",
    "max",
    "min",
    "list",
    "append",
    "count",
    "extend",
    "index",
    "insert",
    "pop",
    "remove",
    "reverse",
    "sort",
    "tuple",
    "int",
    "long",
    "float",
    "complex",
    "str",
    "repr",
    "eval",
    "set",
    "dict",
    "frozenset",
    "chr",
    "unichr",
    "ord",
    "hex",
    "oct",
    "movej",
    "movec",
    "movel",
    "get_actual_joint_positions",
    "get_actual_joint_speeds",
    "get_actual_tcp_pose",
    "get_actual_tcp_speed",
    "get_controller_temperature",
    "get_joint_temperatures",
    "get_joint_torques",
    "get_target_joint_positions",
    "get_target_joint_speeds",
    "get_target_tcp_pose",
    "get_target_tcp_speed",
    "set_gravity",
    "set_payload",
    "get_target_payload_cog",
    "get_target_payload_mass",
    "set_tcp",
    "encoder_enable_set_tick_count",
    "encoder_get_tick_count",
    "encoder_set_tick_count",
    "encoder_unwind_delta_tick_count",
    "stop_conveyor_tracking",
    "speedj",
    "speedl",
    "stopj",
    "stopl",
    "track_conveyor_circular",
    "track_conveyor_linear",
    "get_actual_tool_flange_pose",
    "get_target_waypoint",
    "get_tcp_offset",
    "load_micro_line_file",
    "micro_line_set_pcs",
    "get_micro_line_values_by_index",
    "moveml",
    "get_inverse_kin",
    "get_inverse_kin_has_solution",
    "get_forward_kin",
    "binary_list_to_integer",
    "integer_to_binary_list",
    "get_list_length",
    "length",
    "acos",
    "asin",
    "atan",
    "atan2",
    "cos",
    "sin",
    "tan",
    "d2r",
    "r2d",
    "ceil",
    "floor",
    "log",
    "sqrt",
    "pow",
    "norm",
    "normalize",
    "point_dist",
    "pose_add",
    "pose_dist",
    "pose_inv",
    "pose_sub",
    "pose_trans",
    "rotvec2rpy",
    "rpy2rotvec",
    "interpolate_pose",
    "random",
    "get_configurable_digital_in",
    "get_configurable_digital_out",
    "get_standard_analog_in",
    "get_standard_analog_out",
    "get_standard_digital_in",
    "get_standard_digital_out",
    "get_tool_analog_in",
    "get_tool_analog_out",
    "set_standard_analog_output_domain",
    "set_configurable_digital_out",
    "set_standard_analog_input_domain",
    "set_standard_analog_out",
    "set_standard_digital_out",
    "set_tool_analog_input_domain",
    "set_tool_analog_output_domain",
    "set_tool_digital",
    "get_tool_digital",
    "set_runstate_configurable_digital_output_to_value",
    "set_runstate_gp_boolean_output_to_value",
    "set_runstate_standard_analog_output_to_value",
    "set_runstate_tool_analog_output_to_value",
    "set_runstate_standard_digital_output_to_value",
    "set_runstate_tool_digital_output_to_value",
    "set_input_actions_to_default",
    "set_configurable_digital_input_action",
    "set_standard_digital_input_action",
    "set_tool_digital_input_action",
    "set_gp_boolean_input_action",
    "tool_serial_is_open",
    "tool_serial_config",
    "tool_serial_read",
    "tool_serial_write",
    "tool_modbus_read_registers",
    "tool_modbus_read_bits",
    "tool_modbus_write_registers",
    "tool_modbus_write_bits",
    "tool_modbus_read_input_registers",
    "tool_modbus_read_input_bits",
    "tool_modbus_write_single_bit",
    "tool_modbus_write_single_register",
    "tool_serial_mode",
    "set_tool_voltage",
    "serial_config",
    "serial_read",
    "serial_write",
    "serial_is_open",
    "serial_mode",
    "serial_modbus_write_registers",
    "serial_modbus_write_bits",
    "serial_modbus_read_registers",
    "serial_modbus_read_bits",
    "serial_modbus_read_input_registers",
    "serial_modbus_read_input_bits",
    "serial_modbus_write_single_bit",
    "serial_modbus_write_single_register",
    "serial_flush",
    "get_task_path",
    "read_input_boolean_register",
    "read_input_float_register",
    "read_input_integer_register",
    "read_output_boolean_register",
    "read_output_float_register",
    "read_output_integer_register",
    "write_output_boolean_register",
    "write_output_float_register",
    "write_output_integer_register",
    "modbus_add_signal",
    "modbus_delete_signal",
    "modbus_get_signal_status",
    "modbus_send_custom_command",
    "modbus_set_output_register",
    "modbus_set_output_signal",
    "modbus_set_runstate_dependent_choice",
    "modbus_set_signal_update_frequency",
    "read_port_bit",
    "read_port_register",
    "write_port_bit",
    "write_port_register",
    "socket_open",
    "socket_close",
    "socket_get_var",
    "socket_read_ascii_float",
    "socket_read_binary_integer",
    "socket_read_byte_list",
    "socket_send_byte_list",
    "socket_read_string",
    "socket_send_byte",
    "socket_send_int",
    "socket_send_line",
    "socket_send_string",
    "socket_set_var",
    "rpc_factory",
    "start_thread",
    "stop_thread",
    "textmsg",
    "popup",
    "str_at",
    "str_cat",
    "str_empty",
    "str_find",
    "str_len",
    "str_sub",
    "to_num",
    "to_str",
    "sleep",
    "set_flag",
    "get_flag",
    "get_steptime",
    "rtsi_set_watchdog",
    "servoj",
    "powerdown",
]
EliserverKeywords = [
    "xor",
    "end",
    "thread",
    "run",
    "kill",
    "halt",
    "join",
    "local",
    "acos",
    "asin",
    "assert",
    "assert_equals",
    "atan",
    "atan2",
    "binary_list_to_integer",
    "blocking_usleep",
    "ceil",
    "cma_avg",
    "cma_max",
    "cma_min",
    "cma_reset",
    "cma_tick",
    "cma_tock",
    "conveyor_pulse_decode",
    "cos",
    "d2r",
    "debugmsg",
    "enable_external_ft_sensor",
    "encoder_enable_pulse_decode",
    "encoder_enable_set_tick_count",
    "encoder_get_tick_count",
    "encoder_set_tick_count",
    "encoder_unwind_delta_tick_count",
    "end_force_mode",
    "end_freedrive_mode",
    "end_joint_torque_mode",
    "end_screw_driving",
    "end_teach_mode",
    "enter_critical",
    "exit_critical",
    "floor",
    "force",
    "force_mode",
    "force_mode_set_damping",
    "force_mode_set_gain_scaling",
    "freedrive_mode",
    "get_actual_joint_positions",
    "get_actual_joint_positions_history",
    "get_actual_joint_speeds",
    "get_actual_tcp_pose",
    "get_actual_tcp_speed",
    "get_actual_tool_flange_pose",
    "get_analog_in",
    "get_analog_out",
    "get_configurable_digital_in",
    "get_configurable_digital_out",
    "get_controller_temp",
    "get_conveyor_tick_count",
    "get_digital_in",
    "get_digital_out",
    "get_digital_out",
    "get_euromap_input",
    "get_euromap_output",
    "get_flag",
    "get_forward_kin",
    "get_forward_kin",
    "get_inverse_kin",
    "get_joint_positions",
    "get_joint_speeds",
    "get_joint_temp",
    "get_joint_torques",
    "get_list_length",
    "get_max_torque_fraction",
    "get_numerical_inverse_kin",
    "get_robot_power",
    "get_standard_analog_in",
    "get_standard_analog_out",
    "get_standard_digital_in",
    "get_standard_digital_out",
    "get_steptime",
    "get_target_joint_positions",
    "get_target_joint_speeds",
    "get_target_payload",
    "get_target_payload_cog",
    "get_target_tcp_pose",
    "get_target_tcp_speed",
    "get_target_waypoint",
    "get_tcp_force",
    "get_tcp_offset",
    "get_tcp_power",
    "get_tcp_speed",
    "get_tool_accelerometer_reading",
    "get_tool_analog_in",
    "get_tool_current",
    "get_tool_digital_in",
    "get_tool_digital_out",
    "get_tool_digital_output_mode",
    "get_tool_output_mode",
    "inkognito",
    "integer_to_binary_list",
    "interpolate_pose",
    "is_steady",
    "is_within_safety_limits",
    "joint_torque_mode",
    "jump_to_label",
    "labelmsg",
    "length",
    "log",
    "MODBUS",
    "modbus_add_signal",
    "modbus_delete_signal",
    "modbus_get_signal_status",
    "modbus_request_update_signal_value",
    "modbus_send_custom_command",
    "modbus_set_digital_input_action",
    "modbus_set_output_register",
    "modbus_set_output_signal",
    "modbus_set_runstate_dependent_choice",
    "modbus_set_signal_update_frequency",
    "movec",
    "movel",
    "movel_rel",
    "movep",
    "norm",
    "normalize",
    "pathj",
    "point_dist",
    "popup",
    "pose_add",
    "pose_dist",
    "pose_inv",
    "pose_sub",
    "pose_trans",
    "position_deviation_warning",
    "pow",
    "powerdown",
    "protective_stop",
    "r2d",
    "random",
    "read_input_boolean_register",
    "read_input_float_register",
    "read_input_integer_register",
    "read_output_boolean_register",
    "read_output_float_register",
    "read_output_integer_register",
    "read_output_integer_register",
    "read_port_bit",
    "read_port_register",
    "read_register",
    "request_boolean_from_primary_client",
    "request_float_from_primary_client",
    "request_integer_from_primary_client",
    "request_string_from_primary_client",
    "reset_revolution_counter",
    "rotvec2rpy",
    "rpc_factory",
    "rpc_stub",
    "rpy2rotvec",
    "rtde_set_watchdog",
    "screw_driving",
    "script_stop",
    "send_joint_regulation_parameter",
    "servoc",
    "servoj",
    "servol",
    "set_analog_inputrange",
    "set_analog_inputrange",
    "set_analog_out",
    "set_analog_outputdomain",
    "set_analog_outputdomain",
    "set_configurable_digital_in",
    "set_configurable_digital_input_action",
    "set_configurable_digital_out",
    "set_conveyor_tick_count",
    "set_digital_in",
    "set_digital_out",
    "set_euromap_output",
    "set_euromap_runstate_dependent_choice",
    "set_flag",
    "set_gp_boolean_input_action",
    "set_gravity",
    "set_gravity",
    "set_input_actions_to_default",
    "set_joint_param",
    "set_max_torque_fraction",
    "set_payload",
    "set_payload",
    "set_payload_cog",
    "set_payload_mass",
    "set_pos",
    "set_runstate_configurable_digital_output_to_value",
    "set_runstate_configurable_digital_output_to_value",
    "set_runstate_configurable_digital_outputs",
    "set_runstate_configurable_digital_outputs",
    "set_runstate_gp_boolean_output_to_value",
    "set_runstate_gp_boolean_output_to_value",
    "set_runstate_output_to_value",
    "set_runstate_output_to_value",
    "set_runstate_outputs",
    "set_runstate_outputs",
    "set_runstate_standard_analog_output_to_value",
    "set_runstate_standard_analog_output_to_value",
    "set_runstate_standard_analog_outputs",
    "set_runstate_standard_analog_outputs",
    "set_runstate_standard_digital_output_to_value",
    "set_runstate_standard_digital_output_to_value",
    "set_runstate_standard_digital_outputs",
    "set_runstate_standard_digital_outputs",
    "set_runstate_tool_digital_output_to_value",
    "set_runstate_tool_digital_output_to_value",
    "set_runstate_tool_digital_outputs",
    "set_runstate_tool_digital_outputs",
    "set_safety_mode_transition_hardness",
    "set_standard_analog_input_domain",
    "set_standard_analog_input_domain",
    "set_standard_analog_out",
    "set_standard_digital_in",
    "set_standard_digital_input_action",
    "set_standard_digital_out",
    "set_standard_digital_out",
    "set_tcp",
    "set_tcp",
    "set_tcp_wrench",
    "set_tool_analog_input_domain",
    "set_tool_analog_input_domain",
    "set_tool_communication",
    "set_tool_digital_in",
    "set_tool_digital_input_action",
    "set_tool_digital_out",
    "set_tool_digital_output_mode",
    "set_tool_output_mode",
    "set_tool_voltage",
    "set_tool_voltage",
    "sin",
    "sleep",
    "socket_close",
    "socket_get_var",
    "socket_open",
    "socket_read_ascii_float",
    "socket_read_binary_integer",
    "socket_read_byte_list",
    "socket_read_line",
    "socket_read_string",
    "socket_send_int",
    "socket_send_line",
    "socket_send_string",
    "socket_set_var",
    "speedj",
    "speedl",
    "sqrt",
    "start_thread",
    "stop_conveyor_tracking",
    "stop_thread",
    "stopj",
    "stopl",
    "str_at",
    "str_cat",
    "str_empty",
    "str_find",
    "str_sub",
    "sync",
    "tan",
    "teach_mode",
    "textmsg",
    "to_num",
    "to_str",
    "tool_contact",
    "tool_pose",
    "track_conveyor_circular",
    "track_conveyor_linear",
    "tracka",
    "varmsg",
    "wrench_trans",
    "write_output_boolean_register",
    "write_output_float_register",
    "write_port_bit",
    "write_port_register",
    "write_register",
    "zero_ftsensor",
]

RobodkKeywords = [
    "TCP",
    "Home",
    "负载",
    "工具法兰",
    "Ctrl_view_frame",
    "Ctrl_base_frame",
    "Ctrl_tool_frame",
]
RobotKeywords = ["点", "路点", "计时器", "Waypoint", "Timer"]
TCPNames = []
FrameNames = []
ProgNames = []


def get_safe_name(varStr: str, type: str = "None") -> str:
    """
    Get a safe program or variable name that can be used for robot programming.

    :param varStr: The name to filter
    :type varStr: str
    :param type: The type decide which set to compare with to avoid duplicates
    :type type: str, optional, 'None|TCP|Frame|Program'

    :return: The filtered name
    :rtype: str
    """
    varStr = re.sub("\W|^(?=\d)", "_", varStr[:max_string_len])
    if varStr[0] == "_":
        varStr = "N" + varStr
        varStr = get_safe_name(varStr)

    if varStr in keyword.kwlist:
        varStr = name_add_num(varStr)
        varStr = get_safe_name(varStr)
    if varStr in dir(builtins):
        varStr = name_add_num(varStr)
        varStr = get_safe_name(varStr)
    if varStr in EliteKeywords:
        varStr = name_add_num(varStr)
        varStr = get_safe_name(varStr)
    if varStr in EliserverKeywords:
        varStr = name_add_num(varStr)
        varStr = get_safe_name(varStr)

    if varStr in RobodkKeywords:
        varStr = name_add_num(varStr)
        varStr = get_safe_name(varStr)
    for RobotKeyword in RobotKeywords:
        if varStr.startswith(RobotKeyword):
            varStr = "N_" + varStr
            varStr = get_safe_name(varStr)
    if type == "TCP":
        if varStr in FrameNames:
            varStr = name_add_num(varStr)
            varStr = get_safe_name(varStr, "TCP")
        if varStr in ProgNames:
            varStr = name_add_num(varStr)
            varStr = get_safe_name(varStr, "TCP")

    if type == "Frame":
        if varStr in TCPNames:
            varStr = name_add_num(varStr)
            varStr = get_safe_name(varStr, "Frame")
        if varStr in ProgNames:
            varStr = name_add_num(varStr)
            varStr = get_safe_name(varStr, "Frame")
    if type == "Program":
        if varStr in TCPNames:
            varStr = name_add_num(varStr)
            varStr = get_safe_name(varStr, "Program")
        if varStr in FrameNames:
            varStr = name_add_num(varStr)
            varStr = get_safe_name(varStr, "Program")

    return varStr


# ----------------------------------------------------
def pose_2_list(pose):
    """Converts a robot pose target to a list according to the syntax/format of the controller.

    **Tip**: Change the output of this function according to your controller.

    :param pose: 4x4 pose matrix
    :type pose: :meth:`robodk.robomath.Mat`
    :return: postion as a XYZWPR string
    :rtype: str
    """
    [x, y, z, r, p, w] = pose_2_xyzrpw(pose)
    return [x, y, z, math.radians(r), math.radians(p), math.radians(w)]


def joints_2_list(joints):
    """Converts a robot joint target to a list according to the syntax/format of the controller.

    **Tip**: Change the output of this function according to your controller.

    :param joints: robot joints as a list
    :type joints: float list
    :return: joint format as a J1-Jn string
    :rtype: str
    """

    joints_rad = []
    for joint in joints:
        joints_rad.append(math.radians(joint))
    return joints_rad


# ----------------------------------------------------
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """RoboDK Post Processor object.

    As Post processors can be compiled (.pyc), class variables can be exposed to the user for further customization.

    More information on how users can modify variables of a RoboDK Post Processor here:

        - https://robodk.com/doc/en/Post-Processors.html
        - https://robodk.com/doc/en/Post-Processors.html

    .. code-block:: python

        class RobotPost(object):



            MY_PUBLIC_VAR = TRUE





            MY_PRIVATE_VAR = False

    """

    # Define variables that can be edited by the user below
    #
    # More information on how users can modify variables of a RoboDK Post Processor here:
    #     https://robodk.com/doc/en/Post-Processors.html#PPEditor
    #     https://robodk.com/doc/en/Post-Processors.html#EditPost

    DEFAULT_TCP = [0, 0, 0, 0, 0, 0]  #: Default TCP [Xmm, Ymm, Zmm, RX°, RY°, RZ°]
    DEFAULT_SPEED = 250  #: Default linear speed (mm/s)
    DEFAULT_ACCELERATION = 1200  #: Default linear acceleration (mm/s2)
    DEFAULT_SPEEDJOINTS = 60  #: Default joint speed (deg/s)
    DEFAULT_ACCELERATIONJOINTS = 80  #: Default joint acceleration (deg/s2)
    DEFAULT_ROUNDING = 0  #: Default rounding (mm)
    MOVEJ_POSITION_TYPE = 0  #: Default 0 (0:JOINT_ANGLES 1:CARTESIAN_POSE)
    PROGRAM_CALL_MODE = 0  #: Default 0 (0:Folder 1:SubTask)
    LANGUAGE_SETTING = "zh"  #: Default zh (zh:中文 ja:日本語 en:English)

    # --------------------------------------------------------
    # Define internal variables that cannot be edited by the user below (note the "# ---------..." splitter)

    PROG_EXT = "task"

    ROBOT_POST = ""
    ROBOT_NAME = ""
    NATIVE_NAME = ""

    PROG = []
    PROG_FILES = []
    LOG = ""

    nAxes = 6

    def __init__(self, robotpost="", robotname="", robot_axes=6, **kwargs):
        """Create a new post processor.

        **Tip**: Avoid using instances of :meth:`robodk.robolink.Robolink` and/or :meth:`robodk.robolink.Item` in the post processor unless absolutely necessary.

        :param robotpost: Name of the post processor
        :type robotpost: str
        :param robotname: Name of the robot
        :type robotname: str
        :param robot_axes: Number of axes of the robot
        :type robot_axes: int
        :param axes_type: Type of each axes of the robot ('R': Robot rotative, 'T': Robot translation, 'J': Ext. axis rotative, 'L': Ext. axis linear)
        :type axes_type: list of str, optional
        :param native_name: Native name of the robot (before any rename)
        :type native_name: str, optional
        :param ip_com: IP address of the robot ("Connect" tab in RoboDK)
        :type ip_com: str, optional
        :param api_port: RoboDK API port to the RoboDK instance
        :type api_port: int, optional
        :param prog_ptr: RoboDK Item pointer to the program generated
        :type prog_ptr: int, optional
        :param robot_ptr: RoboDK Item pointer to the robot associated with the program generated
        :type robot_ptr: int, optional
        :param pose_turntable: Pose of the synchronized turn table
        :type pose_turntable: :meth:`robodk.robomath.PosePP`, optional
        :param pose_rail: Pose of the synchronized linear rail
        :type pose_rail: :meth:`robodk.robomath.PosePP`, optional
        :param lines_x_prog: Maximum number of lines per program (to generate multiple files). This setting can be overridden by in RoboDK (Tools-Options-Program)
        :type lines_x_prog: int, optional
        :param pulses_x_deg: Pulses per degree (provided in the robot parameters of RoboDK)
        :type pulses_x_deg: list of int, optional

        Example for a program using the KUKA KRC2 post processor and a KUKA KR 500 3 robot with synchronized linear rail and turn table:

        .. code-block:: python

            RobotPost(r\"\"\"KUKA_KRC2\"\"\",r\"\"\"KUKA KR 500 3\"\"\",9, axes_type=['R','R','R','R','R','R','T','J','J'], native_name=r\"\"\"KUKA KR 500 3\"\"\", ip_com=r\"\"\"192.168.241.127\"\"\", api_port=20500, prog_ptr=2109546424992, robot_ptr=2109347059584, pose_turntable=p(1890.627070,1670.706012,-504.767930,-0.060103,0.046410,-0.246888), pose_rail=p(0.000000,0.000000,0.000000,90.000000,0.000000,0.000000))

        """
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.NATIVE_NAME = robotname
        self.nAxes = robot_axes

        # Initialize internal non-constant variables
        self.PROG = []
        self.PROG_FILES = []
        self.LOG = ""
        self.MAX_LINES_X_PROG = 0

        # Optional arguments, you may or may not use those
        for k, v in kwargs.items():
            if k == "axes_type":
                self.AXES_TYPE = v
            elif k == "native_name":
                self.NATIVE_NAME = v
            elif k == "ip_com":
                self.ROBOT_IP = v
            elif k == "api_port":
                self.API_PORT = v
            elif k == "prog_ptr":
                self.PROG_PTR = v
            elif k == "robot_ptr":
                self.ROBOT_PTR = v
            elif k == "pose_turntable":
                self.POSE_TURNTABLE = v
            elif k == "pose_rail":
                self.POSE_RAIL = v
            elif k == "lines_x_prog":
                self.MAX_LINES_X_PROG = v
            elif k == "pulses_x_deg":
                self.PULSES_X_DEG = v

        self.DEFAULT_ENABLE = True
        self.lines = 0
        self.programs = {}
        self.tcps = {}
        self.frames = {}
        self.analogOutputDomain = ["-1", "-1"]
        self.analogOutputDomainSwitched = [False, False]
        self.current_program = ""
        self.task = ""
        self.language_dict = {
            "zh": {
                "EliTask": "机器人主任务",
                "MainTask": "机器人主任务",
                "Comment": "注释",
                "SetNode": "设置",
                "WaypointNode": "路点",
                "TimerNode": "计时器",
                "WaitNode": "等待",
                "FolderNode": "文件夹",
                "CallSubTaskNode": "调用",
                "subTask": "子任务",
                "PopupNode": "弹出窗口",
                "ArcMotion": "圆弧移动",
                "ViaPoint": "经过点",
                "EndPoint": "终点",
            },
            "ja": {
                "EliTask": "ロボットの主なタスク",
                "MainTask": "ロボットの主なタスク",
                "Comment": "ノート",
                "SetNode": "設定",
                "WaypointNode": "ウェイポイント",
                "TimerNode": "タイマー",
                "WaitNode": "待つ",
                "FolderNode": "フォルダ",
                "CallSubTaskNode": "移行",
                "subTask": "サブタスク",
                "PopupNode": "ポップアップ",
                "ArcMotion": "円弧動き",
                "ViaPoint": "パスポイント",
                "EndPoint": "終わり",
            },
            "en": {
                "EliTask": "Robot Main Task",
                "MainTask": "Robot Main Task",
                "Comment": "Comment",
                "SetNode": "Set",
                "WaypointNode": "Waypoint",
                "TimerNode": "Timer",
                "WaitNode": "Wait",
                "FolderNode": "Folder",
                "CallSubTaskNode": "Call",
                "subTask": "Subtask",
                "PopupNode": "Popup",
                "ArcMotion": "ArcMotion",
                "ViaPoint": "ViaPoint",
                "EndPoint": "EndPoint",
            },
        }

    def ProgStart(self, progname, new_page=False):

        if self.PROGRAM_CALL_MODE == 1:
            self.current_program = get_safe_name(progname, "Program")
            if ProgNames == []:
                ProgNames.append(self.current_program)
        else:
            self.current_program = progname

        self.programs[self.current_program] = []

    def ProgFinish(self, progname, new_page=False):
        pass

    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
        """Saves the program. This method is executed after all programs have been processed.

        **Tip**:
        ProgSave is triggered after all the programs and instructions have been executed.

        :param folder: Folder hint to save the program
        :type folder: str
        :param progname: Program name as a hint to save the program
        :type progname: str
        :param ask_user: True if the default settings in RoboDK are set to prompt the user to select the folder
        :type ask_user: bool, str
        :param show_result: False if the default settings in RoboDK are set to not show the program once it has been saved. Otherwise, a string is provided with the path of the preferred text editor
        :type show_result: bool, str
        """

        if self.PROGRAM_CALL_MODE == 1:
            progname = get_safe_name(progname, "Program")
        taskname = progname + "." + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(
                folder,
                taskname,
                "Save program as...",
                ".task",
                [("Task Files", ".task")],
            )
            if filesave is not None:
                filesave = filesave.name
                folder = getFileDir(filesave)
                filename = getFileName(filesave)
            else:
                return
        else:
            filesave = folder + "/" + taskname
            filename = progname

        self.convert_task(filename, progname)

        if self.MAX_LINES_X_PROG != 0:
            if self.lines > self.MAX_LINES_X_PROG:
                self.addlog("You set the maximum number of lines per program")
                self.addlog("Maximum number of lines:" + str(self.MAX_LINES_X_PROG))
                self.addlog("Current number of lines:" + str(self.lines))
                self.addlog("You should split the script into several files")

        with open(filesave, "w", encoding="utf-8") as fid:
            for index, line in enumerate(self.PROG):
                fid.write(line)
                if index < len(self.PROG) - 1:
                    fid.write("\n")

        print("SAVED: %s\n" % filesave)
        self.PROG_FILES.append(filesave)

        if show_result:
            if type(show_result) is str:

                import subprocess

                p = subprocess.Popen([show_result, filesave])
            else:

                import os

                os.startfile(filesave)

            if len(self.LOG) > 0:
                mbox("Program generation LOG:\n\n" + self.LOG)

        self.PROG = []
        self.convert_configuration()
        configurationname = filename + ".configuration"
        filesave = folder + "/" + configurationname

        with open(filesave, "w", encoding="utf-8") as fid:
            for index, line in enumerate(self.PROG):
                fid.write(line)
                if index < len(self.PROG) - 1:
                    fid.write("\n")

        print("SAVED: %s\n" % filesave)
        self.PROG_FILES.append(filesave)

        if show_result:
            if type(show_result) is str:

                import subprocess

                p = subprocess.Popen([show_result, filesave])
            else:

                import os

                os.startfile(filesave)

    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send the progran to the robot"""
        if remote_path == "" or remote_path == "/":
            remote_path = "/home/elite/user/program/"
        if remote_path[-1] != "/":
            remote_path += "/"

        if ftp_user == "":
            ftp_user = "root"

        if ftp_pass == "":
            ftp_pass = "elibot"

        try:
            import paramiko

            ssh_client = paramiko.SSHClient()
        except ModuleNotFoundError as e:
            ShowMessage(
                "paramiko module not found, please use 'pip install paramiko' to install paramiko module",
                "Module not found",
            )
            return

        def sftp_download(ip, user, pwd, local_file_path, remote_file_path):
            try:
                ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                ssh_client.connect(hostname=ip, port=22, username=user, password=pwd)
                ftp = ssh_client.open_sftp()
                files = ftp.put(local_file_path, remote_file_path)
            except Exception as e:
                ShowMessage(
                    f"{local_file_path}\tsend failed\nRobot IP: {robot_ip}\nFTP path: {remote_path}\nFTP user name: {ftp_user}\nFTP password: {ftp_pass}\n{e}",
                    "Error",
                )
                return False
            else:
                ftp.close()
                ssh_client.close()
                return True

        if isinstance(self.PROG_FILES, list):
            if len(self.PROG_FILES) == 0:
                ShowMessage("Nothing to transfer", "Error")

                return

            result = True
            for file in self.PROG_FILES:

                remote_file_path = remote_path + getBaseName(file)
                if not sftp_download(
                    robot_ip, ftp_user, ftp_pass, file, remote_file_path
                ):
                    result = False
            if result:
                if len(self.PROG_FILES) == 1:
                    ShowMessage(
                        "Done: %i file successfully transferred" % len(self.PROG_FILES),
                        "Success",
                    )
                elif len(self.PROG_FILES) > 1:
                    ShowMessage(
                        "Done: %i files successfully transferred"
                        % len(self.PROG_FILES),
                        "Success",
                    )
            else:
                ShowMessage("Error program files", "Error")
        return

    def MoveJ(self, pose, joints, conf_RLF=None):
        """Defines a joint movement.

        **Tip**:
        MoveJ is triggered by the RoboDK instruction Program->Move Joint Instruction.

        :param pose: Pose target of the tool with respect to the reference frame. Pose can be None if the target is defined as a joint target
        :type pose: :meth:`robodk.robomath.Mat`, None
        :param joints: Robot joints as a list
        :type joints: float list
        :param conf_RLF: Robot configuration as a list of 3 ints: [REAR, LOWER-ARM, FLIP]. [0,0,0] means [front, upper arm and non-flip] configuration. Configuration can be None if the target is defined as a joint target
        :type conf_RLF: int list, None
        """
        if len(joints) != 6:
            msg = "Current not supports synchronize external axes! Try again after removing sync with external axes."
            self.addlog(msg)
            self.RunMessage(msg, True)
            return

        instruction = {"name": "MoveJ", "pose": pose, "joints": joints}
        self.programs[self.current_program].append(instruction)

    def MoveL(self, pose, joints, conf_RLF=None):
        """Defines a linear movement.

        **Tip**:
        MoveL is triggered by the RoboDK instruction Program->Move Linear Instruction.

        :param pose: Pose target of the tool with respect to the reference frame. Pose can be None if the target is defined as a joint target
        :type pose: :meth:`robodk.robomath.Mat`, None
        :param joints: Robot joints as a list
        :type joints: float list
        :param conf_RLF: Robot configuration as a list of 3 ints: [REAR, LOWER-ARM, FLIP]. [0,0,0] means [front, upper arm and non-flip] configuration. Configuration can be None if the target is defined as a joint target
        :type conf_RLF: int list, None
        """
        if len(joints) != 6:
            msg = "Current not supports synchronize external axes! Try again after removing sync with external axes."
            self.addlog(msg)
            self.RunMessage(msg, True)
            return

        if pose is None:
            msg = "Linear movement using joint targets is not supported. Change the target type to cartesian or use a joint movement."
            self.addlog(msg)
            self.RunMessage(msg, True)
            return

        instruction = {"name": "MoveL", "pose": pose, "joints": joints}
        self.programs[self.current_program].append(instruction)

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Defines a circular movement.

        **Tip**:
        MoveC is triggered by the RoboDK instruction Program->Move Circular Instruction.

        :param pose1: Pose target of a point defining an arc (waypoint). Pose can be None if the target is defined as a joint target
        :type pose1: :meth:`robodk.robomath.Mat`, None
        :param pose2: Pose target of the end of the arc (final point). Pose can be None if the target is defined as a joint target
        :type pose2: :meth:`robodk.robomath.Mat`, None
        :param joints1: Robot joints of the waypoint
        :type joints1: float list
        :param joints2: Robot joints of the final point
        :type joints2: float list
        :param conf_RLF_1: Robot configuration of the waypoint as a list of 3 ints: [REAR, LOWER-ARM, FLIP]. [0,0,0] means [front, upper arm and non-flip] configuration. Configuration can be None if the target is defined as a joint target
        :type conf_RLF_1: int list, None
        :param conf_RLF_2: Robot configuration of the final point as a list of 3 ints: [REAR, LOWER-ARM, FLIP]. [0,0,0] means [front, upper arm and non-flip] configuration. Configuration can be None if the target is defined as a joint target
        :type conf_RLF_2: int list, None
        """
        if len(joints1) != 6 or len(joints2) != 6:
            msg = "Current not supports synchronize external axes! Try again after removing sync with external axes."
            self.addlog(msg)
            self.RunMessage(msg, True)
            return

        if pose1 is None or pose2 is None:
            msg = "Circular movement using joint targets is not supported. Change the target type to cartesian or use a joint movement."
            self.addlog(msg)
            self.RunMessage(msg, True)
            return

        instruction = {
            "name": "MoveC",
            "pose1": pose1,
            "joints1": joints1,
            "pose2": pose2,
            "joints2": joints2,
        }
        self.programs[self.current_program].append(instruction)

    def setFrame(self, pose, frame_id, frame_name):
        """Defines a new reference frame with respect to the robot base frame. This reference frame is used for following pose targets used by movement instructions.

        **Tip**:
        setFrame is triggered by the RoboDK instruction Program->Set Reference Frame Instruction.

        :param pose: Pose of the reference frame with respect to the robot base frame
        :type pose: :meth:`robodk.robomath.Mat`
        :param frame_id: Number of the reference frame (if available, else -1)
        :type frame_id: int
        :param frame_name: Name of the reference frame as defined in RoboDK
        :type frame_name: str
        """
        frame_name = get_safe_name(frame_name, "Frame")
        frame_pose = pose_2_list(pose)
        try:
            while frame_pose != self.frames[frame_name]:
                frame_name = name_add_num(frame_name)
                frame_name = get_safe_name(frame_name, "Frame")
        except:
            FrameNames.append(frame_name)
        instruction = {"name": "setFrame", "frame_name": frame_name}
        self.programs[self.current_program].append(instruction)
        self.frames[frame_name] = frame_pose

    def setTool(self, pose, tool_id, tool_name):
        """Change the robot TCP (Tool Center Point) with respect to the robot flange. Any movement defined in Cartesian coordinates assumes that it is using the last reference frame and tool frame provided.

        **Tip**:
        setTool is triggered by the RoboDK instruction Program->Set Tool Frame Instruction.

        :param pose: Pose of the TCP frame with respect to the robot base frame
        :type pose: :meth:`robodk.robomath.Mat`
        :param tool_id: Tool ID (if available, else -1)
        :type tool_id: int, None
        :param tool_name: Name of the tool as defined in RoboDK
        :type tool_name: str
        """
        tool_name = get_safe_name(tool_name, "TCP")
        tool_pose = pose_2_list(pose)
        try:
            while tool_pose != self.tcps[tool_name]:
                tool_name = name_add_num(tool_name)
                tool_name = get_safe_name(tool_name, "TCP")
        except:
            TCPNames.append(tool_name)
        instruction = {"name": "setTool", "tool_name": tool_name}
        self.programs[self.current_program].append(instruction)
        self.tcps[tool_name] = tool_pose

    def Pause(self, time_ms):
        """Pause the robot program"""
        instruction = {"name": "Pause", "time_ms": time_ms}
        self.programs[self.current_program].append(instruction)

    def setSpeed(self, speed_mms):

        instruction = {"name": "setSpeed", "speed_mms": speed_mms}
        self.programs[self.current_program].append(instruction)

    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""

        instruction = {"name": "setAcceleration", "accel_mmss": accel_mmss}
        self.programs[self.current_program].append(instruction)

    def setSpeedJoints(self, speed_degs):

        instruction = {"name": "setSpeedJoints", "speed_degs": speed_degs}
        self.programs[self.current_program].append(instruction)

    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""

        instruction = {"name": "setAccelerationJoints", "accel_degss": accel_degss}
        self.programs[self.current_program].append(instruction)

    def setZoneData(self, zone_mm):

        instruction = {"name": "setZoneData", "zone_mm": zone_mm}
        self.programs[self.current_program].append(instruction)

    def setDO(self, io_var, io_value):
        """Set a Digital Output"""

        instruction = {"name": "setDO", "io_var": io_var, "io_value": io_value}
        self.programs[self.current_program].append(instruction)

    def setAO(self, io_var, io_value):
        """Set an Analog Output"""
        aIO_id = str(io_var)
        aIO_domain = "-1"
        aIO_value = str(io_value)

        if aIO_id == "0" or aIO_id == "analog_out[0]":
            aIO_id = 0
        elif aIO_id == "1" or aIO_id == "analog_out[1]":
            aIO_id = 1
        else:
            self.addlog("Please check the AO Name:")
            self.addlog("The index of the output, integer type data: [0:1]")
            self.addlog("Current AO Name:" + str(io_var))
            return

        if aIO_value[-2:].lower() == "ma":
            aIO_domain = "0"
            try:
                current = float(aIO_value[:-2])
            except:
                self.addlog("Please check the AO Value:")
                self.addlog(
                    "Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0."
                )
                self.addlog("Current AO Value:" + str(io_value))
                return
            else:
                if current < 4 or current > 20:
                    self.addlog("Please check the AO Value:")
                    self.addlog("current mode (4-20mA)")
                    self.addlog("Current AO Value:" + str(io_value))
                    return
                else:
                    aIO_value = str(current)
        elif aIO_value[-1:].lower() == "v":
            aIO_domain = "1"
            try:
                voltage = float(aIO_value[:-1])
            except:
                self.addlog("Please check the AO Value:")
                self.addlog(
                    "Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0."
                )
                self.addlog("Current AO Value:" + str(io_value))
                return
            else:
                if voltage < 0 or voltage > 10:
                    self.addlog("Please check the AO Value:")
                    self.addlog("voltage mode (0-10V)")
                    self.addlog("Current AO Value:" + str(io_value))
                    return
                else:
                    aIO_value = str(voltage)
        else:
            try:
                if float(aIO_value) < 0:
                    aIO_value = "0"
                elif float(aIO_value) > 1:
                    aIO_value = "1"
                else:
                    aIO_value = str(aIO_value)
            except:
                self.addlog("Please check the AO Value:")
                self.addlog(
                    "Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0."
                )
                self.addlog("Current AO Value:" + str(io_value))
                return

        if aIO_domain != "-1":
            if self.analogOutputDomain[aIO_id] == "-1":
                self.analogOutputDomain[aIO_id] = aIO_domain
            elif self.analogOutputDomain[aIO_id] != aIO_domain:
                self.analogOutputDomainSwitched[aIO_id] = True

        instruction = {"name": "setAO", "io_var": io_var, "io_value": io_value}
        self.programs[self.current_program].append(instruction)

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""

        instruction = {
            "name": "waitDI",
            "io_var": io_var,
            "io_value": io_value,
            "timeout_ms": timeout_ms,
        }
        self.programs[self.current_program].append(instruction)

    def RunCode(self, code, is_function_call=False):
        """Adds code or a function call"""
        if self.PROGRAM_CALL_MODE == 1:
            if self.current_program != ProgNames[0]:
                self.addlog(
                    "SubTask not support call another subtask, please use folder mode and generate robot program again!"
                )
                return
            code = get_safe_name(code, "Program")
            instruction = {
                "name": "RunCode",
                "code": code,
                "is_function_call": is_function_call,
                "is_first_call": code not in ProgNames,
            }
            if code not in ProgNames:
                ProgNames.append(code)
        else:
            instruction = {
                "name": "RunCode",
                "code": code,
                "is_function_call": is_function_call,
            }
        self.programs[self.current_program].append(instruction)

    def RunMessage(self, message, iscomment=False):
        """Add a comment or a popup message"""

        instruction = {"name": "RunMessage", "message": message, "iscomment": iscomment}
        self.programs[self.current_program].append(instruction)

    # ------------------ private ----------------------
    def addline(self, newline):
        """Add a new program line. This is a private method used only by the other methods.

        :param newline: New program line to add
        :type newline: str
        """
        self.PROG.append(newline)
        self.lines += 1

    def addlog(self, newline):
        """Add a message to the log. This is a private method used only by the other methods. The log is displayed when the program is generated to show any issues when the robot program has been generated.

        :param newline: New log line to add
        :type newline: str
        """
        self.LOG = self.LOG + newline + "\n"

    def convert_program(self, progname):

        if self.DEFAULT_ENABLE:
            global current_tcp, current_frame, current_speed, current_acceleration, current_speedjoints, current_accelerationjoints, current_rounding, current_waypoint, current_movec
            current_tcp = ""
            current_frame = ""
            current_speed = str(self.DEFAULT_SPEED / 1000)
            current_acceleration = str(self.DEFAULT_ACCELERATION / 1000)
            current_speedjoints = str(math.radians(self.DEFAULT_SPEEDJOINTS))
            current_accelerationjoints = str(
                math.radians(self.DEFAULT_ACCELERATIONJOINTS)
            )
            current_rounding = self.DEFAULT_ROUNDING / 1000
            current_waypoint = 1
            current_movec = 0
            self.DEFAULT_ENABLE = False
        for instruction in self.programs[progname]:

            if instruction["name"] == "MoveJ":
                movej = """
<MoveNode movementType="JOINT_MOVEMENT" positionType="JOINT_ANGLES" tcpType="ACTIVE_TCP" typeName="MoveJ">
    <frameReference>Ctrl_base_frame</frameReference>
    <jointSpeed>
    <valueInSi>1.0471975511965976</valueInSi>
    <siUnit class="cn.elibot.robot.plugin.domain.value.AngularSpeed$Unit">RAD_S</siUnit>
    <xmlElementName>AngularSpeed</xmlElementName>
    <supportType>cn.elibot.robot.plugin.core.domain.value.AngularSpeedImpl</supportType>
    </jointSpeed>
    <jointAcceleration>
    <valueInSi>1.3962634015954636</valueInSi>
    <siUnit class="cn.elibot.robot.plugin.domain.value.AngularAcceleration$Unit">RAD_S2</siUnit>
    <xmlElementName>AngularAcceleration</xmlElementName>
    <supportType>cn.elibot.robot.plugin.core.domain.value.AngularAccelerationImpl</supportType>
    </jointAcceleration>
    <WaypointNode name="路点_1" positionNodeType="FIXED_POSITION" kinematicFlag="-1" customTransitionRadius="false" advancedPositionOptionType="SPEED_AND_ACCELERATION" typeName="路点">
    <transitionRadius>
        <valueInSi>0.0</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Length$Unit">M</siUnit>
        <xmlElementName>Length</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.LengthImpl</supportType>
    </transitionRadius>
    <internalPosition>
        <jointPositions joints="0.0, -1.5707963705062866, 0.0, -1.5707963705062866, 1.5707963705062866, 0.0"/>
        <toolPose X="92.00004434917304" Y="-147.49998942204118" Z="1076.499983076011" RX="-1.5707964142176767" RY="3.821371231789832E-15" RZ="-1.5707962830835067"/>
    </internalPosition>
    <fromPosition>
        <jointPositions joints="NaN, NaN, NaN, NaN, NaN, NaN"/>
        <toolPose X="0.0" Y="0.0" Z="0.0" RX="0.0" RY="0.0" RZ="0.0"/>
    </fromPosition>
    <jointSpeed>
        <valueInSi>1.0471975511965976</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.AngularSpeed$Unit">RAD_S</siUnit>
        <xmlElementName>AngularSpeed</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AngularSpeedImpl</supportType>
    </jointSpeed>
    <jointAcceleration>
        <valueInSi>1.3962634015954636</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.AngularAcceleration$Unit">RAD_S2</siUnit>
        <xmlElementName>AngularAcceleration</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AngularAccelerationImpl</supportType>
    </jointAcceleration>
    <cartesianSpeed>
        <valueInSi>0.25</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Speed$Unit">M_S</siUnit>
        <xmlElementName>Speed</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.SpeedImpl</supportType>
    </cartesianSpeed>
    <cartesianAcceleration>
        <valueInSi>1.2</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Acceleration$Unit">M_S2</siUnit>
        <xmlElementName>Acceleration</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AccelerationImpl</supportType>
    </cartesianAcceleration>
    <nextMotionTime>
        <valueInSi>2.0</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Time$Unit">S</siUnit>
        <xmlElementName>Time</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.TimeImpl</supportType>
    </nextMotionTime>
    <Pose key="baseToFrame">
        <Position key="posePosition">
        <Length key="X" value="0.0" unit="M"/>
        <Length key="Y" value="0.0" unit="M"/>
        <Length key="Z" value="0.0" unit="M"/>
        </Position>
        <Rotation key="poseRotation">
        <Angle key="RX" value="0.0" unit="RAD"/>
        <Angle key="RY" value="0.0" unit="RAD"/>
        <Angle key="RZ" value="0.0" unit="RAD"/>
        </Rotation>
    </Pose>
    </WaypointNode>
</MoveNode>
                """
                root = ET.fromstring(movej)
                if self.MOVEJ_POSITION_TYPE == 1:
                    root.set("positionType", "CARTESIAN_POSE")
                if current_tcp != "":
                    root.set("tcpType", "USER_DEFINED_TCP")
                    new = ET.Element("tcpToolNameReference")
                    new.text = current_tcp
                    root.append(new)
                if current_frame != "":
                    root.find("frameReference").text = current_frame

                root.find("jointSpeed").find("valueInSi").text = current_speedjoints
                root.find("jointAcceleration").find(
                    "valueInSi"
                ).text = current_accelerationjoints
                root.find("WaypointNode").set("name", "路点_" + str(current_waypoint))
                current_waypoint += 1
                if current_rounding > 0:
                    root.find("WaypointNode").set("customTransitionRadius", "true")
                    root.find("WaypointNode").find("transitionRadius").find(
                        "valueInSi"
                    ).text = str(current_rounding)
                root.find("WaypointNode").find("internalPosition").find(
                    "jointPositions"
                ).set(
                    "joints",
                    ", ".join([str(x) for x in joints_2_list(instruction["joints"])]),
                )
                if instruction["pose"] is not None:
                    root.find("WaypointNode").find("internalPosition").find(
                        "toolPose"
                    ).set("X", str(pose_2_list(instruction["pose"])[0]))
                    root.find("WaypointNode").find("internalPosition").find(
                        "toolPose"
                    ).set("Y", str(pose_2_list(instruction["pose"])[1]))
                    root.find("WaypointNode").find("internalPosition").find(
                        "toolPose"
                    ).set("Z", str(pose_2_list(instruction["pose"])[2]))
                    root.find("WaypointNode").find("internalPosition").find(
                        "toolPose"
                    ).set("RX", str(pose_2_list(instruction["pose"])[3]))
                    root.find("WaypointNode").find("internalPosition").find(
                        "toolPose"
                    ).set("RY", str(pose_2_list(instruction["pose"])[4]))
                    root.find("WaypointNode").find("internalPosition").find(
                        "toolPose"
                    ).set("RZ", str(pose_2_list(instruction["pose"])[5]))
                root.find("WaypointNode").find("jointSpeed").find(
                    "valueInSi"
                ).text = current_speedjoints
                root.find("WaypointNode").find("jointAcceleration").find(
                    "valueInSi"
                ).text = current_accelerationjoints
                root.find("WaypointNode").find("cartesianSpeed").find(
                    "valueInSi"
                ).text = current_speed
                root.find("WaypointNode").find("cartesianAcceleration").find(
                    "valueInSi"
                ).text = current_acceleration
                self.task += ET.tostring(root, encoding="UTF-8").decode()

            elif instruction["name"] == "MoveL":
                movel = """
<MoveNode movementType="LINEAR_MOVEMENT" positionType="CARTESIAN_POSE" tcpType="ACTIVE_TCP" typeName="MoveL">
    <frameReference>Ctrl_base_frame</frameReference>
    <cartesianSpeed>
    <valueInSi>0.25</valueInSi>
    <siUnit class="cn.elibot.robot.plugin.domain.value.Speed$Unit">M_S</siUnit>
    <xmlElementName>Speed</xmlElementName>
    <supportType>cn.elibot.robot.plugin.core.domain.value.SpeedImpl</supportType>
    </cartesianSpeed>
    <cartesianAcceleration>
    <valueInSi>1.2</valueInSi>
    <siUnit class="cn.elibot.robot.plugin.domain.value.Acceleration$Unit">M_S2</siUnit>
    <xmlElementName>Acceleration</xmlElementName>
    <supportType>cn.elibot.robot.plugin.core.domain.value.AccelerationImpl</supportType>
    </cartesianAcceleration>
    <WaypointNode name="路点_1" positionNodeType="FIXED_POSITION" kinematicFlag="-1" customTransitionRadius="false" advancedPositionOptionType="SPEED_AND_ACCELERATION" typeName="路点">
    <transitionRadius>
        <valueInSi>0.0</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Length$Unit">M</siUnit>
        <xmlElementName>Length</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.LengthImpl</supportType>
    </transitionRadius>
    <internalPosition>
        <jointPositions joints="0.0, -1.5707963705062866, 0.0, -1.5707963705062866, 1.5707963705062866, 0.0"/>
        <toolPose X="93.00004430546164" Y="-150.4999893783298" Z="1074.4999829885883" RX="-0.38759674783838294" RY="0.36136705782129974" RZ="-2.4278682525651645"/>
    </internalPosition>
    <fromPosition>
        <jointPositions joints="NaN, NaN, NaN, NaN, NaN, NaN"/>
        <toolPose X="0.0" Y="0.0" Z="0.0" RX="0.0" RY="0.0" RZ="0.0"/>
    </fromPosition>
    <jointSpeed>
        <valueInSi>1.0471975511965976</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.AngularSpeed$Unit">RAD_S</siUnit>
        <xmlElementName>AngularSpeed</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AngularSpeedImpl</supportType>
    </jointSpeed>
    <jointAcceleration>
        <valueInSi>1.3962634015954636</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.AngularAcceleration$Unit">RAD_S2</siUnit>
        <xmlElementName>AngularAcceleration</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AngularAccelerationImpl</supportType>
    </jointAcceleration>
    <cartesianSpeed>
        <valueInSi>0.25</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Speed$Unit">M_S</siUnit>
        <xmlElementName>Speed</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.SpeedImpl</supportType>
    </cartesianSpeed>
    <cartesianAcceleration>
        <valueInSi>1.2</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Acceleration$Unit">M_S2</siUnit>
        <xmlElementName>Acceleration</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AccelerationImpl</supportType>
    </cartesianAcceleration>
    <nextMotionTime>
        <valueInSi>2.0</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Time$Unit">S</siUnit>
        <xmlElementName>Time</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.TimeImpl</supportType>
    </nextMotionTime>
    <Pose key="baseToFrame">
        <Position key="posePosition">
        <Length key="X" value="0.0" unit="M"/>
        <Length key="Y" value="0.0" unit="M"/>
        <Length key="Z" value="0.0" unit="M"/>
        </Position>
        <Rotation key="poseRotation">
        <Angle key="RX" value="0.0" unit="RAD"/>
        <Angle key="RY" value="0.0" unit="RAD"/>
        <Angle key="RZ" value="0.0" unit="RAD"/>
        </Rotation>
    </Pose>
    </WaypointNode>
</MoveNode>
                """
                root = ET.fromstring(movel)
                if current_tcp != "":
                    root.set("tcpType", "USER_DEFINED_TCP")
                    new = ET.Element("tcpToolNameReference")
                    new.text = current_tcp
                    root.append(new)
                if current_frame != "":
                    root.find("frameReference").text = current_frame

                root.find("cartesianSpeed").find("valueInSi").text = current_speed
                root.find("cartesianAcceleration").find(
                    "valueInSi"
                ).text = current_acceleration
                root.find("WaypointNode").set("name", "路点_" + str(current_waypoint))
                current_waypoint += 1
                if current_rounding > 0:
                    root.find("WaypointNode").set("customTransitionRadius", "true")
                    root.find("WaypointNode").find("transitionRadius").find(
                        "valueInSi"
                    ).text = str(current_rounding)
                root.find("WaypointNode").find("internalPosition").find(
                    "jointPositions"
                ).set(
                    "joints",
                    ", ".join([str(x) for x in joints_2_list(instruction["joints"])]),
                )
                root.find("WaypointNode").find("internalPosition").find("toolPose").set(
                    "X", str(pose_2_list(instruction["pose"])[0])
                )
                root.find("WaypointNode").find("internalPosition").find("toolPose").set(
                    "Y", str(pose_2_list(instruction["pose"])[1])
                )
                root.find("WaypointNode").find("internalPosition").find("toolPose").set(
                    "Z", str(pose_2_list(instruction["pose"])[2])
                )
                root.find("WaypointNode").find("internalPosition").find("toolPose").set(
                    "RX", str(pose_2_list(instruction["pose"])[3])
                )
                root.find("WaypointNode").find("internalPosition").find("toolPose").set(
                    "RY", str(pose_2_list(instruction["pose"])[4])
                )
                root.find("WaypointNode").find("internalPosition").find("toolPose").set(
                    "RZ", str(pose_2_list(instruction["pose"])[5])
                )
                root.find("WaypointNode").find("jointSpeed").find(
                    "valueInSi"
                ).text = current_speedjoints
                root.find("WaypointNode").find("jointAcceleration").find(
                    "valueInSi"
                ).text = current_accelerationjoints
                root.find("WaypointNode").find("cartesianSpeed").find(
                    "valueInSi"
                ).text = current_speed
                root.find("WaypointNode").find("cartesianAcceleration").find(
                    "valueInSi"
                ).text = current_acceleration
                self.task += ET.tostring(root, encoding="UTF-8").decode()

            elif instruction["name"] == "MoveC":
                movec = """
<MoveNode movementType="PROCESS_MOVEMENT" positionType="CARTESIAN_POSE" tcpType="ACTIVE_TCP" typeName="MoveP">
    <frameReference>Ctrl_base_frame</frameReference>
    <cartesianSpeed>
    <valueInSi>0.25</valueInSi>
    <siUnit class="cn.elibot.robot.plugin.domain.value.Speed$Unit">M_S</siUnit>
    <xmlElementName>Speed</xmlElementName>
    <supportType>cn.elibot.robot.plugin.core.domain.value.SpeedImpl</supportType>
    </cartesianSpeed>
    <cartesianAcceleration>
    <valueInSi>1.2</valueInSi>
    <siUnit class="cn.elibot.robot.plugin.domain.value.Acceleration$Unit">M_S2</siUnit>
    <xmlElementName>Acceleration</xmlElementName>
    <supportType>cn.elibot.robot.plugin.core.domain.value.AccelerationImpl</supportType>
    </cartesianAcceleration>
    <inheritedProcessMoveTransitionRadius>
    <valueInSi>0.025</valueInSi>
    <siUnit class="cn.elibot.robot.plugin.domain.value.Length$Unit">M</siUnit>
    <xmlElementName>Length</xmlElementName>
    <supportType>cn.elibot.robot.plugin.core.domain.value.LengthImpl</supportType>
    </inheritedProcessMoveTransitionRadius>
    <ArcMotion orientationMode="UNCONSTRAINED" typeName="圆弧移动">
    <WaypointNode name="经过点" positionNodeType="FIXED_POSITION" kinematicFlag="-1" customTransitionRadius="false" advancedPositionOptionType="INHERITED_PARAMETERS" typeName="路点">
        <transitionRadius reference="../../../inheritedProcessMoveTransitionRadius"/>
        <internalPosition>
        <jointPositions joints="0.4878447682976299, -1.6778871049300503, -2.0085866809384, -1.025919661125958, 1.5708031040296353, 0.4878505001398193"/>
        <toolPose X="489.68927316992006" Y="92.86216050191292" Z="292.65080297815695" RX="3.1415855313626815" RY="3.892992942432792E-6" RZ="-1.5708020586660851"/>
        </internalPosition>
        <fromPosition>
        <jointPositions joints="NaN, NaN, NaN, NaN, NaN, NaN"/>
        <toolPose X="0.0" Y="0.0" Z="0.0" RX="0.0" RY="0.0" RZ="0.0"/>
        </fromPosition>
        <jointSpeed>
        <valueInSi>1.0471975511965976</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.AngularSpeed$Unit">RAD_S</siUnit>
        <xmlElementName>AngularSpeed</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AngularSpeedImpl</supportType>
        </jointSpeed>
        <jointAcceleration>
        <valueInSi>1.3962634015954636</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.AngularAcceleration$Unit">RAD_S2</siUnit>
        <xmlElementName>AngularAcceleration</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AngularAccelerationImpl</supportType>
        </jointAcceleration>
        <cartesianSpeed>
        <valueInSi>0.25</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Speed$Unit">M_S</siUnit>
        <xmlElementName>Speed</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.SpeedImpl</supportType>
        </cartesianSpeed>
        <cartesianAcceleration>
        <valueInSi>1.2</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Acceleration$Unit">M_S2</siUnit>
        <xmlElementName>Acceleration</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AccelerationImpl</supportType>
        </cartesianAcceleration>
        <nextMotionTime>
        <valueInSi>2.0</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Time$Unit">S</siUnit>
        <xmlElementName>Time</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.TimeImpl</supportType>
        </nextMotionTime>
        <Pose key="baseToFrame">
        <Position key="posePosition">
            <Length key="X" value="0.0" unit="M"/>
            <Length key="Y" value="0.0" unit="M"/>
            <Length key="Z" value="0.0" unit="M"/>
        </Position>
        <Rotation key="poseRotation">
            <Angle key="RX" value="0.0" unit="RAD"/>
            <Angle key="RY" value="0.0" unit="RAD"/>
            <Angle key="RZ" value="0.0" unit="RAD"/>
        </Rotation>
        </Pose>
    </WaypointNode>
    <WaypointNode name="终点" positionNodeType="FIXED_POSITION" kinematicFlag="-1" customTransitionRadius="false" advancedPositionOptionType="SPEED_AND_ACCELERATION" typeName="路点">
        <transitionRadius>
        <valueInSi>0.0</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Length$Unit">M</siUnit>
        <xmlElementName>Length</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.LengthImpl</supportType>
        </transitionRadius>
        <internalPosition>
        <jointPositions joints="0.008961039651251878, -1.57517772389714, -1.6162278088401574, -1.5209884982956305, 1.5708031040296353, 0.008962319577177672"/>
        <toolPose X="489.68814000986947" Y="-143.1170599541338" Z="478.05150783559935" RX="3.1415875424054653" RY="6.731697638312738E-6" RZ="-1.5707976067551406"/>
        </internalPosition>
        <fromPosition>
        <jointPositions joints="NaN, NaN, NaN, NaN, NaN, NaN"/>
        <toolPose X="0.0" Y="0.0" Z="0.0" RX="0.0" RY="0.0" RZ="0.0"/>
        </fromPosition>
        <jointSpeed>
        <valueInSi>1.0471975511965976</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.AngularSpeed$Unit">RAD_S</siUnit>
        <xmlElementName>AngularSpeed</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AngularSpeedImpl</supportType>
        </jointSpeed>
        <jointAcceleration>
        <valueInSi>1.3962634015954636</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.AngularAcceleration$Unit">RAD_S2</siUnit>
        <xmlElementName>AngularAcceleration</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AngularAccelerationImpl</supportType>
        </jointAcceleration>
        <cartesianSpeed>
        <valueInSi>0.25</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Speed$Unit">M_S</siUnit>
        <xmlElementName>Speed</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.SpeedImpl</supportType>
        </cartesianSpeed>
        <cartesianAcceleration>
        <valueInSi>1.2</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Acceleration$Unit">M_S2</siUnit>
        <xmlElementName>Acceleration</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.AccelerationImpl</supportType>
        </cartesianAcceleration>
        <nextMotionTime>
        <valueInSi>2.0</valueInSi>
        <siUnit class="cn.elibot.robot.plugin.domain.value.Time$Unit">S</siUnit>
        <xmlElementName>Time</xmlElementName>
        <supportType>cn.elibot.robot.plugin.core.domain.value.TimeImpl</supportType>
        </nextMotionTime>
        <Pose key="baseToFrame">
        <Position key="posePosition">
            <Length key="X" value="0.0" unit="M"/>
            <Length key="Y" value="0.0" unit="M"/>
            <Length key="Z" value="0.0" unit="M"/>
        </Position>
        <Rotation key="poseRotation">
            <Angle key="RX" value="0.0" unit="RAD"/>
            <Angle key="RY" value="0.0" unit="RAD"/>
            <Angle key="RZ" value="0.0" unit="RAD"/>
        </Rotation>
        </Pose>
    </WaypointNode>
    </ArcMotion>
</MoveNode>
                """
                root = ET.fromstring(movec)
                if current_tcp != "":
                    root.set("tcpType", "USER_DEFINED_TCP")
                    new = ET.Element("tcpToolNameReference")
                    new.text = current_tcp
                    root.append(new)
                if current_frame != "":
                    root.find("frameReference").text = current_frame

                root.find("cartesianSpeed").find("valueInSi").text = current_speed
                root.find("cartesianAcceleration").find(
                    "valueInSi"
                ).text = current_acceleration
                if current_rounding < 0.001:
                    root.find("inheritedProcessMoveTransitionRadius").find(
                        "valueInSi"
                    ).text = "0.001"
                else:
                    root.find("inheritedProcessMoveTransitionRadius").find(
                        "valueInSi"
                    ).text = str(current_rounding)
                for waypointnode in root.find("ArcMotion").findall("WaypointNode"):
                    if waypointnode.get("name") == "经过点":
                        if current_movec > 0:
                            waypointnode.set("name", "经过点_" + str(current_movec))
                        waypointnode.find("internalPosition").find(
                            "jointPositions"
                        ).set(
                            "joints",
                            ", ".join(
                                [str(x) for x in joints_2_list(instruction["joints1"])]
                            ),
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "X", str(pose_2_list(instruction["pose1"])[0])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "Y", str(pose_2_list(instruction["pose1"])[1])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "Z", str(pose_2_list(instruction["pose1"])[2])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "RX", str(pose_2_list(instruction["pose1"])[3])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "RY", str(pose_2_list(instruction["pose1"])[4])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "RZ", str(pose_2_list(instruction["pose1"])[5])
                        )
                        waypointnode.find("jointSpeed").find(
                            "valueInSi"
                        ).text = current_speedjoints
                        waypointnode.find("jointAcceleration").find(
                            "valueInSi"
                        ).text = current_accelerationjoints
                        waypointnode.find("cartesianSpeed").find(
                            "valueInSi"
                        ).text = current_speed
                        waypointnode.find("cartesianAcceleration").find(
                            "valueInSi"
                        ).text = current_acceleration
                    if waypointnode.get("name") == "终点":
                        if current_movec > 0:
                            waypointnode.set("name", "终点_" + str(current_movec))
                        if current_rounding > 0:
                            waypointnode.find("transitionRadius").find(
                                "valueInSi"
                            ).text = str(current_rounding)
                        waypointnode.find("internalPosition").find(
                            "jointPositions"
                        ).set(
                            "joints",
                            ", ".join(
                                [str(x) for x in joints_2_list(instruction["joints2"])]
                            ),
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "X", str(pose_2_list(instruction["pose2"])[0])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "Y", str(pose_2_list(instruction["pose2"])[1])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "Z", str(pose_2_list(instruction["pose2"])[2])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "RX", str(pose_2_list(instruction["pose2"])[3])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "RY", str(pose_2_list(instruction["pose2"])[4])
                        )
                        waypointnode.find("internalPosition").find("toolPose").set(
                            "RZ", str(pose_2_list(instruction["pose2"])[5])
                        )
                        waypointnode.find("jointSpeed").find(
                            "valueInSi"
                        ).text = current_speedjoints
                        waypointnode.find("jointAcceleration").find(
                            "valueInSi"
                        ).text = current_accelerationjoints
                        waypointnode.find("cartesianSpeed").find(
                            "valueInSi"
                        ).text = current_speed
                        waypointnode.find("cartesianAcceleration").find(
                            "valueInSi"
                        ).text = current_acceleration
                current_movec += 1
                self.task += ET.tostring(root, encoding="UTF-8").decode()

            elif instruction["name"] == "setFrame":
                current_frame = instruction["frame_name"]

            elif instruction["name"] == "setTool":
                current_tcp = instruction["tool_name"]

            elif instruction["name"] == "Pause":
                pause = """
<WaitNode type="WAIT_TIME" typeName="等待">
  <waitTime>
    <valueInSi>0.01</valueInSi>
    <siUnit class="cn.elibot.robot.plugin.domain.value.Time$Unit">S</siUnit>
    <xmlElementName>Time</xmlElementName>
    <supportType>cn.elibot.robot.plugin.core.domain.value.TimeImpl</supportType>
  </waitTime>
</WaitNode>
                """
                root = ET.fromstring(pause)
                root.find("waitTime").find("valueInSi").text = str(
                    instruction["time_ms"] / 1000
                )
                self.task += ET.tostring(root, encoding="UTF-8").decode()

            elif instruction["name"] == "setSpeed":
                current_speed = str(instruction["speed_mms"] / 1000)

            elif instruction["name"] == "setAcceleration":
                current_acceleration = str(instruction["accel_mmss"] / 1000)

            elif instruction["name"] == "setSpeedJoints":
                current_speedjoints = str(math.radians(instruction["speed_degs"]))

            elif instruction["name"] == "setAccelerationJoints":
                current_accelerationjoints = str(
                    math.radians(instruction["accel_degss"])
                )

            elif instruction["name"] == "setZoneData":
                current_rounding = instruction["zone_mm"] / 1000

            elif instruction["name"] == "setDO":
                dIO_id = str(instruction["io_var"])
                dIO_value = str(instruction["io_value"])

                index = [
                    "0",
                    "1",
                    "2",
                    "3",
                    "4",
                    "5",
                    "6",
                    "7",
                    "8",
                    "9",
                    "10",
                    "11",
                    "12",
                    "13",
                    "14",
                    "15",
                ]
                if dIO_id not in index:
                    self.addlog("Please check the DO Name:")
                    self.addlog("The index of the output, integer type data: [0:15]")
                    self.addlog("Current DO Name:" + str(instruction["io_var"]))
                    return

                if dIO_value == "1" or dIO_value == "True":
                    dIO_value = "HIGH"
                elif dIO_value == "0" or dIO_value == "False":
                    dIO_value = "LOW"
                else:
                    self.addlog("Please check the IO Value:")
                    self.addlog("Signal level, boolean type data, True or False")
                    self.addlog("Current IO Value:" + str(instruction["io_value"]))
                    return
                setdo = """
<SetNode type="DIGITAL_OUTPUT" typeName="设置">
    <digitalOutputPinName>digital_out[0]</digitalOutputPinName>
    <digitalOutputPinValue>LOW</digitalOutputPinValue>
</SetNode>
                """
                root = ET.fromstring(setdo)
                root.find("digitalOutputPinName").text = "digital_out[" + dIO_id + "]"
                root.find("digitalOutputPinValue").text = dIO_value
                self.task += ET.tostring(root, encoding="UTF-8").decode()

            elif instruction["name"] == "setAO":
                aIO_id = str(instruction["io_var"])
                aIO_domain = "-1"
                aIO_value = str(instruction["io_value"])

                if aIO_id == "0" or aIO_id == "analog_out[0]":
                    aIO_id = 0
                elif aIO_id == "1" or aIO_id == "analog_out[1]":
                    aIO_id = 1
                else:
                    self.addlog("Please check the AO Name:")
                    self.addlog("The index of the output, integer type data: [0:1]")
                    self.addlog("Current AO Name:" + str(instruction["io_var"]))
                    return

                if aIO_value[-2:].lower() == "ma":
                    aIO_domain = "0"
                    try:
                        current = float(aIO_value[:-2])
                    except:
                        self.addlog("Please check the AO Value:")
                        self.addlog(
                            "Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0."
                        )
                        self.addlog("Current AO Value:" + str(instruction["io_value"]))
                        return
                    else:
                        if current < 4 or current > 20:
                            self.addlog("Please check the AO Value:")
                            self.addlog("current mode (4-20mA)")
                            self.addlog(
                                "Current AO Value:" + str(instruction["io_value"])
                            )
                            return
                        else:
                            aIO_value = str((current - 4) / 16)
                elif aIO_value[-1:].lower() == "v":
                    aIO_domain = "1"
                    try:
                        voltage = float(aIO_value[:-1])
                    except:
                        self.addlog("Please check the AO Value:")
                        self.addlog(
                            "Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0."
                        )
                        self.addlog("Current AO Value:" + str(instruction["io_value"]))
                        return
                    else:
                        if voltage < 0 or voltage > 10:
                            self.addlog("Please check the AO Value:")
                            self.addlog("voltage mode (0-10V)")
                            self.addlog(
                                "Current AO Value:" + str(instruction["io_value"])
                            )
                            return
                        else:
                            aIO_value = str(voltage / 10)
                else:
                    try:
                        if float(aIO_value) < 0:
                            aIO_value = "0"
                        elif float(aIO_value) > 1:
                            aIO_value = "1"
                        else:
                            aIO_value = str(aIO_value)
                    except:
                        self.addlog("Please check the AO Value:")
                        self.addlog(
                            "Relative to the analog signal level (percentage of analog output range), float type data: [0:1], if the given value is greater than 1, then set to 1, less than 0, set to 0."
                        )
                        self.addlog("Current AO Value:" + str(instruction["io_value"]))
                        return
                if aIO_domain == "-1":
                    setao = """
<SetNode type="EXPRESSION_OUTPUT" typeName="设置">
  <expressionOutputPinName>analog_out[0]</expressionOutputPinName>
  <Expression>
  </Expression>
</SetNode>
                    """
                    root = ET.fromstring(setao)
                    root.find("expressionOutputPinName").text = (
                        "analog_out[" + str(aIO_id) + "]"
                    )
                    for char in aIO_value:
                        new = ET.Element("ExpressionCharCell", attrib={"char": char})
                        root.find("Expression").append(new)
                    self.task += ET.tostring(root, encoding="UTF-8").decode()
                elif self.analogOutputDomainSwitched[aIO_id]:
                    setao = """
<ScriptNode scriptType="LINE_TYPE" typeName="脚本">
  <Expression>
  </Expression>
</ScriptNode>
                    """
                    root = ET.fromstring(setao)
                    for char in (
                        "set_standard_analog_output_domain("
                        + str(aIO_id)
                        + ","
                        + aIO_domain
                        + ")"
                    ):
                        new = ET.Element("ExpressionCharCell", attrib={"char": char})
                        root.find("Expression").append(new)
                    self.task += ET.tostring(root, encoding="UTF-8").decode()

                    setao = """
<SetNode type="EXPRESSION_OUTPUT" typeName="设置">
  <expressionOutputPinName>analog_out[0]</expressionOutputPinName>
  <Expression>
  </Expression>
</SetNode>
                    """
                    root = ET.fromstring(setao)
                    root.find("expressionOutputPinName").text = (
                        "analog_out[" + str(aIO_id) + "]"
                    )
                    for char in aIO_value:
                        new = ET.Element("ExpressionCharCell", attrib={"char": char})
                        root.find("Expression").append(new)
                    self.task += ET.tostring(root, encoding="UTF-8").decode()
                else:
                    setao = """
<SetNode type="ANALOG_OUTPUT" typeName="设置">
    <analogOutputPinName>analog_out[0]</analogOutputPinName>
    <analogOutputPinValue>4.0</analogOutputPinValue>
</SetNode>
                    """
                    root = ET.fromstring(setao)
                    root.find("analogOutputPinName").text = (
                        "analog_out[" + str(aIO_id) + "]"
                    )
                    if aIO_domain == "0":
                        root.find("analogOutputPinValue").text = str(current)
                    else:
                        root.find("analogOutputPinValue").text = str(voltage)
                    self.task += ET.tostring(root, encoding="UTF-8").decode()

            elif instruction["name"] == "waitDI":
                dIO_id = str(instruction["io_var"])
                dIO_value = str(instruction["io_value"])

                index = [
                    "0",
                    "1",
                    "2",
                    "3",
                    "4",
                    "5",
                    "6",
                    "7",
                    "8",
                    "9",
                    "10",
                    "11",
                    "12",
                    "13",
                    "14",
                    "15",
                ]
                if dIO_id not in index:
                    self.addlog("Please check the DI Name:")
                    self.addlog("The index of the input, integer type data: [0:15]")
                    self.addlog("Current DI Name:" + str(instruction["io_var"]))
                    return

                if dIO_value == "1" or dIO_value == "True":
                    dIO_value = "HIGH"
                elif dIO_value == "0" or dIO_value == "False":
                    dIO_value = "LOW"
                else:
                    self.addlog("Please check the DI Value:")
                    self.addlog("Signal level, boolean type data, True or False")
                    self.addlog("Current DI Value:" + str(instruction["io_value"]))
                    return
                if instruction["timeout_ms"] == -1:
                    waitdi = """
<WaitNode type="WAIT_DIGITAL_INPUT" typeName="等待">
    <digitalInputType>digital_in[0]</digitalInputType>
    <digitalValue>LOW</digitalValue>
</WaitNode>
                    """
                    root = ET.fromstring(waitdi)
                    root.find("digitalInputType").text = "digital_in[" + dIO_id + "]"
                    root.find("digitalValue").text = dIO_value
                    self.task += ET.tostring(root, encoding="UTF-8").decode()
                else:
                    waitdi = """
<TimerNode action="RESET" typeName="计时器">
    <variable name="计时器_1" prefersPersistentValue="false"/>
</TimerNode>
<TimerNode action="START" typeName="计时器">
    <variable reference="../../TimerNode/variable"/>
</TimerNode>
                    """
                    self.task += waitdi
                    waitdi = """
<WaitNode type="EXPRESSION" typeName="等待">
  <Expression>
  </Expression>
</WaitNode>
                    """
                    root = ET.fromstring(waitdi)
                    for char in "get_standard_digital_in(" + dIO_id + ")":
                        new = ET.Element("ExpressionCharCell", attrib={"char": char})
                        root.find("Expression").append(new)
                    new = ET.Element(
                        "ExpressionTokenCell",
                        attrib={"token": " ?= ", "scriptCode": "=="},
                    )
                    root.find("Expression").append(new)
                    if dIO_value == "HIGH":
                        new = ET.Element(
                            "ExpressionTokenCell",
                            attrib={"token": " True ", "scriptCode": " True "},
                        )
                    elif dIO_value == "LOW":
                        new = ET.Element(
                            "ExpressionTokenCell",
                            attrib={"token": " False ", "scriptCode": " False "},
                        )
                    else:
                        self.addlog("Please check the DI Value:")
                        self.addlog("Signal level, boolean type data, True or False")
                        self.addlog("Current DI Value:" + str(instruction["io_value"]))
                        return
                    root.find("Expression").append(new)
                    new = ET.Element(
                        "ExpressionTokenCell",
                        attrib={"token": " or ", "scriptCode": " or "},
                    )
                    root.find("Expression").append(new)
                    new = ET.Element("ExpressionVariableCell")
                    root.find("Expression").append(new)
                    new = ET.Element(
                        "TaskVariable",
                        attrib={"reference": "../../../../TimerNode/variable"},
                    )
                    root.find("Expression").find("ExpressionVariableCell").append(new)
                    for char in "≥" + str(instruction["timeout_ms"] / 1000):
                        new = ET.Element("ExpressionCharCell", attrib={"char": char})
                        root.find("Expression").append(new)
                    self.task += ET.tostring(root, encoding="UTF-8").decode()
                    waitdi = """
<TimerNode action="STOP" typeName="计时器">
    <variable reference="../../TimerNode/variable"/>
</TimerNode>
                    """
                    self.task += waitdi

            elif instruction["name"] == "RunCode":
                if instruction["is_function_call"]:
                    if self.PROGRAM_CALL_MODE == 1:
                        if instruction["is_first_call"]:
                            runcode = (
                                '''
<CallSubTaskNode typeName="调用">
    <subTask name="'''
                                + instruction["code"]
                                + """" isHideSubtree="false" isSynchronized="false" isTracking="false" typeName="子任务">
    <children>
                            """
                            )
                            self.task += runcode

                            temp_tcp = current_tcp
                            temp_frame = current_frame
                            temp_speed = current_speed
                            temp_acceleration = current_acceleration
                            temp_speedjoints = current_speedjoints
                            temp_acceleationjoints = current_accelerationjoints
                            temp_rounding = current_rounding

                            current_tcp = ""
                            current_frame = ""
                            current_speed = str(self.DEFAULT_SPEED / 1000)
                            current_acceleration = str(self.DEFAULT_ACCELERATION / 1000)
                            current_speedjoints = str(
                                math.radians(self.DEFAULT_SPEEDJOINTS)
                            )
                            current_accelerationjoints = str(
                                math.radians(self.DEFAULT_ACCELERATIONJOINTS)
                            )
                            current_rounding = self.DEFAULT_ROUNDING / 1000

                            try:
                                self.convert_program(instruction["code"])
                            except:
                                runcode = """
    <PlaceHolder typeName="占位节点"/>
                                """
                                self.task += runcode
                                self.addlog(
                                    "Program:" + instruction["code"] + " not found."
                                )
                                self.addlog("Please check the program name!")
                            else:
                                current_tcp = temp_tcp
                                current_frame = temp_frame
                                current_speed = temp_speed
                                current_acceleration = temp_acceleration
                                current_speedjoints = temp_speedjoints
                                current_accelerationjoints = temp_acceleationjoints
                                current_rounding = temp_rounding

                            runcode = """
    </children>
    </subTask>
</CallSubTaskNode>
                            """
                            self.task += runcode
                        else:
                            if ProgNames.index(instruction["code"]) == 1:
                                runcode = """
<CallSubTaskNode typeName="调用">
    <subTask reference="../../CallSubTaskNode/subTask"/>
</CallSubTaskNode>
                                """
                            else:
                                runcode = (
                                    """
<CallSubTaskNode typeName="调用">
    <subTask reference="../../CallSubTaskNode["""
                                    + ProgNames.index(instruction["code"])
                                    + """]/subTask"/>
</CallSubTaskNode>
                                """
                                )
                            self.task += runcode
                    else:
                        runcode = (
                            '''
<FolderNode display="'''
                            + instruction["code"]
                            + """" isHideSubtree="false" typeName="文件夹">
                        """
                        )
                        self.task += runcode
                        try:
                            self.convert_program(instruction["code"])
                        except:
                            runcode = """
<PlaceHolder typeName="占位节点"/>
                            """
                            self.task += runcode
                            self.addlog(
                                "Program:" + instruction["code"] + " not found."
                            )
                            self.addlog("Please check the program name!")
                        runcode = """
</FolderNode>
                        """
                        self.task += runcode
                else:
                    for line in instruction["code"].splitlines():
                        runcode = """
<ScriptNode scriptType="LINE_TYPE" typeName="脚本">
    <Expression>
    </Expression>
</ScriptNode>
                        """
                        root = ET.fromstring(runcode)
                        for char in line:
                            new = ET.Element(
                                "ExpressionCharCell", attrib={"char", char}
                            )
                            root.find("Expression").append(new)
                        self.task += ET.tostring(root, encoding="UTF-8").decode()

            elif instruction["name"] == "RunMessage":
                if instruction["iscomment"]:
                    runmessage = """
<Comment comment="" typeName="注释"/>
                    """
                    root = ET.fromstring(runmessage)
                    root.set("comment", instruction["message"])
                    self.task += ET.tostring(root, encoding="UTF-8").decode()
                else:
                    runmessage = """
<PopupNode dialogType="INFO" contentType="TEXT" message="" blockWhenPopup="false" typeName="弹出窗口"/>
                    """
                    root = ET.fromstring(runmessage)
                    root.set("message", instruction["message"])
                    self.task += ET.tostring(root, encoding="UTF-8").decode()

    def convert_task(self, filename, progname):
        """Convert programs to task file. This is a private method used only by the other methods.

        :param progname: The name of main program
        :type progname: str
        """
        self.task = (
            '''
<EliTask name="'''
            + filename
            + '''" installationName="'''
            + filename
            + '''" robotType="'''
            + self.ROBOT_NAME.split(" ")[-1]
            + """" typeName="机器人主任务">
  <MainTask isTaskAlwaysLoops="true" hasBeforeStart="false" hasInitVariables="false" typeName="机器人主任务">
        """
        )

        self.convert_program(progname)

        self.task += """
  </MainTask>
        """

        if self.PROGRAM_CALL_MODE == 1:
            for i in range(1, len(ProgNames)):
                if i == 1:
                    self.task += """
  <SubTaskNode reference="../MainTask/CallSubTaskNode/subTask"/>
                    """
                else:
                    self.task += (
                        """
  <SubTaskNode reference="../MainTask/CallSubTaskNode["""
                        + str(i)
                        + """]/subTask"/>
                    """
                    )

        self.task += """
</EliTask>
        """
        root = ET.fromstring(self.task)
        if self.LANGUAGE_SETTING in self.language_dict:
            for nodename, typename in self.language_dict[self.LANGUAGE_SETTING].items():
                for node in root.iter(nodename):
                    node.set("typeName", typename)
                    if nodename == "WaypointNode":
                        node.set(
                            "name",
                            node.get("name").replace(
                                "路点",
                                self.language_dict[self.LANGUAGE_SETTING][
                                    "WaypointNode"
                                ],
                            ),
                        )
                        node.set(
                            "name",
                            node.get("name").replace(
                                "经过点",
                                self.language_dict[self.LANGUAGE_SETTING]["ViaPoint"],
                            ),
                        )
                        node.set(
                            "name",
                            node.get("name").replace(
                                "终点",
                                self.language_dict[self.LANGUAGE_SETTING]["EndPoint"],
                            ),
                        )
                    elif nodename == "TimerNode":
                        if node.get("action") == "RESET":
                            node.find("variable").set(
                                "name",
                                node.find("variable")
                                .get("name")
                                .replace(
                                    "计时器",
                                    self.language_dict[self.LANGUAGE_SETTING][
                                        "TimerNode"
                                    ],
                                ),
                            )

        ET.indent(root)
        for line in ET.tostring(root, encoding="UTF-8").decode().splitlines():
            self.addline(line)

    def convert_configuration(self):
        """Convert tcps and frames to configuration. This is a private method used only by the other methods."""
        configuration = """
<ConfigurationSettings robotType="6206" savedTime="2024-02-19 14:29:50">
  <ToolsManager Active="21b4d8ff-5458-42e7-ae90-d80529e1d6da">
    <TcpTool name="TCP" uuid="21b4d8ff-5458-42e7-ae90-d80529e1d6da">
      <ToolShape value="0.0, 0.0, 0.0, 0.0, 0.0, 0.0"/>
    </TcpTool>
  </ToolsManager>
  <FramesManager>
  </FramesManager>
  <IOConfig>
  </IOConfig>
  <StartupConfigsManager>
    <config needAutoLoad="false" triggerValueToRunTask="false" triggerValueToInit="false"/>
  </StartupConfigsManager>
</ConfigurationSettings>
        """
        root = ET.fromstring(configuration)
        robotType = self.ROBOT_NAME.split(" ")[-1].replace("CS6", "")
        if len(robotType) == 1:
            robotType = "0" + robotType
        root.set("robotType", "62" + robotType)
        root.set("savedTime", time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))

        current_uuid = str(uuid.uuid1())
        root.find("ToolsManager").set("Active", current_uuid)
        root.find("ToolsManager").find("TcpTool").set("uuid", current_uuid)
        root.find("ToolsManager").find("TcpTool").find("ToolShape").set(
            "value",
            f"{self.DEFAULT_TCP[0]}, {self.DEFAULT_TCP[1]}, {self.DEFAULT_TCP[2]}, {math.radians(self.DEFAULT_TCP[3])}, {math.radians(self.DEFAULT_TCP[4])}, {math.radians(self.DEFAULT_TCP[5])}",
        )
        for tool_name, tool_pose in self.tcps.items():
            new = ET.Element(
                "TcpTool", attrib={"name": tool_name, "uuid": str(uuid.uuid1())}
            )
            new.append(
                ET.Element(
                    "ToolShape",
                    attrib={"value": ", ".join([str(x) for x in tool_pose])},
                )
            )
            root.find("ToolsManager").append(new)

        for frame_name, frame_pose in self.frames.items():
            new = ET.Element(
                "Frame",
                attrib={
                    "name": frame_name,
                    "isValid": "true",
                    "isVariable": "true",
                    "uuid": str(uuid.uuid1()),
                },
            )
            new.append(
                ET.Element(
                    "Transform",
                    attrib={"value": ", ".join([str(x) for x in frame_pose])},
                )
            )
            root.find("FramesManager").append(new)

        if self.analogOutputDomain[0] != "-1" or self.analogOutputDomain[1] != "-1":
            if self.analogOutputDomain[0] == "-1":
                self.analogOutputDomain[0] = "0"
            if self.analogOutputDomain[1] == "-1":
                self.analogOutputDomain[1] = "0"
            new = ET.Element(
                "analogOutputDomain",
                attrib={"value": ", ".join(self.analogOutputDomain)},
            )
            root.find("IOConfig").append(new)

        ET.indent(root)
        for line in ET.tostring(root, encoding="UTF-8").decode().splitlines():
            self.addline(line)


# -------------------------------------------------
# ------------ For testing purposes ---------------
def test_post():
    """Test the post processor with a simple program"""

    from robodk.robomath import PosePP as p

    r = RobotPost(
        r"""SamplePost""",
        r"""RoboDK Industrial""",
        6,
        axes_type=["R", "R", "R", "R", "R", "R"],
        native_name=r"""""",
        ip_com=r"""127.0.0.1""",
        api_port=20500,
        prog_ptr=2465610763008,
        robot_ptr=2465605812256,
    )
    r.ProgStart(r"""MAIN_1""")
    r.RunMessage(r"""Program generated by RoboDK using a custom post processor""", True)
    r.RunMessage(r"""Using nominal kinematics.""", True)
    r.setDO(5, 1)
    r.setAO(1, 1)

    r.waitDI(5, 1, 100)

    r.RunMessage(r"""Message""")

    r.RunMessage(r"""Insert Comment""", True)
    r.setFrame(p(19.013, -4.003, 0, 0, 0, 0), -1, r"""Frame Pick and Place""")
    r.setTool(p(29.073, 0, 5.477, 0, 22.849, 0), 1, r"""Tool 1""")
    r.setAccelerationJoints(800.000)
    r.setSpeedJoints(500.000)
    r.setZoneData(10.000)
    r.MoveJ(
        p(187.264, 125.487, 305.356, 180, 0.184, -180),
        [29.3322, 46.677, 0.024523, 26.5456, 25.3957, 51.634],
        [0, 0, 0],
    )
    r.setZoneData(-1.000)
    r.setAcceleration(200.000)
    r.setSpeed(100.000)
    r.MoveL(
        p(-187.786, -125.499, -139.112, -179.997, 0.18, 179.996),
        [29.3185, 56.1632, 0.458104, 38.9815, 17.7354, 64.9585],
        [0, 0, 0],
    )
    r.setDO(5, 1)

    r.setZoneData(10.000)
    r.setAcceleration(600.000)
    r.setSpeed(300.000)

    r.MoveL(
        p(-187.272, -447.522, -305.347, 179.999, 0.185, -179.998),
        [-12.7285, 39.1891, 14.3365, 19.7398, 14.7824, -30.877],
        [0, 0, 0],
    )
    r.setZoneData(-1.000)
    r.setAcceleration(200.000)
    r.setSpeed(100.000)
    r.MoveL(
        p(-187.851, -447.517, -126.235, 179.999, 0.185, -179.998),
        [-12.7213, 50.7452, 13.7281, 58.8071, 5.77865, -70.4133],
        [0, 0, 0],
    )
    r.setDO(5, 0)

    r.setZoneData(10.000)
    r.MoveL(
        p(-187.272, -447.522, -305.347, 179.999, 0.185, -179.998),
        [-12.7285, 39.1891, 14.3365, 19.7398, 14.7824, -30.877],
        [0, 0, 0],
    )
    r.setFrame(p(7.67, 2.25, 0, 38.9595, 0, 0), -1, r"""Frame Circular""")
    r.setTool(p(29.073, 0, 5.477, 0, 22.849, 0), 1, r"""Tool 1""")
    r.setZoneData(25.000)
    r.setAccelerationJoints(800.000)
    r.setSpeedJoints(500.000)
    r.setAcceleration(200.000)
    r.setSpeed(100.000)
    r.MoveJ(
        p(312.208, 292.214, 255.156, -134.549, 11.1813, 178.913),
        [52.186, 45.6758, 6.92, 66.7342, 19.9323, -92.73],
        [0, 0, 0],
    )
    r.MoveC(
        p(-30.047, -343.774, -331.503, -134.549, 11.1813, 178.913),
        [38.5084, 52.8105, -4.82239, 56.4521, 29.7882, -93.0255],
        p(-39.688, -353.222, -361.073, -134.549, 11.1813, 178.913),
        [24.802, 41.3257, 29.6032, 87.9704, 29.4626, -142.416],
        [0, 0, 0],
        [0, 0, 0],
    )

    r.setZoneData(-1.000)
    r.setAcceleration(200.000)
    r.setSpeed(100.000)
    r.MoveL(
        p(72.208, -150, -180.156, -134.549, 11.1813, 178.913),
        [52.186, 45.6758, 6.92, 66.7342, 19.9323, -92.73],
        [0, 0, 0],
    )

    r.setFrame(p(0, 2.714, 7.61, 0, 0, 0), -1, r"""Frame Inspection""")
    r.setTool(p(29.073, 0, 5.477, 0, 22.849, 0), 1, r"""Tool 1""")
    r.MoveJ(
        p(19.898, 201.357, 302.721, -94.9281, -5.42599, -179.533),
        [88.676, 29.6016, -6.42883, -1.98422, 49.4792, 4.54769],
        [0, 0, 0],
    )
    r.setFrame(p(0, 2.714, 7.61, 0, 0, 0), -1, r"""Frame Inspection""")
    r.setTool(p(29.073, 0, 5.477, 0, 22.849, 0), 1, r"""Tool 1""")
    r.MoveL(
        p(-119.898, -174.307, -119.008, -94.9281, -5.42599, -179.533),
        [88.6412, 37.6157, 7.6344, -3.25372, 27.4219, 6.11377],
        [0, 0, 0],
    )
    r.MoveL(
        p(-178.402, -174.416, -118.997, -94.9281, -5.42599, -179.533),
        [69.5908, 42.46, -1.04795, 7.90648, 31.8952, -21.7004],
        [0, 0, 0],
    )
    r.MoveL(
        p(-180.996, -112.363, -134.882, -95.2937, -18.1364, -178.348),
        [69.0981, 41.4527, 1.28767, -0.293386, 42.3038, -15.9791],
        [0, 0, 0],
    )
    r.MoveL(
        p(-116.551, -114.359, -134.226, -95.2937, -18.1364, -178.348),
        [88.801, 36.7217, 9.82069, -2.95529, 38.8939, 5.74175],
        [0, 0, 0],
    )
    r.MoveL(
        p(-126.434, -154.524, -146.072, -95.4608, 4.34825, 179.585),
        [87.9395, 25.3303, 24.2482, -5.09867, 13.2949, 8.14809],
        [0, 0, 0],
    )
    r.MoveL(
        p(-185.448, -148.05, -145.137, -95.5904, 11.1809, 178.913),
        [65.1172, 28.8227, 18.4513, 48.4598, 15.6087, -63.1988],
        [0, 0, 0],
    )
    r.setFrame(p(0, 2.714, 7.61, 0, 0, 0), -1, r"""Frame Inspection""")
    r.setTool(p(29.073, 0, 5.477, 0, 22.849, 0), 1, r"""Tool 1""")
    r.MoveL(
        p(-185.448, -100.625, -195.251, -95.5904, 11.1809, 178.913),
        [63.7612, 15.3737, 13.1682, 23.9712, 31.7352, -37.6981],
        [0, 0, 0],
    )

    r.setZoneData(1.000)
    r.setSpeed(1000.000)
    r.setFrame(p(13.637, 40.08, 0, -9, 0, 0), -1, r"""Frame Inspection""")
    r.setTool(p(29.073, 0, 5.477, 0, 22.849, 0), 1, r"""Tool 1""")
    r.RunMessage(r"""Show Tool 1""", True)
    r.MoveJ(
        p(-49.939, 252.644, 419.997, -90, 0.353, -179.975),
        [-68.0903, 24.2019, 26.7855, 38.2134, 36.2017, -98.7964],
        [0, 0, 0],
    )

    r.MoveL(
        p(-149.895, 53.259, -119.999, -90, 0.353, -179.975),
        [-68.0531, 30.5428, 28.4999, 46.5342, 30.2163, -108.684],
        [0, 0, 0],
    )
    r.MoveL(
        p(-200, 50, -170, -90, 0.353, -179.975),
        [-69.4372, 36.478, 24.0662, 48.0716, 29.7098, -111.839],
        [0, 0, 0],
    )

    r.setSpeed(50.000)
    r.MoveL(
        p(-235.579, -147.455, -70, -90, 0.345, -179.925),
        [-70.3489, 38.2711, 20.535, 45.6808, 31.1886, -109.993],
        [0, 0, 0],
    )
    r.MoveL(
        p(-270.433, -139.873, -70, -89.999, 0.330998, -179.876),
        [-71.4686, 39.9889, 17.146, 43.5315, 32.7489, -108.597],
        [0, 0, 0],
    )
    r.MoveL(
        p(-203.854, -127.408, -70, -89.999, 0.309997, -179.831),
        [-72.7633, 41.6047, 13.9532, 41.6289, 34.3488, -107.63],
        [0, 0, 0],
    )
    r.MoveL(
        p(-235.16, -110.313, -70, -89.999, 0.282996, -179.788),
        [-74.2041, 43.0922, 11.0104, 39.9679, 35.9473, -107.066],
        [0, 0, 0],
    )
    r.MoveL(
        p(-263.715, -188.937, -70, -89.999, 0.249996, -179.75),
        [-75.7653, 44.4258, 8.36962, 38.5347, 37.5045, -106.874],
        [0, 0, 0],
    )
    r.MoveL(
        p(-288.937, -163.715, -70, -89.999, 0.211995, -179.717),
        [-77.4239, 45.581, 6.08058, 37.3112, 38.9832, -107.02],
        [0, 0, 0],
    )
    r.MoveL(
        p(-210.313, -135.16, -70, -89.999, 0.169995, -179.689),
        [-79.1593, 46.5348, 4.19016, 36.281, 40.3478, -107.477],
        [0, 0, 0],
    )
    r.MoveL(
        p(-227.408, -103.854, -70, -89.999, 0.123994, -179.668),
        [-80.9524, 47.2671, 2.73896, 35.4254, 41.5679, -108.213],
        [0, 0, 0],
    )
    r.MoveL(
        p(-239.873, -170.433, -70, -90, 0.075, -179.654),
        [-82.7857, 47.7617, 1.76039, 34.7286, 42.6165, -109.203],
        [0, 0, 0],
    )
    r.MoveL(
        p(-247.455, -135.579, -70, -90, 0.025, -179.647),
        [-84.6423, 48.0069, 1.27773, 34.1759, 43.4729, -110.421],
        [0, 0, 0],
    )
    r.MoveL(
        p(-250, -100, -70, -90, -0.025, -179.647),
        [-86.5062, 47.9966, 1.30332, 33.7555, 44.1209, -111.841],
        [0, 0, 0],
    )
    r.MoveL(
        p(-147.455, -135.579, -70, -90, -0.075, -179.654),
        [-88.3612, 47.7315, 1.83611, 33.4564, 44.5527, -113.438],
        [0, 0, 0],
    )
    r.MoveL(
        p(-139.873, -170.433, -70, -90.001, -0.123994, -179.668),
        [-90.1913, 47.2177, 2.86315, 33.2711, 44.7653, -115.187],
        [0, 0, 0],
    )
    r.MoveL(
        p(-227.408, -103.854, -70, -90.001, -0.169995, -179.689),
        [-91.9795, 46.4674, 4.35991, 33.1942, 44.7621, -117.062],
        [0, 0, 0],
    )
    r.MoveL(
        p(-210.313, -135.16, -70, -90.001, -0.211995, -179.717),
        [-93.7081, 45.4974, 6.29208, 33.2231, 44.5515, -119.037],
        [0, 0, 0],
    )
    r.MoveL(
        p(-188.937, -163.715, -70, -90.001, -0.249996, -179.75),
        [-95.358, 44.3279, 8.61845, 33.3584, 44.1445, -121.088],
        [0, 0, 0],
    )
    r.MoveL(
        p(-163.715, -188.937, -70, -90.001, -0.282996, -179.788),
        [-96.9084, 42.9821, 11.2918, 33.6038, 43.5568, -123.189],
        [0, 0, 0],
    )
    r.MoveL(
        p(-235.16, -210.313, -70, -90.001, -0.310997, -179.83),
        [-98.3359, 41.4847, 14.2621, 33.9667, 42.805, -125.317],
        [0, 0, 0],
    )
    r.MoveL(
        p(-203.854, -227.408, -70, -90.001, -0.331998, -179.876),
        [-99.6145, 39.8611, 17.4769, 34.4583, 41.9071, -127.446],
        [0, 0, 0],
    )
    r.MoveL(
        p(-170.433, -239.873, -70, -90, -0.346, -179.925),
        [-100.715, 38.1376, 20.8827, 35.0938, 40.8826, -129.555],
        [0, 0, 0],
    )
    r.MoveL(
        p(-235.579, -247.455, -70, -90, -0.353, -179.975),
        [-101.604, 36.341, 24.425, 35.8921, 39.7503, -131.622],
        [0, 0, 0],
    )
    r.MoveL(
        p(-200, -250, 270, -90, -0.353, -179.975),
        [-102.247, 34.4892, 28.0665, 36.9342, 38.491, -133.695],
        [0, 0, 0],
    )
    r.MoveL(
        p(-200, -250, 270, -90, -0.353, 179.975),
        [-102.245, 34.4987, 28.0477, 36.877, 38.5317, -133.625],
        [0, 0, 0],
    )
    r.MoveL(
        p(-235.579, -247.455, -70, -90, -0.346, 179.925),
        [-102.596, 32.6395, 31.6928, 38.0753, 37.2477, -135.543],
        [0, 0, 0],
    )
    r.MoveL(
        p(-170.433, -239.873, -70, -89.999, -0.330998, 179.876),
        [-102.611, 30.7935, 35.299, 39.5163, 35.9209, -137.353],
        [0, 0, 0],
    )
    r.MoveL(
        p(-203.854, -227.408, -70, -89.999, -0.309997, 179.831),
        [-102.244, 28.9935, 38.801, 41.2289, 34.5749, -139.031],
        [0, 0, 0],
    )
    r.MoveL(
        p(-235.16, -210.313, -70, -89.999, -0.282996, 179.788),
        [-101.448, 27.2756, 42.1281, 43.2369, 33.234, -140.549],
        [0, 0, 0],
    )
    r.MoveL(
        p(-163.715, -188.937, -70, -89.999, -0.249996, 179.75),
        [-100.183, 25.6792, 45.2042, 45.554, 31.924, -141.876],
        [0, 0, 0],
    )
    r.MoveL(
        p(-188.937, -163.715, -70, -89.999, -0.211995, 179.717),
        [-98.4224, 24.2473, 47.9477, 48.1695, 30.6693, -142.971],
        [0, 0, 0],
    )
    r.MoveL(
        p(-210.313, -135.16, -70, -89.999, -0.168995, 179.69),
        [-96.1665, 23.0258, 50.2743, 51.0388, 29.4927, -143.783],
        [0, 0, 0],
    )
    r.MoveL(
        p(-227.408, -103.854, -70, -89.999, -0.122994, 179.669),
        [-93.4513, 22.0607, 52.1014, 54.0696, 28.4134, -144.254],
        [0, 0, 0],
    )
    r.MoveL(
        p(-239.873, -170.433, -70, -90, -0.075, 179.654),
        [-90.3597, 21.3939, 53.3553, 57.1156, 27.4453, -144.315],
        [0, 0, 0],
    )
    r.MoveL(
        p(-247.455, -135.579, -70, -90, -0.025, 179.647),
        [-87.0219, 21.0579, 53.9806, 59.9848, 26.5968, -143.904],
        [0, 0, 0],
    )
    r.MoveL(
        p(-250, -100, -70, -90, 0.025, 179.647),
        [-83.6036, 21.0703, 53.9474, 62.4559, 25.8733, -142.962],
        [0, 0, 0],
    )
    r.MoveL(
        p(-247.455, -135.579, -70, -90, 0.075, 179.655),
        [-80.2832, 21.4306, 53.2574, 64.3165, 25.2806, -141.457],
        [0, 0, 0],
    )
    r.MoveL(
        p(-239.873, -170.433, -70, -90.001, 0.122994, 179.669),
        [-77.224, 22.12, 51.9432, 65.4009, 24.829, -139.389],
        [0, 0, 0],
    )
    r.MoveL(
        p(-227.408, -103.854, -70, -90.001, 0.169995, 179.689),
        [-74.5516, 23.1051, 50.0624, 65.6191, 24.5358, -136.794],
        [0, 0, 0],
    )
    r.MoveL(
        p(-210.313, -135.16, -70, -90.001, 0.211995, 179.717),
        [-72.344, 24.3435, 47.6903, 64.973, 24.4221, -133.759],
        [0, 0, 0],
    )
    r.MoveL(
        p(-188.937, -163.715, -70, -90.001, 0.249996, 179.75),
        [-70.6334, 25.7892, 44.9098, 63.5472, 24.5124, -130.402],
        [0, 0, 0],
    )
    r.MoveL(
        p(-163.715, -188.937, -70, -90.001, 0.282996, 179.788),
        [-69.4163, 27.3965, 41.8051, 61.4929, 24.8267, -126.877],
        [0, 0, 0],
    )
    r.MoveL(
        p(-235.16, -110.313, -70, -90.001, 0.309997, 179.831),
        [-68.6649, 29.1225, 38.4571, 58.9975, 25.3753, -123.345],
        [0, 0, 0],
    )
    r.MoveL(
        p(-203.854, -127.408, -70, -90.001, 0.331998, 179.876),
        [-68.338, 30.9278, 34.9417, 56.2542, 26.1575, -119.965],
        [0, 0, 0],
    )
    r.MoveL(
        p(-170.433, -139.873, -70, -90, 0.346, 179.925),
        [-68.3888, 32.7769, 31.3288, 53.4349, 27.1586, -116.864],
        [0, 0, 0],
    )
    r.MoveL(
        p(-235.579, -147.455, -70, -90, 0.353, 179.975),
        [-68.7701, 34.6371, 27.6833, 50.6754, 28.3536, -114.137],
        [0, 0, 0],
    )
    r.MoveL(
        p(-200, -150, -70, -90, 0.353, 179.975),
        [-69.4415, 36.4872, 24.0478, 48.0255, 29.7609, -111.78],
        [0, 0, 0],
    )
    r.setSpeed(1000.000)

    r.MoveL(
        p(-149.895, -153.258, -19.999, -90, 0.353, 179.975),
        [-70.3186, 36.0547, 18.426, 41.04, 34.416, -104.418],
        [0, 0, 0],
    )
    r.MoveL(
        p(-149.939, -152.642, -19.997, -90, 0.353, 179.975),
        [-70.3547, 30.4247, 16.7719, 35.1262, 40.174, -97.0327],
        [0, 0, 0],
    )

    r.setFrame(p(19.013, -4.003, 0, 0, 0, 0), -1, r"""Frame Pick and Place""")
    r.setTool(p(29.073, 0, 5.477, 0, 22.849, 0), 1, r"""Tool 1""")
    r.MoveJ(
        p(187.264, 125.487, 305.356, 180, 0.184, -180),
        [-29.3322, 46.677, -0.024523, 26.5456, 25.3957, -51.634],
        [0, 0, 0],
    )
    r.Pause(1250.0)
    r.MoveL(
        p(-187.786, -125.499, -139.112, -179.997, 0.18, 179.996),
        [-29.3185, 56.1632, -0.458104, 38.9815, 17.7354, -64.9585],
        [0, 0, 0],
    )

    r.MoveL(
        p(-187.264, -125.487, 5.356, 180, 0.184, -180),
        [-29.3322, 46.677, -0.024523, 26.5456, 25.3957, -51.634],
        [0, 0, 0],
    )
    r.RunCode(r"""PROG_1""", True)
    r.ProgFinish(r"""MAIN_1""")
    r.ProgStart(r"""PROG_1""")
    r.RunMessage(r"""This is a subprogram call to PROG_1""", True)
    r.ProgFinish(r"""PROG_1""")
    r.ProgSave(".", r"""MAIN_1""", True, True)

    for line in r.PROG:
        print(line)

    if len(r.LOG) > 0:
        mbox("Program generation LOG:\n\n" + r.LOG)

    input("Press Enter to close...")


if __name__ == "__main__":
    """Procedure to call when the module is executed by itself: test_post()"""
    test_post()
