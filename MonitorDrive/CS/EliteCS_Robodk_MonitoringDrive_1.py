'''
Author: chenliao@elibot.cn
CreateDate: 2023-08-10 17:00:00
Description: RoboDK 监控ELite_CS机器人，在仿真中模拟真实机器人的运动
'''

# from robolink.robolink import *    # API to communicate with RoboDK for simulation and offline/online programming
# from robodk.robodk import *      # Robotics toolbox for industrial robots


from robodk import *      # RoboDK API
from robolink import *    # Robot toolbox

import threading
import socket
import struct
import time, datetime


global ROBOT_JOINTS

class EcRobodkDrive:
    
    # define date position ,(start pos,end pos)
    Elite_GET_TIME = 1
    Elite_GET_BUF_LEN = (0,4)
    Elite_GET_JOINT_POSITION = (13,77) 
    Elite_GET_JOINT_SPEEDS = None
    Elite_GET_JOINT_CURRENTS = None
    Elite_GET_TCP_FORCES = None
    
    
    def __init__(self, ip: str = "192.168.1.200", port: int = 30001) -> None:
        self.ip = ip
        self.port = port
    
    
    def __connect(self):
        """create a socket obj connect
        """
        try:
            # self.ec_sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            # self.ec_sock.settimeout(8)

            # self.ec_sock.connect((self.ip,self.port))
            self.ec_sock = socket.create_connection((self.ip,self.port))
            # self.ec_sock.settimeout(8)
        except:
            print("%s 连接失败,请检查对应机器人的8056端口是否已经打开" %self.ip)
            print("%s The connection failed. Please check whether port 8056 of the corresponding robot is open" %self.ip)
            quit()
    
    
    def __first_recv(self, buf_len: int = 366):
        """Get data from port 8056 and parse the packet length corresponding to the current version  

        Args:
            buf_len (int, optional): The length of the packet that first fetched the data. Defaults to 366.
        """
        self.__connect()
        buf_len = buf_len
        first_buf = self.ec_sock.recv(buf_len,socket.MSG_WAITALL)
        self.buf_len = self.get_buf_len(first_buf)
        self.ec_sock.close()
        self.ec_sock = None

    
    def get_buf_len(self,buf: bytes):
        """Get length information from byte packets

        Args:
            buf (bytes): The byte package returned by 8056

        Returns:
            int: Length of the parsed packet
        """
        date_len = buf[self.Elite_GET_BUF_LEN[0]:self.Elite_GET_BUF_LEN[1]]
        if hasattr(self,"buf_len") and self.buf_len[0] != len(buf):
            print("字节长度错误，为%s" %len(buf))
            print("The length of the byte is incorrect,%s" %len(buf))

        # print(date_len)
        return struct.unpack("!I",date_len)
    
    
    def get_joint_data(self, buf: bytes):
        """Intercept and parse joint information

        Args:
            buf (bytes): The byte package returned by 8056

        Returns:
            tuple: Parsed joint data
        """
        joint_byte = buf[self.Elite_GET_JOINT_POSITION[0]:self.Elite_GET_JOINT_POSITION[1]]
        # print(joint_byte)
        return struct.unpack_from("!dddddddd",joint_byte,0)
        
    
    def loop(self):
        """loop process
        """
        # self.__first_recv()
        self.__connect()
        
        global ROBOT_JOINTS
        recv_count = 0
        recv_time_last = time.time()
        while True:

            __buf = self.ec_sock.recv(4096*3)
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
                    j = [0, 0, 0, 0, 0, 0]
                    print('sublen:'+str(sub_length))
                    for i in range(0, 6):
                        j[i] = struct.unpack(">d", data1[curr_data_add:curr_data_add+8])[0]
                        j[i] = round(j[i] / 3.1415926 * 180.0, 1)
                        curr_data_add = curr_data_add+(119-62)

                    ROBOT_JOINTS=[j[0],j[1],j[2],j[3],j[4],j[5]]
            
            # buf = self.ec_sock.recv(4096)

            # if self.buf_len != self.get_buf_len(buf):
            #     print("数据包长度错误，原始长度%s,实际长度%s" % (self.buf_len,self.get_buf_len(buf)))
            #     print("Packet length error, original length%s,actual length%s" % (self.buf_len,self.get_buf_len(buf)))
            #     self.ec_sock.close()
            #     self.__connect()
            #     continue

            # ROBOT_JOINTS = list(self.get_joint_data(buf))

            recv_count += 1
            if recv_count % 10 == 0:       #*计算每秒的数据包个数
                t_now = time.time()
                print("Monitoring at %.1f packets per second" % (recv_count/(t_now-recv_time_last)))
                recv_count = 0
                recv_time_last = t_now
            
            time.sleep(0.01)
            
        self.ec_sock.close()
    
    
# Procedure to check if robot joint positions are different according to a certain tolerance
def Robot_Joints_Check(jA,jB, tolerance_deg=1):
    # 设置一个误差值，从而判断机器人是否真的已经移动啦
    #todo:是否要*pi/180
    if jA is None:
        return True
    
    for i in range(6):
        if abs(jA[i]-jB[i]) > tolerance_deg*pi/180:
            return True
    return False 
    
    
def main_loop():
    """main loop
    """
    global ROBOT_JOINTS

    ROBOT_JOINTS = None
    # init joint data
    last_joints_target = None
    last_joints_refresh = None
    target_count = 0
    
    # Refresh the screen every time the robot position changes
    TOLERANCE_JOINTS_REFRESH   = 0.1
    RETRIEVE_JOINTS_ONCE = False  # If True, the current robot position will be retrieved once only

    # Create targets given a tolerance in degrees
    CREATE_TARGETS = True
    TOLERANCE_JOINTS_NEWTARGET = 10 # in degrees
    
    REFRESH_RATE = 0.1
    
    # The following constants are used to display the trajectory of motion
    PROGRAM_NAME = None
    CREATE_MOVJ = True
    HIDE_TARGET = True
    HIDE_PROGRAM = True

    while 1 :
        # Gets the number of threads to determine whether the thread is still running
        length = len(threading.enumerate())
        if length < 2:
            print("程序退出")
            quit()
        
        # print(ROBOT_JOINTS)
        if ROBOT_JOINTS == None :
            continue
        
        # Set the robot to that position 更新位置
        if Robot_Joints_Check(last_joints_refresh, ROBOT_JOINTS, TOLERANCE_JOINTS_REFRESH):
            last_joints_refresh = ROBOT_JOINTS    
            robot.setJoints(list(ROBOT_JOINTS))
        
        # Stop here if we need only the current position
        if RETRIEVE_JOINTS_ONCE:
            quit(0)
    
        # # Check if the robot has moved enough to create a new target
        # if CREATE_TARGETS and Robot_Joints_Check(last_joints_target, ROBOT_JOINTS, TOLERANCE_JOINTS_NEWTARGET):
        #     last_joints_target = ROBOT_JOINTS
        #     target_count = target_count + 1
        #     newtarget = RDK.AddTarget('T%i' % target_count, 0, robot)
            
        #     # create program and show move trajectory
        #     if CREATE_MOVJ == True and PROGRAM_NAME == None:
        #         PROGRAM_NAME = "EC_" + datetime.datetime.now().strftime('%Y-%m-%d_%H:%M')
        #         EC_pro = RDK.AddProgram(PROGRAM_NAME, robot)
            
        #     if CREATE_MOVJ == True and PROGRAM_NAME != None:
        #         EC_pro.addMoveJ(newtarget)
                
        #         if HIDE_PROGRAM :
        #             EC_pro.ShowInstructions(False)

        #         if HIDE_TARGET :
        #             EC_pro.ShowTargets(False)    
            
        # Take a short break        
        pause(REFRESH_RATE)
    

    
    
if __name__ == "__main__":


    # define ec robot ip and port
    # ROBOT_IP = '192.168.230.145'     # default ip : "192.168.1.200"
    ROBOT_IP = None     # default ip : "192.168.1.200"
    
    # create Robodk obj
    RDK = Robolink()

    # get robot obj
    robot = RDK.ItemUserPick("请选择Elite_EC机器人", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        quit()
    
    # get robot's ip from robodk frame:
    if ROBOT_IP == None :
        ip,port,path,ftpuser,ftppass = robot.ConnectionParams()
        if ip == "127.0.0.1":
            ROBOT_IP = "192.168.1.200"
        else:
            ROBOT_IP = ip
        
    # create EC robot obj and state loop thread
    ec_robot = EcRobodkDrive(ROBOT_IP)
    threading.Thread(target = ec_robot.loop, daemon = True).start()
    main_loop()

    # threading.Thread(target = main_loop,args=(ROBOT_JOINTS,) , daemon = True).start()
    # ec_robot.loop(ROBOT_JOINTS)