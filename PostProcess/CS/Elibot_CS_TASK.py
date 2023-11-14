# Copyright 2015-2020 - RoboDK Inc. - https://robodk.com/
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
# for an Elibot robot
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

def get_safe_name(progname, max_chars = 20):
    """Get a safe program name"""
    # Remove special characters
    for c in r'-[]/\;,><&*:%=+@!#^()|?^':
        progname = progname.replace(c,'')
    # Set a program name by default:
    if len(progname) <= 0:
        progname = 'Program'
    # Force the program to start with a letter (not a number)
    if progname[0].isdigit():
        progname = 'Robodk_' + progname
    # Set the maximum size of a program (number of characters)
    if len(progname) > max_chars:
        progname = progname[:max_chars]
    return progname

# ----------------------------------------------------
# Import RoboDK tools
from time import sleep
from robodk import *
import sys

from robodk import ShowMessage


# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
class RobotPost(object):
    """Robot post object defined for Elibot robots"""
    #--------------------------------------------------------------------------------------
    # ---------------- Customize your post processor for best results ---------------------
    # Set the default maximum number of lines per program. 
    # It will then generate multiple "pages (files)". This can be overriden by RoboDK settings.    
    MAX_LINES_X_PROG = 2000

    # Specify the default UTool Id to use (register). 
    # You can also use Numbered tools in RoboDK (for example, a tool named "Tool 2" will use UTOOL number 2)
    ACTIVE_TOOL = 9
    
    # Generate sub programs with each program
    INCLUDE_SUB_PROGRAMS = True 
        
    # Specify a spare Position register for calculations (Tool, Reference, ...)
    SPARE_PR = 95
    
    # Set to True to use MFRAME for setting reference frames automatically within the program
    USE_MFRAME = False      
    
    # Specify the default UFrame Id to use (register).     
    # You can also use Numbered References in RoboDK (for example, a reference named "Reference 4" will use UFRAME number 4)
    ACTIVE_FRAME = 9   
        
    #----------------------------------------------------------------------

    PROG_EXT = 'task'             # set the program extension
    STR_V = '0.25'         # set default cartesian speed
    STR_ACC_CART = '1.2'         # set default cartesian speed
  
    MAX_VJ = 50.0               #system maximum joint speed deg/s
    STR_VJ = str(60/180*3.14159)       # set default joints speed
    STR_ACC_JOINT = str(100/180*3.14159)       # set default joints ACC
    STR_PL = '0'             # Rounding value (from 0 to 4) (in RoboDK, set to 100 mm rounding for PL=4
    User1 = Fanuc_2_Pose([0,0,0,0,0,0])       # Frame
    PreMoveType = ''
    CurrMoveType = ''
    #wp_name = 'Waypoint_'
    wp_name = '路点_'
    
    REGISTER_DIGITS = 5

    # PROG specific variables:
    LINE_COUNT = 0      # Count the number of instructions (limited by MAX_LINES_X_PROG)
    P_COUNT = 0         # Count the number of P targets in one file
    V_COUNT = 0         # Count the number of V targets in one file
    nProgs = 0          # Count the number of programs and sub programs

    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = [] # List of Program files to be uploaded through FTP
    
    PROG_NAMES = [] # List of PROG NAMES
    PROG_LIST = [] # List of PROG 
    
    PROG_NAME = 'unknown'  # Original name of the current program (example: ProgA)
    PROG_NAME_CURRENT = 'unknown' # Auto generated name (different from PROG_NAME if we have more than 1 page per program. Example: ProgA2)
    
    nPages = 0           # Count the number of pages
    PROG_NAMES_MAIN = [] # List of programs called by a main program due to splitting
    
    PROG = []     # Save the program lines
    PROG_TARGETS = []  # Save the program lines (targets section)

    LOG = '' # Save a log
    bStartCir = True
    bFirstIns = True
    bUseFrame = False
    
    nAxes = 6 # Important: This is usually provided by RoboDK automatically. Otherwise, override the __init__ procedure. 
    AXES_TYPE = ['R','R','R','R','R','R']  # Important: This is usually set up by RoboDK automatically. Otherwise, override the __init__ procedure.
    # 'R' for rotative axis, 'L' for linear axis, 'T' for external linear axis (linear track), 'J' for external rotative axis (turntable)
    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):

        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.nAxes = robot_axes
        self.PROG = []
        self.PROG_TARGETS = []
        self.LOG = ''
        #for k,v in kwargs.iteritems(): # python2
        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v
            if k == 'axes_type':
                self.AXES_TYPE = v                

    def ProgStart(self, progname, new_page = False):
        progname = get_safe_name(progname)
        progname_i = progname
        if new_page:
            #nPages = len(self.PROG_LIST)
            if self.nPages == 0:
                if len(self.PROG_NAMES_MAIN) > 0:
                    print("Can't split %s: Two or more programs are split into smaller programs" % progname)
                    print(self.PROG_NAMES_MAIN)
                    raise Exception("Only one program at a time can be split into smaller programs")
                self.PROG_NAMES_MAIN.append(self.PROG_NAME) # add the first program in the list to be genrated as a subprogram call
                self.nPages = self.nPages + 1

            self.nPages = self.nPages + 1
            progname_i = "%s%i" % (self.PROG_NAME, self.nPages)          
            self.PROG_NAMES_MAIN.append(progname_i)
            
        else:
            if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
                return
            self.PROG_NAME = progname
            self.nProgs = self.nProgs + 1
            #self.PROG_NAMES = []
            
        self.PROG_NAME_CURRENT = progname_i
        self.PROG_NAMES.append(progname_i)

    def ProgFinish(self, progname, new_page = False):
        #progname = get_safe_name(progname)
        progname = self.PROG_NAME_CURRENT
        if not new_page:
            # Reset page count
            self.nPages = 0

        #if self.nPROGS > 1:
        #    # Does not support defining multiple programs in the same file, one program per file
        #    return

        header = ''
        header += '//JOB' + '\n'
        header += '//NAME %s' % progname + '\n'
        header += '//POS' + '\n'

        import time        
        datestr = time.strftime("%Y/%m/%d %H:%M")
        
        header_ins = ''
        header_ins += '<EliTask name="Robodk_1" installationName="default" >\n'
        header_ins +='  <MainTask>\n'
      
        header_ins +='<!-- '       

        #self.PROG = self.PROG +self.PROG_TARGETS
        self.PROG.insert(0, header_ins)
        header_last ='  </MainTask>\n</EliTask>'
        self.PROG.append(header_last)
        # Save PROG in PROG_LIST
        self.PROG_LIST.append(self.PROG)
        self.PROG = []
        self.LINE_COUNT = 0
        self.P_COUNT = 0
        self.LBL_ID_COUNT = 0
        
    def progsave(self, folder, progname, ask_user = False, show_result = False):
        if ask_user or not DirExists(folder):
            folder = getSaveFolder(folder,'Select a directory to save your program')
            if folder is None:
                # The user selected the Cancel button
                return
                
        filesave = folder + '/' + progname + '.' + self.PROG_EXT
        
        import io
        # with open(filesave, "w", encoding="shift_jis") as fid:
        with open(filesave, "w", encoding="utf-8") as fid:
            #fid = io.open(filesave, "w", newline='\r\n')
            #fid.write(self.PROG)
            for line in self.PROG:
                fid.write(line)
                fid.write('\n')
                #fid.write(line.decode('unicode-escape'))
                #fid.write(u'\n')
                
        print('SAVED: %s\n' % filesave) # tell RoboDK the path of the saved file
        self.PROG_FILES.append(filesave)
        
        # open file with default application
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave])
            elif type(show_result) is list:
                import subprocess
                p = subprocess.Popen(show_result + [filesave])   
            else:
                # open file with default application
                import os
                os.startfile(filesave)
            #if len(self.LOG) > 0:
            #    mbox('Program generation LOG:\n\n' + self.LOG)
            
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        progname = get_safe_name(progname)
        nfiles = len(self.PROG_LIST)
        if nfiles >= 1:
            if self.LINE_COUNT > 0:
                # Progfinish was not called!
                print("Warning: ProgFinish was not called properly")
                self.PROG_LIST.append(self.PROG)
                self.PROG_NAMES.append("Unknown")
                self.PROG = []
                self.LINE_COUNT = 0
            
            if len(self.PROG_NAMES_MAIN) > 1:
                # Warning: the program might be cut to a maximum number of chars
                progname_main = "M_" + self.PROG_NAMES_MAIN[0]
                self.INCLUDE_SUB_PROGRAMS = True # Force generation of main program
                self.ProgStart(progname_main)
                for prog_call in self.PROG_NAMES_MAIN:
                    self.PROG.append('CALL JOB:%s' % (prog_call))
                    
                self.ProgFinish(progname_main)
            
            # Save the last program added to the PROG_LIST
            self.PROG = self.PROG_LIST.pop()
            progname_last = self.PROG_NAMES.pop()
            self.progsave(folder, progname_last, ask_user, show_result)
            #-------------------------
            #self.LOG = ''
            if len(self.PROG_FILES) == 0:
                # cancelled by user
                return
                
            first_file = self.PROG_FILES[0]
            folder_user = getFileDir(first_file)
            # progname_user = getFileName(self.FILE_SAVED)
            
            # Generate each program
            for i in range(len(self.PROG_LIST)):
                self.PROG = self.PROG_LIST[i]
                if show_result and i > 3:
                    show_result = False
                self.progsave(folder_user, self.PROG_NAMES[i], False, show_result)
                
        elif nfiles == 1:
            self.PROG = self.PROG_NAMES[0]
            self.progsave(folder, progname, ask_user, show_result)
            
        else:
            print("Warning! Program has not been properly finished")
            self.progsave(folder, progname, ask_user, show_result)

        if show_result and len(self.LOG) > 0:
            mbox('Program generation LOG:\n\n' + self.LOG)
            
            
    def ProgSendRobot(self, robot_ip="192.168.1.200", remote_path="/rbctrl/", robot_user="root", robot_pwd="elite2014"):
        """Send the progran to the robot"""
        if remote_path == "" or remote_path == "/":
            remote_path = "/rbctrl/"
            
        if robot_ip == "127.0.0.1":
            robot_ip = "192.168.1.200"
            
        if robot_user == "":
            robot_user = "root"

        if robot_pwd == "":
            robot_pwd = "elite2014"
            
        try:
            import pysftp           # robodk自带为ftp，elite支持sftp传输
        except ModuleNotFoundError as e:
            ShowMessage("pysftp module not found ,please use \"pip install pysftp\" to install pysftp module","module not found")
        
        def sftp_download(ip, user, pwd, local_path, remote_path):
            try:
                with pysftp.Connection(host = ip, username = user, password = pwd) as sftp:
                    sftp.put(local_path, remote_path)
            except Exception as e:
                ShowMessage(f"{local_path}\tdownload fail,\n {e}" )


        def get_file_name(file_name: str):
            file_list = file_name.split("/")
            return file_list[len(file_list)-1]

        
        print(f"robot_ip:{robot_ip},robot_user:{robot_user},robot_pwd:{robot_pwd}")
        for file in self.PROG_FILES:
            # ShowMessage(file)
            # ShowMessage(remote_path + get_file_name(file))

            sftp_download(robot_ip, robot_user, robot_pwd, file, remote_path + get_file_name(file))


        
    def setFrame(self, pose, frame_id, frame_name):
        """Change the robot reference frame"""
        if self.bFirstIns:
            self.bFirstIns = False
            self.addline('-->')

        self.User1 = pose

        # user_pose = ''
        # for i,k in enumerate(pose_2_xyzrpw(pose)):
        #     if i<3:
        #         k = k/1000.0
        #     if i > 2:
        #         k = k/180.0 * 3.14159
        #         k = round(k,5)
        #     user_pose = user_pose + str(k)
        #     if i < 5 :
        #         user_pose = user_pose + ','
        # # self.addline("SETPOSE V000 %s" % user_pose)
        # # self.addline("SETUSERFRAME USER #(1) V000")
        # self.addline('global user1')
        # self.addline("user1 = [%s]" % user_pose)
        # self.addline('# setting user frame')
        # self.bUseFrame = True
  

        # xyzwpr = Pose_2_Motoman(pose)
        # #self.ACTIVE_FRAME = None
        # self.POSE_FRAME = pose
        # self.RunMessage('Using %s (targets wrt base):' % (str(frame_name)), True)
        # self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        # fid = str(frame_name).split()[-1]
        # if fid.isdigit() and int(fid) <= 7:
        #     self.addline("// COORD #(%i)" % (int(fid)))
        #     self.addline("SETPOSE V000 %s" % tool_pose)
        #     self.addline("SETTOOLFRAME TOOL#(1) V000")
        #     self.addline('SETTOOLNUMBER TF=1')

    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        if self.bFirstIns:
            self.bFirstIns = False
            self.addline('-->')
        tool_pose = ''
        tool1_tmp = pose_2_xyzrpw(pose)
        for i,k in enumerate(pose_2_xyzrpw(pose)):
            if i < 3:
                k = k/1000.0
            if i > 2:
                k = k/180.0 * 3.14159
                k = round(k,5)
            tool_pose = tool_pose + str(k)
            if i < 5 :
                tool_pose = tool_pose + ','
        
        out_set_tcp_str = "set_tcp([%s])" % tool_pose  
        self.addline('    <ScriptNode scriptType="LINE_TYPE">')
        self.addline('      <Expression>')
        for d in out_set_tcp_str:
            self.addline('        <ExpressionCharCell char=\"'+d+'\" />')
        self.addline('      </Expression>')
        self.addline('    </ScriptNode>')

        str_out_tcp = '    <Comment comment="在 配置-TCP 中添加[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]并激活" typeName="Comment"/>' % (tool1_tmp[0],tool1_tmp[1],tool1_tmp[2],tool1_tmp[3],tool1_tmp[4],tool1_tmp[5])
        self.addline(str_out_tcp)


    def MoveJ(self, pose, joints, conf_RLF=None):
        if self.bFirstIns:
            self.bFirstIns = False
            self.addline('-->')
        if pose is not None:
            self.CurrMoveType = 'MoveJ'
            str_move_node = '    <MoveNode movementType=\"JOINT_MOVEMENT\" positionType=\"CARTESIAN_POSE\" tcpType=\"ACTIVE_TCP\">\n      <frameReference>Ctrl_base_frame</frameReference>\n      <jointSpeed>\n        <valueInSi>'+self.STR_VJ+'</valueInSi>\n      </jointSpeed>\n      <jointAcceleration>\n        <valueInSi>1.3962634015954636</valueInSi>\n      </jointAcceleration>'
            # 如果移动指令变化，添加新的MoveNode
            if self.PreMoveType != self.CurrMoveType:
                self.addline(str_move_node)
                str_move_node ='    </MoveNode>'
                self.addline(str_move_node)

            
            # 添加路点Node
            # """Add a joint movement"""
            self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
      
            pid = self.P_COUNT
            self.P_COUNT = self.P_COUNT + 1
            self.insertline(self.GetStrWayPoint(self.wp_name,self.P_COUNT,pose,joints))
            self.PreMoveType = self.CurrMoveType

    def PoseTrans(self,p1,p2):
        #p1 和 p2 均为robodk的pose类型（4*4矩阵）
        # 返回值格式为 X="564.4002963141432" Y="-147.49998942204118" Z="497.4985318691541" RX="-3.1415913542124887" RY="4.371139000182553E-8" RZ="-1.5707963267948397" 
        p = p1.copy()
        new_p = pose_2_xyzrpw(p.__mul__(p2))
        out_pose_str = 'X="'+str(new_p[0])+'\" '
        out_pose_str += 'Y="'+str(new_p[1])+'\" '
        out_pose_str += 'Z="'+str(new_p[2])+'\" '
        out_pose_str += 'RX="%.4f\" ' % (new_p[3]/180*3.14159)
        out_pose_str += 'RY="%.4f\" ' % (new_p[4]/180*3.14159)
        out_pose_str += 'RZ="%.4f\"' % (new_p[5]/180*3.14159)
        return out_pose_str
        

    def MoveL(self, pose, joints, conf_RLF=None):
        if self.bFirstIns:
            self.bFirstIns = False
            self.addline('-->')
            
        """Add a linear movement"""
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
        # sleep(10000)

        if pose is not None:

            self.CurrMoveType = 'MoveL'
            str_move_node = '    <MoveNode movementType="LINEAR_MOVEMENT" positionType="CARTESIAN_POSE" tcpType="ACTIVE_TCP" typeName="MoveL">\n      <frameReference>Ctrl_base_frame</frameReference>\n      <cartesianSpeed>\n        <valueInSi>'+self.STR_V+'</valueInSi>\n      </cartesianSpeed>'
            if self.PreMoveType != self.CurrMoveType:
                self.addline(str_move_node)
                str_move_node ='    </MoveNode>'
                self.addline(str_move_node)

        # 添加路点
            pid = self.P_COUNT
            self.P_COUNT = self.P_COUNT + 1
            self.insertline(self.GetStrWayPoint(self.wp_name,self.P_COUNT,pose,joints))
            self.PreMoveType = self.CurrMoveType

    def GetStrWayPoint(self, wp_name1,count, pose, joints):
        user_tmp = pose_2_xyzrpw(self.User1)
        user_tmp[0] = user_tmp[0]/1000
        user_tmp[1] = user_tmp[1]/1000
        user_tmp[2] = user_tmp[2]/1000
        user_tmp[3] = user_tmp[3]/180*3.14159
        user_tmp[4] = user_tmp[4]/180*3.14159
        user_tmp[5] = user_tmp[5]/180*3.14159



        out_wp_name = wp_name1+str(count)
        out_joint = ''
        for i in range(0, 5):
            out_joint += str(joints[i]/180*3.14159) + ', '
        out_joint += str(joints[5]/180*3.14159)

        out_pose = self.PoseTrans(self.User1, pose)
        str_wp_node = '      <WaypointNode name=\"'+out_wp_name+'\" positionNodeType="FIXED_POSITION" kinematicFlag="-1" customTransitionRadius="true" advancedPositionOptionType="SPEED_AND_ACCELERATION">\n        <transitionRadius>\n          <valueInSi>' + \
            self.STR_PL+'</valueInSi>\n        </transitionRadius>\n        <internalPosition>\n          <jointPositions joints='
        str_wp_node += '\"'+out_joint+'\" />\n'
        str_wp_node += '          <toolPose ' + \
            out_pose+'/>\n        </internalPosition>\n'
        str_wp_node +='        <jointSpeed>\n          <valueInSi>'+self.STR_VJ+'</valueInSi>\n        </jointSpeed>\n        <jointAcceleration>\n          <valueInSi>'+self.STR_ACC_JOINT+'</valueInSi>\n        </jointAcceleration>\n'
        str_wp_node += '      <cartesianSpeed>\n        <valueInSi>' + \
            self.STR_V+'</valueInSi>\n      </cartesianSpeed>\n'
        str_wp_node += '        <cartesianAcceleration>\n          <valueInSi>' + \
            self.STR_ACC_CART+'</valueInSi>\n        </cartesianAcceleration>\n'
        #str_wp_node += '        <Pose key="baseToFrame">\n          <Position key="posePosition">\n            <Length key="X" value="%.4f" unit="M"/>\n            <Length key="Y" value="%.4f" unit="M"/>\n            <Length key="Z" value="%.4f" unit="M"/>\n          </Position>\n          <Rotation key="poseRotation">\n            <Angle key="RX" value="%.4f" unit="RAD"/>\n            <Angle key="RY" value="%.4f" unit="RAD"/>\n            <Angle key="RZ" value="%.4f" unit="RAD"/>\n          </Rotation>\n        </Pose>\n' %(user_tmp[0],user_tmp[1],user_tmp[2],user_tmp[3],user_tmp[4],user_tmp[5])
        str_wp_node += '     </WaypointNode>'
        return str_wp_node
              

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        if self.bFirstIns:
            self.bFirstIns = False
            self.addline('-->')
            
        """Add a circular movement"""
        # Important to control the maximum lines per program and not save last target on new program
        # Reserve enough space for the full circular movement
        self.page_size_control()
        if pose1 is not None:
          
            pid = self.V_COUNT
            self.V_COUNT = self.V_COUNT + 1
            target_pose1 = '['
            for i,k in enumerate(pose_2_xyzrpw(pose1)):
                if i<3:
                    k = k/1000.0
                if i > 2:
                    k = k/180.0 * 3.14159
                    k =  '%.5f' % k
                target_pose1 = target_pose1 + str(k)
                if i < 5 :
                    target_pose1 = target_pose1 + ','
            target_pose1 = target_pose1+']'
            
            target_pose2 = '['
            for i,k in enumerate(pose_2_xyzrpw(pose2)):
                if i<3:
                    k = k/1000.0
                if i > 2:
                    k = k/180.0 * 3.14159
                    k =  '%.5f' % k
                target_pose2 = target_pose2 + str(k)
                if i < 5 :
                    target_pose2 = target_pose2 + ','
            target_pose2 = target_pose2+']'


            # movec(p_via, p_to, a=0, v=0, r=0, mode=0)
            if self.bUseFrame:
                self.addline('movec(pose_trans(user1,%s),pose_trans(user1,%s),a=1.05,%s,%s,mode = 1)' %(target_pose1,target_pose2,self.STR_V,self.STR_PL))
            else:
                self.addline('movec(%s,%s,a=1.05,%s,%s,mode = 1)' %(target_pose1,target_pose2,self.STR_V,self.STR_PL))
          
            # pid = self.V_COUNT
            # self.V_COUNT = self.V_COUNT + 1
            # target_pose = ''
            # for i,k in enumerate(pose_2_xyzrpw(pose2)):
            #     if i > 2:
            #         k = k/180.0 * 3.14159
            #         k =  '%.5f' % k
            #     target_pose = target_pose + str(k)
            #     if i < 5 :
            #         target_pose = target_pose + ','
            # self.addline('SETPOSE V%03i ' % pid+target_pose)
            # self.addline2('MOVC V%03i %s %s ACC=50 DEC=50' % (pid, self.STR_V, self.STR_PL))

    def Pause(self, time_ms):
        """Pause the robot program"""
        self.addline("TIMER T=%s" % str(time_ms/1000))
        pass

    def setSpeed(self, speed_mms):
        if self.bFirstIns:
            self.bFirstIns = False
            self.addline('-->')
            
        """Changes the robot speed (in mm/s)"""
        speedl = max(0.01,min(speed_mms,3000.0)) 
        #self.STR_V = "v=%.iMM/S" % speedl
        self.STR_V = str(speedl/1000.0)

    def setSpeedJoints(self, speed_degs):
        if self.bFirstIns:
            self.bFirstIns = False
            self.addline('-->')
            
        """Changes the robot joint speed (in percentage)"""
        speedj = max(1.0, min(speed_degs,400.0)) 
        speedj = int(round(speedj))
        self.STR_VJ = str(speedj/180*3.14159)

    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.STR_ACC_CART = '%.3f' % (accel_mmss/1000)
        #self.addlog('Set acceleration not defined')

    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.STR_ACC_JOINT = '%.3f' % (accel_degss/1000)

    def setZoneData(self, zone_mm):
        if self.bFirstIns:
            self.bFirstIns = False
            self.addline('-->')
            
        """Changes the zone data approach (makes the movement more smooth)"""
        if zone_mm < 0:
            self.STR_PL = '0'
        else:
            self.STR_PL = '%.3f' % (round(min(zone_mm, 150))/1000.0)

    def setDO(self, io_var, io_value):
        """Set a Digital Output"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'OT#(%s)' % str(io_var)        
        if type(io_value) == str:
            if io_value.upper() != 'ON' and io_value.upper() != 'OFF':
                io_value = ''
        
        # at this point, io_var and io_value must be string values
        #DOUT OT#(2) ON
        self.addline('DOUT %s %s' % (io_var, str(io_value)))

    def setAO(self, io_var, io_value):
        """Set an Analog Output"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'AO#(%s)' % str(io_var)
        if type(io_value) != str:
            if io_value > 10:
                io_value = 10
        self.addline('AOUT %s %s' % (io_var, str(io_value)))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IN#(%s)' % str(io_var)
        if type(io_value) != str:
            if io_value > 10:
                io_value = 10
        if timeout_ms > 0:
            self.addline('WAIT %s=%s T=%s' % (io_var, str(io_value),str(round(timeout_ms/1000,3))))
        else:
            self.addline('WAIT %s=%s' % (io_var, str(io_value)))
        # if type(io_var) != 
        pass

    def RunCode(self, code, is_function_call = False):
        """Adds code or a function call"""
        if is_function_call:
            code = get_safe_name(code)
            #if code.startswith("ArcStart"):
                #return
                
            # default program call
            code.replace(' ','_')
            self.addline('CALL JOB:%s' % (code))
        else:
            #if code.endswith(';'):
                #code = code[:-1]
            self.addline(code)
        
    def RunMessage(self, message, iscomment = False):
        """Add a comment or a popup message"""
        if iscomment:
            for i in range(0,len(message), 73):
                i2 = min(i + 73, len(message))
                self.addline("// %s" % message[i:i2])
                
        else:
            for i in range(0,len(message), 25):
                i2 = min(i + 25, len(message))
                self.addline('TPWRITE %s' % message[i:i2])

        
# ------------------ private ----------------------
    def page_size_control(self, reserved_lines=0):
        # Reserved lines is used for MoveC
        if (self.LINE_COUNT + reserved_lines) >= self.MAX_LINES_X_PROG:
            self.ProgFinish(self.PROG_NAME, True)
            self.ProgStart(self.PROG_NAME, True)


    def addline(self, newline, movetype = ' '):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        self.page_size_control()        
        self.LINE_COUNT = self.LINE_COUNT + 1
        self.PROG.append(newline)

    def insertline(self, newline, movetype = ' '):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        self.page_size_control()        
        self.LINE_COUNT = self.LINE_COUNT + 1
        self.PROG.insert((len(self.PROG)-1),newline)

    def addline2(self, newline, movetype = ' '):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        self.page_size_control()        
        self.LINE_COUNT = self.LINE_COUNT + 1
        self.PROG_TARGETS.append(newline)  

    

    def addlog(self, newline):
        """Add a log message"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        self.LOG = self.LOG + newline + '\n'

