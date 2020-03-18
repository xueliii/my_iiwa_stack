import time
from EK_RemoteControl import *
from threading import Thread
from copy import copy
import logging
import rospy

'''logging with line number and time'''
# logging.basicConfig(level=rospy.loginfo,
#                     format='%(asctime)s [%(levelname)s] (%(threadName)-10s) (%(lineno)d): %(message)s'
#                     ,datefmt='%H:%M:%S')

''' Logging without line number '''
# logging.basicConfig(level=rospy.loginfo,
#                     format='[%(levelname)s] (%(threadName)-10s) %(message)s'
#                     )

class Interface:
    # threadcount = 0

    def __init__(self,iiwa_ip="172.31.1.147",iiwa_port=30009,send_rate=50,recv_rate=50):
        self._iiwa_ip = iiwa_ip
        self._iiwa_port=iiwa_port
        self._send_rate = send_rate
        self._recv_rate = recv_rate
        self.db = global_database()
        self.op = operator(self.db,self._iiwa_ip,self._iiwa_port\
          ,self._send_rate,self._recv_rate)
        # start to receive data from iiwa . Make sure YW_UDP_Server.java is running on the IIWA
        self.op.receive_server_start()
        # start to send msg to iiwa. This is for build continues connection between IIWA.
        self.op.send_server_start()
        rospy.loginfo("GRIPPER INTERFACE START!")
        # self.target_gripper = 1

# Static Method, to impose a custom limit on input and return input
    @staticmethod
    def clamp(input,min,max):
        if(input>=0):
            if(input>max):
                input=max
            elif(input<min):
                input=min
        if(input<0):
            if(input>-min):
                input=-min
            elif(input<-max):
                input=-max

        return input

    def set_gripper_actreq(self,val=11):
        self.db.iiwa_o_actreq = val

    def get_gripper_actreq(self):
        return copy(int(self.db.r_iiwa_o_actreq))

    def set_gripper_pos(self, val):
        '''
        gripper control (Blocking wait)
        :param val: False:RELEASE, True:GRIP
        :return: None

        val = true or false

        '''
        #6 open val=false
        #113 close val=true
        # print("set gripper",val)
        self.db.iiwa_o_sp = val

        # if self.target_gripper != val:
        #     self.target_gripper = val
        # if val:
        #     # self.gripper.close_gripper()
        #     self.db.iiwa_o_sp=113
        # else:
        #     # pose_read=self.db.get_read_poses()
        #     self.db.iiwa_o_sp=6
        #     # self.gripper.open_gripper()

    def get_gripper_pos(self):
        """
        get gripper status
        :return: Boolean (0 opened / 1 closed)
        """
        #6 open val=false
        #113 close val=true
        # return self.db.r_iiwa_o>90
        return copy(int(self.db.r_iiwa_o))

    def set_gripper_force(self, val):
        self.db.iiwa_o_force = val

    def get_gripper_force(self):
        return copy(int(self.db.r_iiwa_o_force))

    def set_gripper_spd(self,val):
        self.db.iiwa_o_spd = val

    def get_gripper_spd(self):
        return copy(int(self.db.r_iiwa_o_spd))

# Method: To send signal to EK_RemoteControl.py to stop the send and receive thread to exit safely
#         ALWAYS DO THIS AT THE END OF THE PROGRAMME. OTHERWISE IT WILL NOT STOP RUNNING.
    def disconnect(self):
        self.op.Disconnect()
