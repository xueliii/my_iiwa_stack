import sys
import socket
import random
import time
from threading import Thread
import logging
from copy import copy
import rospy

# logging.basicConfig(level=logging.INFO,
#                     format='[%(levelname)s] (%(threadName)-10s) %(message)s'
#                     )

# This class in-charge of sending and receiving data from Kuka iiwa
class operator():
    def __init__(self,global_db,iiwa_ip="172.31.1.147",iiwa_port=30009,send_rate=50,recv_rate=50):
        self.global_db=global_db
        self.iiwa_ip = iiwa_ip
        self.iiwa_port=iiwa_port
        self.sock=socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self.sock.setblocking(0)

        #self.my_address=("172.31.1.38",12358)
        # self.sock.bind(self.my_address)
        self.send_server_thread = None
        self.receive_server_thread = None
        self.disconnect = False

        self.send_rate = send_rate
        self.recv_rate = recv_rate

# Method: Sends message to Kuka iiwa
#   Input: Message of byte type
    def send_cont(self,MESSAGE=b'123456789'):
        UDP_IP = self.iiwa_ip
        UDP_PORT = self.iiwa_port

        #print("UDP target IP:", UDP_IP)
        #print("UDP target port:", UDP_PORT)
        # print("send message:", MESSAGE)

        # sock = socket.socket(socket.AF_INET, # Internet
        #                      socket.SOCK_DGRAM) # UDP
        self.sock.sendto(MESSAGE, (self.iiwa_ip, self.iiwa_port))

# Method: Create a separate thread to run send_thread method. Thread should be non-daemonic
    def send_server_start(self):
        self.send_server_thread=Thread(target=self.send_thread)
        self.send_server_thread.daemon = False
        self.send_server_thread.start()
        #print("send_server start!")

# Method: Create a separate thread to run receive_thread method. Thread should be non-daemonic
    def receive_server_start(self):
        self.receive_server_thread = Thread(target=self.receive_thread)
        self.receive_server_thread.daemon = False
        self.receive_server_thread.start()
        #print("receive_server start!")

# Method: Prepares message in Byte type and calls send_cont method to send message to Kuka iiwa
    def send_thread(self):
        # a = b"abcdefghijklmn"
        while True:
            try:
                # print(self.disconnect)
                iiwa_sp_str=self.global_db.form_sp_Str()
                # a = 1
                rospy.logdebug(iiwa_sp_str)
                rand_msg = bytes(iiwa_sp_str).encode('utf-8')
                self.send_cont(rand_msg)
                if self.disconnect:
                    break
                time.sleep(1./self.send_rate)
            except socket.error:
                if self.disconnect:
                    rospy.logwarn(sys.exc_info()[0])
                    rospy.logwarn(sys.exc_info()[2])
                    break
                else:
                    pass

            except Exception:
                rospy.logerr(sys.exc_info()[0])
                rospy.logerr(sys.exc_info()[2])


# Method: Receives message in Byte type and allocate data to Database in global_database class
    def receive_thread(self,rate=50):
        while True:
            try:
                data, client = self.sock.recvfrom(2000)
                if not data:
                    # print("no data received")
                    time.sleep(0.1)
                    break

                self.global_db.r_iiwaStr=data
                self.global_db.r_iiwaFltLs=self.iiwaStr_to_double(data)

                self.global_db.r_iiwa_o_actreq = self.global_db.r_iiwaFltLs[0]
                self.global_db.r_iiwa_o = self.global_db.r_iiwaFltLs[1]
                self.global_db.r_iiwa_o_force = self.global_db.r_iiwaFltLs[2]
                self.global_db.r_iiwa_o_spd = self.global_db.r_iiwaFltLs[3]

                self.global_db.first_update_done=1
                time.sleep(1./self.recv_rate)

                if self.disconnect:
                    break

            except socket.error:
                if self.disconnect:
                    rospy.logwarn(sys.exc_info()[0])
                    rospy.logwarn(sys.exc_info()[2])
                    break
                else:
                    pass

            except Exception:
                rospy.logerr(sys.exc_info()[0])
                rospy.logerr(sys.exc_info()[2])

# Method: Set flag (self.disconnect) to True to stop the while loops in the send and receive threads to stop the programme
    def Disconnect(self):
        if (self.disconnect != True):
            self.disconnect = True
            disconnect_timeout = 5
            if (self.receive_server_thread != None):
                self.receive_server_thread.join(disconnect_timeout)
            if (self.send_server_thread != None):
                self.send_server_thread.join(disconnect_timeout)
            rospy.logwarn("UDP connection for gripper disconnected!")
        return True

# Method: Convert bytes to list of floating values
#   Input: Received data in bytes. (string data delimited with space and ends with \0)
#   Output: List of floating values
    @staticmethod
    def iiwaStr_to_double(iiwaStr):
        iiwaStr=iiwaStr.decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
        # print(iiwaStr)
        float_list=iiwaStr.split(" ")
        # print(float_list)
        for i in range(len(float_list)):
            float_list[i]=round(float(float_list[i]),4)
        return float_list

# Class: Manages a database of data
class global_database():
    # this database is shared with iiwa through udp
    def __init__(self):
        #functional
        self.first_update_done=0

        #iiwaStr
        self.r_iiwaStr=""
        self.r_iiwaFltLs=[]

        self.iiwa_o_sp=6
        self.iiwa_o_force=5
        self.iiwa_o_spd=200
        self.iiwa_o_actreq=11 # 9 - basic mode 11 - pinch mode

        #r_iiwa_xx == read from iiwa; r means read from iiwa
        self.r_iiwa_o_actreq=0
        self.r_iiwa_o=0
        self.r_iiwa_o_force=0
        self.r_iiwa_o_spd=0

# Method: Forms a string of values based on the database
#   Output: Returns a string of data with space as delimiter
    def form_sp_Str(self):
        # Robotiq 3-finger gripper position PosReq [0,255] and grip force DigitalOutput [0,255]
        sendStr = str(self.iiwa_o_actreq) + " " + str(self.iiwa_o_sp) + " "
        sendStr += str(self.iiwa_o_force) + " " + str(self.iiwa_o_spd)
        # rospy.loginfo("Sending Cont: " + sendStr)
        return sendStr
