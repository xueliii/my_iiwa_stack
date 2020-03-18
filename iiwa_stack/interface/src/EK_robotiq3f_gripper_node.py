#! /usr/bin/env python

'''
Author: Eric Kwok
Email: mpeknt@nus.edu.sg
'''

import sys
import rospy
import actionlib
import iiwa_msgs.msg
import numpy as np
import time
from EK_gripper_control import *
from threading import Thread, Lock

# A Parent class with 2 sub class: Gripper_action (for action server) and Gripper_pub_sub (Publisher and listener)
class Gripper_control(object):
    rob = Interface(iiwa_ip="172.31.1.147",iiwa_port=30009,send_rate=50,recv_rate=50)
    # To create an instance of class Interface for controlling the gripper_result
    grip_pos_uncertainty = 1 # Uncertainty in the readings of the gripper pos
    rospy.init_node('Robotiq3f_gripper') # initialize ros node
    r = rospy.Rate(15)      # loop rate for action server and publisher
    rospy.on_shutdown(rob.disconnect) # shutdown hook. IMPORTANT TO ALWAYS CALL ROB.DISCONNECT BEFORE ENDING
    set_gripper_lock = Lock() # Initialize lock for set_gripper method

    def __init__(self):
        pass

# Method: Set gripper values - actreq (action request), grip_pos (gripping position), grip_force (gripping force), grip_spd (gripping speed)
#   Input: target - a message of type GripperControl (Refer to the message file in iiwa_msgs/msg)
#          stop - if stop if False, the target set by user will be sent to the gripper. Otherwise, it will use current val as target
    @classmethod
    def set_gripper(cls, target=[], stop=False):
        cls.set_gripper_lock.acquire()
        if (stop == False):
            cls.rob.set_gripper_actreq(target.actreq)
            cls.rob.set_gripper_pos(target.grip_pos)
            cls.rob.set_gripper_force(target.grip_force)
            cls.rob.set_gripper_spd(target.grip_spd)
        else:
            # rospy.loginfo('%s : Executing, gripper_pos = %i' %(cls._action_name,\
            #  target.grip_pos))
            cls.rob.set_gripper_actreq(cls.rob.get_gripper_actreq())
            cls.rob.set_gripper_pos(cls.rob.get_gripper_pos())
            cls.rob.set_gripper_force(cls.rob.get_gripper_force())
            cls.rob.set_gripper_spd(cls.rob.get_gripper_spd())
        cls.set_gripper_lock.release()

# Method: Get gripper status
#   Input: message of type GripperControl
    @classmethod
    def get_gripper(cls,msg):
        msg.actreq = int(cls.rob.get_gripper_actreq())
        msg.grip_pos = int(cls.rob.get_gripper_pos())
        msg.grip_force = int(cls.rob.get_gripper_force())
        msg.grip_spd = int(cls.rob.get_gripper_spd())
        return msg

# Method: Initialize gripper value
    @classmethod
    def initialization(cls,actreq=11,pos=6,force=5,spd=200):
        wait_time = 18
        rospy.loginfo("Initializing! Please wait for %.1fs..." %wait_time)
        cls.rob.set_gripper_actreq(actreq)
        cls.rob.set_gripper_pos(pos)
        cls.rob.set_gripper_force(force)
        cls.rob.set_gripper_spd(spd)
        rospy.Rate(1./wait_time).sleep()
        rospy.loginfo("Gripper initialized!")

# Method: Impose limit on the grip_pos based on actreq status
#   Input: Message of type GripperControl
    def grip_pos_limit(self,msg):
        if (msg.actreq == 11):
            msg.grip_pos = self.rob.clamp(msg.grip_pos,6,113)
        elif (msg.actreq == 9):
            msg.grip_pos = self.rob.clamp(msg.grip_pos,6,240)
        return msg

class Gripper_action(Gripper_control):
    # create messages that are used to publish feedback/result
    _feedback = iiwa_msgs.msg.GripperControlFeedback()
    _result = iiwa_msgs.msg.GripperControlResult()

    def __init__(self,name=rospy.get_name()+"/action"):
        self.gripped = False
        self._action_name = name # Name of the action server. Default is Robotiq3f_gripper
        self._as = actionlib.SimpleActionServer(self._action_name,\
         iiwa_msgs.msg.GripperControlAction, execute_cb=self.execute_cb, auto_start=False)
         # Create the action server object with execute_cb method as callback
        self._as.start()
        self._watch_time = 1.5
        # The amount of time to check if the griper is stationary before declaring
        # goal has reached
        self._gripper_msg = iiwa_msgs.msg.GripperControl()

# Method: Used for callbacks to set gripper values and to obtrain feedbacks from gripper
    def execute_cb(self, goal):
        rospy.loginfo(self._action_name + " callback!")
        self.gripped = False
        target = goal.gripper_goal

        self.grip_pos_limit(target)

        success = True
        self.r.sleep()
        self._feedback.gripper_feedback = self.get_gripper(self._gripper_msg)
        self._as.publish_feedback(self._feedback) # Publish feedback



        # If the target actreq is not equal to the current actreq, set to target
        # actreq and sleep for 1 second to give it time to prepare
        if target.actreq != int(self.rob.get_gripper_actreq()):
            self.rob.set_gripper_actreq(target.actreq)
            rospy.loginfo("Waiting for %.1fs to change actreq..."%self._watch_time)
            rospy.Rate(1./self._watch_time).sleep()

        # Execution of action
        # While loop, loop until target is reached or if action is preempted
        time0 = time.time()
        pos0 = self._feedback.gripper_feedback.grip_pos
        while (not self.gripped):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)

                self.set_gripper(stop=True) # Stop the gripper movement by setting the current pos as target

                self._as.set_preempted()
                success = False
                break

            elif (time.time()-time0 > self._watch_time):
                if (np.abs(self.rob.get_gripper_pos()-pos0) <=\
                 self.grip_pos_uncertainty):
                    self.gripped = True
                    rospy.loginfo("gripped successful")
                else:
                    time0 = time.time()
                    pos0 = self.rob.get_gripper_pos()
                    self.set_gripper(target)
            else:
                self.set_gripper(target)

            self._feedback.gripper_feedback = self.get_gripper(self._gripper_msg)
            self._as.publish_feedback(self._feedback) # Publish feedback
            self.r.sleep()

        if (success):  # publish result if it successfully reaches goal
            self._result.gripper_result = self._feedback.gripper_feedback
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

class Gripper_pub_sub(Gripper_control):

# sub_name is the topic name of the listener; pub_name is for publisher
    def __init__(self,pub_name=rospy.get_name()+"/status_publisher",sub_name=rospy.get_name()+"/control_listener"):
        self.pub = rospy.Publisher(pub_name, iiwa_msgs.msg.GripperControl, queue_size=10)
        self.sub = rospy.Subscriber(sub_name,iiwa_msgs.msg.GripperControl, self.gripper_listener_cb)
        self.pub_msg = iiwa_msgs.msg.GripperControl()
        self.sub_msg = iiwa_msgs.msg.GripperControl()
        self.pub_Server = Thread(target=self.gripper_publisher)

# For publishing gripper's status
    def gripper_publisher(self):
        try:
            while not rospy.is_shutdown():
                self.pub_msg = self.get_gripper(self.pub_msg)
                self.pub.publish(self.pub_msg)
                self.r.sleep()

        except rospy.ROSInterruptException:
            pass
        except Exception:
            rospy.logerr(sys.exc_info()[0])
            rospy.logerr(sys.exc_info()[2])
        finally:
            rospy.logwarn("Gripper publisher stopped")

# Callback method to listen for target messages and send to gripper
    def gripper_listener_cb(self,target):
        self.set_gripper(target=self.grip_pos_limit(target))

    def start_publisher(self):
        self.pub_Server.start()


if __name__ == '__main__':
    try:
        Gripper_control.initialization() # Send an intial pose to Kuka Iiwa and get updated on initial status of the robot
        actionServer = Gripper_action() # Start the action server
        sp = Gripper_pub_sub() # Initialize the listener and publisher
        sp.start_publisher() # Start running the publisher
        rospy.spin()
        if sp.pub_Server.isAlive():
            sp.pub_Server.join(5)

    except rospy.ROSInterruptException:
        Gripper_control.rob.disconnect()
        rospy.logwarn("ROS Interrupted!")
    except Exception:
        Gripper_control.rob.disconnect()
        rospy.logerr(sys.exc_info()[0])
        rospy.logerr(sys.exc_info()[2])
