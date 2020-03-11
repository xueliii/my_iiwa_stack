#! /usr/bin/env python

'''
Author: Eric Kwok
Email: mpeknt@nus.edu.sg
Instruction:
    To use these functions, make sure that the EK_robotiq3f_gripper_node.py is
    running. Start roscore and use rosrun to run the script to initialize the
    node, which starts the ros action server, publisher to publish the gripper
    status and a listener to listen to command for the gripper.
'''

import rospy
import actionlib
import iiwa_msgs.msg
import time
import numpy as np

class Gripper_API(object):
    node_no = 0

    def __init__(self,init_node=False):
        self.node_no += 1
        self.node_name = 'Gripper_node_' + str(self.node_no)
        self.result = iiwa_msgs.msg.GripperControl()
        self.client = self._init_client(self.node_name,init_node)
        self.action_done = False    # For eval() in cognitive architecture
        self._goal_msg = iiwa_msgs.msg.GripperControlGoal()
        self._time0 = time.time()
        self._pos0 = None


# Method: To initialize action clent with or wihtout init_node
# Input: name - name of the node
#        init_node - True (start node upon initialization)
# Output: Returns action client object
    def _init_client(self,name,init_node=False):
        try:
            if init_node:
                rospy.init_node(name)
            client = actionlib.SimpleActionClient('/Robotiq3f_gripper/action',\
             iiwa_msgs.msg.GripperControlAction)
            return client
        except rospy.ROSInterruptException:
            rospy.logwarn(self.node_name + " initialization interrupted...")
            raise rospy.ROSInterruptException
        except Exception:
            # rospy.logerr(self.node_name)
            rospy.logerr(sys.exc_info()[0])
            rospy.logerr(sys.exc_info()[2])

# Method: To open or close gripper
# Input: val [0 ,1] - open or close (Either input val or exact_val, not both)
#        actreq (optional)
#        spd - grippping speed (optional)
#        force - gripping force (optional)
#        exact_val [0,255] - exact gripping pos (optional)
#        blocking [True,False] - If True, it will block until action is finish
# Output: goal_achieved - The achieved goal (only ouput if blocking), in the
#         format goal_achieved.gripper_result.actreq
    def open_close_gripper(self,val=None,actreq=11,spd=100,force=10,exact_val=None,\
     blocking=False):
        self.action_done = False
        try:
            if (self.client == False):
                rospy.logerr("Action client not setup.")
                return False

            elif (val != 0 and val != 1 and exact_val == None):
                rospy.logerr("Wrong input value. Please input 0 (open) or 1 (close).")
                return False

            elif (val == None and exact_val == None):
                rospy.logerr("Please input val or exact_val")
                return False

            elif (val != None and exact_val != None):
                rospy.logerr("Please input either val or exact_val")
                return False

            else:
                rospy.loginfo("Waiting for action client")
                self.client.wait_for_server()

                self._goal_msg.gripper_goal.actreq = int(actreq)
                self._goal_msg.gripper_goal.grip_force = int(force)
                self._goal_msg.gripper_goal.grip_spd = int(spd)

                if (exact_val != None):
                    self._goal_msg.gripper_goal.grip_pos = exact_val
                elif val:
                    self._goal_msg.gripper_goal.grip_pos = 255
                else:
                    self._goal_msg.gripper_goal.grip_pos = 0

                if int(actreq) != self.gripper_status()[1]:
                    wait_time = 2
                    rospy.loginfo("Sending target and waiting for %.1fs to change actreq"\
                    %wait_time)
                    self.client.send_goal(self._goal_msg)
                    rospy.Rate(1./wait_time).sleep()
                else:
                    rospy.loginfo("sending")
                    self.client.send_goal(self._goal_msg)

                if blocking:
                    goal_achieved = self.client.wait_for_result()
                    self.result = self.client.get_result()
                    self.action_done = True
                    return goal_achieved


        except rospy.ROSInterruptException:
            rospy.logwarn(self.node_name + " interrupted...")
            pass
        except Exception:
            # rospy.logerr(self.node_name)
            rospy.logerr(sys.exc_info()[0])
            rospy.logerr(sys.exc_info()[2])

# Method: Evaluate gripper whether it has stopped moving
# Input: watch_time - the length of time to check whether the gripper is stationary
#                     before confirming it has gripped something (optional)
#        watch_time_ub - the upper bound for the watch time (optional)
#        grip_pos_uncertainty - The uncertainty in the grip pos (optional)
# Output: [True,False] - True if it has stopped moving
    def eval_gripper_motion(self,watch_time=1.5,watch_time_ub=2.5,\
     grip_pos_uncertainty=1):
        if self.action_done:
            return True
        elif self._pos0 == None:
            self._pos0 = self.gripper_status()[0]
            self._time0 = time.time()
            if self._pos0 == False:
                rospy.logerr("Unable to get status from gripper_status method")
            return False
        elif time.time()-self._time0 > watch_time_ub:
            self._pos0 = self.gripper_status()[0]
            self._time0 = time.time()
            return False
        elif time.time()-self._time0 < watch_time:
            return False
        elif np.abs(self.gripper_status()[0]-self._pos0) > grip_pos_uncertainty:
            self._time0 = time.time()
            self._pos0 = self.gripper_status()[0]
            if self._pos0 == False:
                rospy.logerr("Unable to get status from gripper_status method")
            return False
        else:
            self.action_done = True
            return True

    # # TODO: Change to other method to stop gripper
    # def stop_action(self):
    #     try:
    #         msg = iiwa_msgs.msg.GripperControl()
    #         msg = Gripper_control.get_gripper(msg)
    #         self.open_close_gripper(exact_val=msg.grip_pos)
    #         # status = self.gripper_status()
    #         # self.open_close_gripper(exact_val=status[0])
    #     except rospy.ROSInterruptException:
    #         rospy.logwarn(self.node_name + " interrupted...")
    #         raise rospy.ROSInterruptException
    #     except:
    #         rospy.logerr(self.node_name + " unexpected error...")
    #         raise

# Method: To check the status of the gripper
# Input: timeout [s] - No. of seconds to wait for publisher before giving up (optional)
# Output: (grip_pos, actreq, grip_force, grip_spd) tuple
    def gripper_status(self,timeout=5):
        try:
            topic_name = '/Robotiq3f_gripper/status_publisher'
            _gripper_status = rospy.wait_for_message(topic_name,\
             iiwa_msgs.msg.GripperControl,timeout)
            if (_gripper_status == False):
                rospy.logwarn("Timeout after waiting for %is for the message from "\
                 %(timeout) + topic_name)
                return False
            return _gripper_status.grip_pos, _gripper_status.actreq,\
             _gripper_status.grip_force, _gripper_status.grip_spd

        except rospy.ROSInterruptException:
            rospy.logwarn(self.node_name + " interrupted...")
            pass
        except Exception:
            # rospy.logerr(self.node_name)
            rospy.logerr(sys.exc_info()[0])
            rospy.logerr(sys.exc_info()[2])

if __name__ == "__main__":
    G = Gripper_API(init_node=True)
    while True:
        val = int(input("Enter gripper val: "))
        G.open_close_gripper(val,actreq=11)
        while True:
            print(G.eval_gripper_motion())
