#!/usr/bin/env python

# from hwinterface import HWInterface
from lx_iiwa14 import iiwa14
from EK_gripper_API import Gripper_API
from iiwa_ros.control_mode_py import SetControlMode
import time
from threading import Thread
import copy

# import roslib
import sys
import os
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

HOMEJ = [-115.48, 25, 0, 90, -0.01, -115, 0]
TOOL_RANGE = [-3.1, 3.1]

TEST_joint = [[560/1000,0/1000,460/1000,0,0,-2.9], \
    [-131, -50, 107, -53, 156, 80, -136]]

TEST_pose = [[0.08, 0.167, 0.6, -0.597, -0.38, 0.38],\
    [0, -0.19, 0.784, 1.57, 0, 3.14], \
    [0.08, 0.17, 0.5, -0.6, -0.38, 0.38]]

class ActionHelper():
    def __init__(self):

        # interface init
        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('SimpleMoveit', anonymous=False)

        self.itf = iiwa14()
        self.gripper = Gripper_API()
        self.control_mode = SetControlMode()

        # robot init
        print('============ Homing robot...')
        # self.gripper...
        self.move_home()
        self.itf.rate.sleep()
        # self.state_gripper, self.target_gripper = 0, 0
        # self.process_gripper = Thread(target=self._set_gripper)
        self.target_pose = self.get_tcp_pose()
        self.target_tool = self.get_tool_rot()
        print('Robot ready!')

        # force monitoring
        # self.process_force = Thread(target=self._watch_force)
        # self.input_force = False
        # self.process_push = Thread(target=self._watch_push)
        # self.push_stopped = 0

    def d2r(self, joint_degree):
        return [round(i*3.14/180, 3) for i in joint_degree]


    def move_home(self):
        """
        move back to home (Blocking)
        :return: None
        """
        self.itf.go_to_joint_state(self.d2r(HOMEJ))

    def move_joint(self, joint):
        """
        move back to home (Blocking)
        :return: None
        """
        self.itf.go_to_joint_state(joint)

    def get_tcp_pose(self):
        """
        get eff pose
        :return: list [x y z R P Y] (meter, radian)
        """
        return self.itf.get_current_pose()

    def move_tcp_relative(self, pose_rel):
        """
        move eff to relative pose
        :param pose: relative differences in [x y z R P Y] (meter, radian)
        :return: None
        """
        # self.itf.add_pose_tool(m3d.Transform(pose))
        pose = self.transform(pose_rel)
        self.target_pose = pose
        self.itf.go_to_pose_goal(pose)

    def move_tcp_absolute(self, pose):
        """
        move eff to absolute pose in robot base frame
        :param pose: list [x y z R P Y] (meter, radian)
        :return: None
        """
        # self.target_pose = copy.deepcopy(pose)
        self.itf.go_to_pose_goal(pose)

    def transform(self, pose_rel):
        """
        transform from [x y z R P Y] relative to tool
        to [x y z R P Y] to absolute robot base frame coord
        """
        pass

    def set_gripper(self, val):
        """
        gripper position control
        :param val: boolean (False:release, True:grip)
        :return: None
        """

        if val:
            self.gripper.open_close_gripper(1)
        else:
            self.gripper.open_close_gripper(0)

        # pass

    def get_gripper(self):
        """
        get gripper position
        :return: boolean (False:moving, True:stopped)
        """
        # return self.gripper.eval_gripper_motion()

    def rot_tool(self, val):
        """
        rotate wrist_3 joint
        :param val: float (0 to 1)
        :return: None
        """
        self.target_tool = val
        joints = self.itf.get_current_joint()
        joints[6] = (TOOL_RANGE[1] - TOOL_RANGE[0]) * val + TOOL_RANGE[0]
        self.itf.go_to_joint_state(joints, acc=0.4, vel=0.6)

    def get_tool_rot(self):
        """
        get wrist_3 joint value
        :return: float (0 to 1)
        """
        val = self.itf.get_current_joint()[6]
        return (min(max(val, TOOL_RANGE[0]), TOOL_RANGE[1]) - TOOL_RANGE[0])\
             / (TOOL_RANGE[1] - TOOL_RANGE[0])

    def wait_push_input(self, force):
        """
        initiate waiting for push input
        require calling get_push_input to reset before reuse
        :param force: minimum amount of force required (newton)
        :return: None
        """
        pass

    def get_push_input(self):
        """
        get if a push input has been registered, reset flag if yes
        :return: boolean (True:Yes, False:No)
        """
        pass

    def push(self, force, max_dist):
        """
        initiate applying force and/or torque in 4 dimensions
        require calling get_push to reset before reuse
        :param force: list (x y z R) (newton, newton meter)
        :param max_dist: list (x y z R) maximum travel in each dimensions (meter, radian)
        :return: None
        """
        pass

    def screw(self, cw):
        """
        initiate applying downward force and torque for a duration
        :param cw: boolean (False:clockwise, True:counter-clockwise)
        :return: None
        """
        self.control_mode.setJointImpedanceMode([2000, 2000, 2000, 1500, 100, 100, 100], \
                                                [0.5, 0.5, 0.5, 0.5, 0.2, 0.2, 0.2])


    def get_push(self):
        """
        get if push has completed
        :return: int (0:No, 1:max distance reached, 2:force acquired)
        """
        pass

    def disconnect(self):
        """
        terminate connection with robot
        :return: None
        """
        self.itf.disconnect()

    def test_joint(self):
        self.control_mode.setJointImpedanceMode([1000, 1000, 1000, 500, 50, 50, 50], \
                                                [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        raw_input("============ Press Enter to rotate joint in impedence mode......")
        self.move_joint(actions.d2r(TEST_joint[1]))
        rospy.loginfo("============ Rotate joint done!")

    def test_pick_place(self):
         # routine_pickplace()
        rospy.loginfo("============ Current position is:")
        current = actions.get_tcp_pose()
        rospy.loginfo(current)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        rospy.sleep(1)
        rospy.loginfo("============ Press `Enter` to execute a pick")
        raw_input()
        actions.move_tcp_absolute(pose_goal)
        rospy.loginfo("============ pick done ==============")

        # Close gripper
        # self.set_gripper(True)
        self.gripper.open_close_gripper(1)

        rospy.loginfo("============ Press `Enter` to go back")
        pose_goal.position.z = 0.8
        raw_input()
        rospy.sleep(1)
        actions.move_tcp_absolute(pose_goal)

        # open gripper
        # actions.set_gripper(False)
        rospy.loginfo("============ Done! ==================")

    def mode_test(self):
        rospy.loginfo("============ Current position is:")
        current = self.get_tcp_pose()
        rospy.loginfo(current)
        pose_goal = current

        pose_goal.pose.position.z -= 0.3
        rospy.loginfo("============ Pose target is:")
        rospy.loginfo(pose_goal)
        rospy.loginfo("============ Press `Enter` to execute a move-down")
        raw_input()
        self.move_tcp_absolute(pose_goal)


if __name__ == '__main__':
    actions = ActionHelper()

    try:
        actions.test_joint()
        # actions.test_pick_place()




        rospy.loginfo("============= All Tests Done! =============")

    except rospy.ROSInterruptException:
        pass

    except KeyboardInterrupt:
        pass

    # finally:
        # actions.disconnect()
