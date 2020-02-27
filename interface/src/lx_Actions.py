#!/usr/bin/env python

# from hwinterface import HWInterface
from lx_iiwa14 import iiwa14
import time
from threading import Thread

# import roslib
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

HOMEJ = [105, -65, -110, -24, 47, -61, 163]
TOOL_RANGE = [-3.1, 3.1]

TEST_pose = [[560/1000,0/1000,460/1000,0,0,-2.9], \
    [-131, -50, 107, -53, 156, 80, -136]]

class Actions():
    def __init__(self):
        
        # interface init
        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('SimpleMoveit', anonymous=False)

        self.itf = iiwa14()

        # robot init
        print('============ Homing robot...')
        # self.gripper...
        self.move_home()
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

    def move_joint(self, joint_list):
        """
        move back to home (Blocking)
        :return: None
        """
        self.itf.go_to_joint_state(joint_list)

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

    def move_tcp_absolute(self, pose_list):
        """
        move eff to absolute pose in robot base frame
        :param pose: list [x y z R P Y] (meter, radian)
        :return: None
        """
        self.target_pose = pose_list
        self.itf.go_to_pose_goal(pose_list)

    def transform(self, pose_rel):
        """
        transform from [x y z R P Y] relative to tool
        to [x y z R P Y] to absolute robot base frame coord
        """
        pass

    def set_gripper(self, val):
        """
        gripper position control
        :param val: boolean (False:released, True:gripped)
        :return: None
        """
        pass

    def get_gripper(self):
        """
        get gripper position
        :return: boolean (False:released, True:gripped)
        """
        pass

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
        pass

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


if __name__ == '__main__':
    actions = Actions()

    try:
        # routine_pickplace()
        print("============ Press `Enter` to execute a move_joint")
        raw_input()
        actions.move_joint(actions.d2r(TEST_pose[1]))
        print("============ move_joint done")

    finally:
        actions.disconnect()