#!/usr/bin/env python

import sys
import os

# import roslib
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


ACCE = 0.5
VELO = 0.3

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class iiwa14():
    def __init__(self):
        #super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('lx_iiwa14', anonymous=False)
        rate = rospy.Rate(10)

        robot = moveit_commander.RobotCommander()
        print("============ Set up RobotCommander done!")
        group_names = robot.get_group_names()
        print("============ Available Planning Groups: ", group_names)
        print("============ Printing robot state: ")
        print(robot.get_current_state())

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.allow_replanning(True)

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # scene = moveit_commander.PlanningSceneInterface()

        # class variables
        self.robot = robot
        # self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.rate = rate

        self.move_group.set_planner_id("RRTConnectkConfigDefault")
        print("============ Initilization done! ")

    def get_current_pose(self):
        return self.move_group.get_current_pose()

    def get_current_joint(self):
        return self.move_group.get_current_joint_values()
    
    def go_to_joint_state(self, joint_list, acc=ACCE, vel=VELO, execute=True):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = joint_list[0]
        joint_goal[1] = joint_list[1]
        joint_goal[2] = joint_list[2]
        joint_goal[3] = joint_list[3]
        joint_goal[4] = joint_list[4]
        joint_goal[5] = joint_list[5]
        joint_goal[6] = joint_list[6]

        self.move_group.set_start_state_to_current_state()
        self.move_group.set_goal_joint_tolerance(0.001)
        self.move_group.set_max_acceleration_scaling_factor(acc)
        self.move_group.set_max_velocity_scaling_factor(vel)

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        self.rate.sleep()
        

    def go_to_pose_goal(self, pose_list, acc=ACCE, vel=VELO):
        # pose_goal = copy.deepcopy(pose_list)
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_max_acceleration_scaling_factor(acc)
        self.move_group.set_max_velocity_scaling_factor(vel)

        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(pose_list)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets() #no such for joint
        self.rate.sleep()

    
    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
    
    def execute_plan(self, plan):
        ## the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

    def disconnect(self):    
        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        # Exit MoveIt
        moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        iiwa14()
    except rospy.ROSInterruptException:
        pass
