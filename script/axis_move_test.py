#!/usr/bin/env python

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
## END_SUB_TUTORIAL

import numpy as np

from std_msgs.msg import String

def axis_move_test():

    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

    print "current joint values:  ", group.get_current_joint_values()
    print "current positino:  \n", group.get_current_pose().pose.position
    print "planning time: " , group.get_planning_time()
    print "planning frame: ", group.get_planning_frame()
    print " goal torleara" , group.get_goal_tolerance()
    group_variable_values = [-5.403968636189596, -1.7618020216571253, -1.6557276884662073, 3.3749208450317383, -0.8409798781024378, -3.101166311894552]
    # moveJoint(group, group_variable_values, 0.05)
    #
    # print "============ Going up"
    # moveRelativePt(group, [0.0, 0.0, 0.05], 0.05)
    # # moveRelativePt(group, [-0.08, -0.0, 0.0172100525103], 0.01)
    #
    # print "============ move` 1"
    # point_1 = [0, -0.15, -0.04]
    # moveRelativePt(group, point_1, 0.05)
    # rospy.sleep(3)
    # print "============ Going up"
    # moveRelativePt(group, [0.0, 0.0, 0.05], 0.05)
    #
    # print "============ move` 2"
    # point_1 = [0, 0.3, -0.04]
    # moveRelativePt(group, point_1, 0.05)
    # print "============ Going up"
    # moveRelativePt(group, [0.0, 0.0, 0.05], 0.05)

def moveJoint(group, group_variable_values, speed):
    group.set_start_state_to_current_state()
    group.set_max_velocity_scaling_factor(speed)
    group.clear_pose_targets()
    print "============ Joint values: ", group_variable_values
    group.set_joint_value_target(group_variable_values)
    plan1 = group.plan()
    group.go(wait=True)
    # group.execute(plan1)
    rospy.sleep(1)

def moveCartesianPath(group, pts, org_pose, speed):
    group.set_max_velocity_scaling_factor(speed)
    waypoints = []
    waypoints.append(group.get_current_pose().pose)

    # print "------------------- current pose -----", pose_target.position
    for pt in pts:
        pose_target = copy.deepcopy(org_pose)
        pose_target.position.x += pt[1]
        pose_target.position.y += pt[0]
        pose_target.position.z += pt[2]
        waypoints.append(pose_target)
    print "waypoints" , waypoints[:5]
    (plan, fraction) = group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.00001,        # eef_step
                                 0.0)         # jump_threshold
    # group.go(wait=True)
    group.execute(plan)
    rospy.sleep(1)

def moveRelRotPt(group, pt, org_pose, speed):
    # if(speed):
    group.set_max_velocity_scaling_factor(speed)
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    pose_target = copy.deepcopy(org_pose)
    print "--------rel move -----", pt
    # print "------------------- current pose -----", pose_target.position
    pose_target.position.x += pt[1]
    pose_target.position.y += pt[0]
    pose_target.position.z += pt[2]
    # print "------------------- after update pose -----", pose_target.position
    pose_target_stamped = group.get_current_pose()
    pose_target_stamped.pose = pose_target
    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(1)

def moveRelativePt(group, pt, speed):
    # if(speed):
    group.set_max_velocity_scaling_factor(speed)
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    pose_target = group.get_current_pose().pose
    print "--------rel move -----", pt
    # print "------------------- current pose -----", pose_target.position
    pose_target.position.x += pt[0]
    pose_target.position.y += pt[1]
    pose_target.position.z += pt[2]
    # print "------------------- after update pose -----", pose_target.position

    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(1)

def moveAbsPt(group, pt, speed):
    # if(speed):
    group.set_max_velocity_scaling_factor(speed)

    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    pose_target = group.get_current_pose().pose

    pose_target.position.x = pt[0]
    pose_target.position.y = pt[1]
    pose_target.position.z = pt[2]

    group.set_pose_target(pose_target)
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(1)

if __name__=='__main__':
  try:
    axis_move_test()
  except rospy.ROSInterruptException:
    pass
