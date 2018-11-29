#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from moveit_msgs.msg import Grasp, CollisionObject
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def openGripper():

  #// BEGIN_SUB_TUTORIAL open_gripper
  #/* Add both finger joints of panda robot. */
  JointTrajectory.joint_names.resize(2);
  JointTrajectory.joint_names[0] = "panda_finger_joint1";
  JointTrajectory.joint_names[1] = "panda_finger_joint2";

  #/* Set them as open, wide enough for the object to fit. */
  JointTrajectory.points.resize(1);
  JointTrajectory.points[0].positions.resize(2);
  JointTrajectory.points[0].positions[0] = 0.04;
  JointTrajectory.points[0].positions[1] = 0.04;
  #JointTrajectory.points[0].time_from_start = ros::Duration(0.5);
  #// END_SUB_TUTORIAL


def closedGripper():
  #// BEGIN_SUB_TUTORIAL closed_gripper
  #/* Add both finger joints of panda robot. */
  JointTrajectory.joint_names.resize(2);
  JointTrajectory.joint_names[0] = "panda_finger_joint1";
  JointTrajectory.joint_names[1] = "panda_finger_joint2";

  #/* Set them as closed. */
  JointTrajectory.points.resize(1);
  JointTrajectory.points[0].positions.resize(2);
  JointTrajectory.points[0].positions[0] = 0.00;
  JointTrajectory.points[0].positions[1] = 0.00;
  #JointTrajectory.points[0].time_from_start = ros::Duration(0.5);
  #// END_SUB_TUTORIAL

def addCollisionObjects(planning_scene_interface):
    CollisionObject[0].id = "table"
    CollisionObject[0].header.frame_id = "base_link"
    CollisionObject[0].primitives[0].dimensions[0] = 0.2
    CollisionObject[0].primitives[0].dimensions[1] = 0.4
    CollisionObject[0].primitives[0].dimensions[2] = 0.4
    #Define the pose of the table. 
    CollisionObject[0].primitive_poses[0].position.x = 0.5
    CollisionObject[0].primitive_poses[0].position.y = 0
    CollisionObject[0].primitive_poses[0].position.z = 0.2

    Grasp.grasp_pose.header.frame_id = "base_link"
    orientation = tf.Quaternion()
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2)
    Grasp.grasp_pose.pose.orientation = tf.toMsg(orientation)
    Grasp.grasp_pose.pose.position.x = 0.415
    Grasp.grasp_pose.pose.position.y = 0
    Grasp.grasp_pose.pose.position.z = 0.5

    CollisionObject[0].operation = CollisionObject[0].ADD

    #add object
    CollisionObject[1].id = "object"
    CollisionObject[1].header.frame_id = "base_link"
    CollisionObject[1].primitives[0].type = CollisionObject[1].primitives[0].BOX;
    CollisionObject[1].primitives[0].dimensions[0] = 0.02
    CollisionObject[1].primitives[0].dimensions[1] = 0.02
    CollisionObject[1].primitives[0].dimensions[2] = 0.2

    Grasp.grasp_pose.header.frame_id = "base_link"
    #tf2::Quaternion orientation
    #orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2)
    Grasp.grasp_pose.pose.orientation = tf.toMsg(orientation)
    Grasp.grasp_pose.pose.position.x = 0.5
    Grasp.grasp_pose.pose.position.y = 0
    Grasp.grasp_pose.pose.position.z = 0.5

    CollisionObject[0].operation = CollisionObject[0].ADD

    planning_scene_interface.applyCollisionObjects(CollisionObject);
    

def pick(move_group):
    
    #setting pregrasp approach
    #Defined with respect to frame_id */
    Grasp.pre_grasp_approach.direction.header.frame_id = "base_link";
    #/* Direction is set as positive x axis */
    Grasp.pre_grasp_approach.direction.vector.x = 1.0;
    Grasp.pre_grasp_approach.min_distance = 0.095;
    Grasp.pre_grasp_approach.desired_distance = 0.115;

    #/* Defined with respect to frame_id */
    Grasp.post_grasp_retreat.direction.header.frame_id = "base_link";
    #/* Direction is set as positive z axis */
    Grasp.post_grasp_retreat.direction.vector.z = 1.0;
    Grasp.post_grasp_retreat.min_distance = 0.1;
    Grasp.post_grasp_retreat.desired_distance = 0.25;

    move_group.setSupportSurfaceName("table")
    move_group.pick("object", Grasp);
#moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
#moveit::planning_interface::MoveGroupInterface group("panda_arm");

if __name__ == '__main__':
    rospy.init_node("demo")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso","base_link")

    # Define ground plane
    # This creates objects in the     planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface()
    
    while not rospy.is_shutdown():
        addCollisionObjects(planning_scene)
        pick(move_group)