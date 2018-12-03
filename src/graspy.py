#!/usr/bin/env python

import sys
import rospy
import copy, math
#import moveit_commander

# from moveit_python import RobotCommander, MoveGroupCommander
import moveit_commander
# from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random

ROBOT_NAME = "fetch"

if ROBOT_NAME == "fetch":
    GROUP_NAME_ARM = 'arm_with_torso'
    GROUP_NAME_GRIPPER = 'gripper'

    GRIPPER_FRAME = 'r_gripper_finger_link'

    FIXED_FRAME = 'base_link'

    GRIPPER_CLOSED = 0.3
    GRIPPER_OPEN = 0.0

    GRIPPER_JOINT_NAMES = ['r_gripper_finger_joint']
    
    GRIPPER_EFFORT = [1.0]

class TestPick():
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_py_demo', anonymous=True)
       
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
        
        right_arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        right_gripper = moveit_commander.MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        eef = right_arm.get_end_effector_link()
        
        rospy.sleep(2)
        
        scene.remove_attached_object(GRIPPER_FRAME, "part")

    
        # clean the scene
        scene.remove_world_object("table")
        scene.remove_world_object("part")
    
        #right_arm.set_named_target("resting")
        #right_arm.go()
       
        #right_gripper.set_named_target("open")
        #right_gripper.go()
       
        rospy.sleep(1)
    
        # publish a demo scene
        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
    
        # add a table
        #p.pose.position.x = 0.42
        #p.pose.position.y = -0.2
        #p.pose.position.z = 0.3
        #scene.add_box("table", p, (0.5, 1.5, 0.6))

    
        # add an object to be grasped
        p.pose.position.x = 0.15

        p.pose.position.y = -0.12
        p.pose.position.z = 0.7
        scene.add_box("part", p, (0.07, 0.01, 0.2))
       
        rospy.sleep(1)
              
        g = Grasp()
        g.id = "test"
        start_pose = PoseStamped()
        start_pose.header.frame_id = FIXED_FRAME
    
        # start the gripper in a neutral pose part way to the target
        start_pose.pose.position.x = 0.0130178
        start_pose.pose.position.y = -0.125155
        start_pose.pose.position.z = 0.597653
        start_pose.pose.orientation.x = 0.0
        start_pose.pose.orientation.y = 0.388109
        start_pose.pose.orientation.z = 0.0
        start_pose.pose.orientation.w = 0.921613
           
        right_arm.set_pose_target(start_pose)
        right_arm.go()
        
        rospy.sleep(2)

        # generate a list of grasps
        grasps = self.make_grasps(start_pose)
    
        result = False
        n_attempts = 0
        
        # repeat until will succeed
        while result == False:
            result = right_arm.pick("part", grasps)       
            n_attempts += 1
            print "Attempts: ", n_attempts
            rospy.sleep(0.2)
           
        rospy.spin()
        moveit_commander.roscpp_shutdown()
        
        
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, pose):
        t = JointTrajectory()
        t.joint_names = GRIPPER_JOINT_NAMES
        tp = JointTrajectoryPoint()
        tp.positions = [pose for j in t.joint_names]
        tp.effort = GRIPPER_EFFORT
        t.points.append(tp)
        return t
    
    def make_gripper_translation(self, min_dist, desired, axis=1.0):
        g = GripperTranslation()
        g.direction.vector.x = axis
        g.direction.header.frame_id = GRIPPER_FRAME
        g.min_distance = min_dist
        g.desired_distance = desired
        return g

    def make_grasps(self, pose_stamped, mega_angle=False):
        # setup defaults for the grasp
        g = Grasp()
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1)
        g.post_grasp_retreat = self.make_gripper_translation(0.05, 0.1, -1.0)
        g.grasp_pose = pose_stamped
    
        pitch_vals = [0, 0.2, -0.2, 0.4, -0.4]
        #pitch_vals = [0]
        
        yaw_vals = [-0.2, -0.1, 0, 0.1, 0.2]
        #yaw_vals = [0]
        
        if mega_angle:
            pitch_vals += [0.78, -0.78, 0.3, -0.3, 0.5, -0.5, 0.6, -0.6]
    
        # generate list of grasps
        grasps = []
        #for y in [-1.57, -0.78, 0, 0.78, 1.57]:
        for y in yaw_vals:
            for p in pitch_vals:
                q = quaternion_from_euler(0, 1.57-p, y)
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                g.id = str(len(grasps))
                g.allowed_touch_objects = ["part"]
                g.max_contact_force = 0
                #g.grasp_quality = 1.0 - abs(p/2.0)
                grasps.append(copy.deepcopy(g))
        return grasps


if __name__=='__main__':
    TestPick()
