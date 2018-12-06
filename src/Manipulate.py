# Executes a grasp as published in topic /planned_grasp by fetchit_gqcnn node
# Prerequisits:
#   * Robot in front of a table occupying most of its field of view with camera looking down
#   * Arm extended (out of the camera view) and above the table surface
#   * fetch_gqcnn has run successfully
# Rough idea:
#   * Receive grasp pose 
#   * Determine approach direction by getting the new x direction from pose-quaternion
#   * Go to about 30 cm before the grasp along the negative x direction of the grasp pose while avoiding the table and object
#   * Move to the grasp psoition (keep in mind that the controllable pose is the endeffector one, not the tool (offset about 16 cm)
#   * Close gripper and retract the way one came from

import rospy
import actionlib

rospy.init_node("maipulation_execution")
print "Node inititilized"

# Receive grasp from planner

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
planned_pose = rospy.wait_for_message("/planned_grasp", PoseStamped, timeout=None)
print "Receieved grasp pose:\n" + planned_pose.__str__()

# Plan and Move to a goal pose
# This is the wrist link not the gripper itself
# The middle between the gripper fingers is 0.16645 m along the x axis
gripper_frame = 'wrist_roll_link'
# Extract the grasp postion and orientation
goal_point = planned_pose.pose.position
final_point = Point()
final_point.x = goal_point.x
final_point.y = goal_point.y
final_point.z = goal_point.z
goal_quart = planned_pose.pose.orientation
print "Original goal from planner: \n" + goal_point.__str__()

# want to move in negative x direction as encoded by the quarternion
offset = 0.19  #0.16645   # Offset between gripper and controled link
distance_approach = 0.3 + offset  # distance from which to start the approach
import tf
matrix = tf.transformations.quaternion_matrix([goal_quart.x,goal_quart.y,goal_quart.z,goal_quart.w])
# print matrix
goal_point.x -= distance_approach*matrix[0][0]
goal_point.y -= distance_approach*matrix[1][0]
goal_point.z -= distance_approach*matrix[2][0]
print "Start of grasp execution: \n" + goal_point.__str__()
final_point.x -= offset*matrix[0][0]
final_point.y -= offset*matrix[1][0]
final_point.z -= offset*matrix[2][0]
print "Goal for grasp: \n" + final_point.__str__()

# Interpolate between the two for execution; Implicit: Hope for little pose change if only moving a bit
approach_poses = []
for i in range(10):
    point = Point()
    point.x = goal_point.x * (9-i)/9 + final_point.x *i/9
    point.y = goal_point.y * (9-i)/9 + final_point.y *i/9
    point.z = goal_point.z * (9-i)/9 + final_point.z *i/9
    approach_poses.append(Pose(point,goal_quart))
    
# Construct a "pose_stamped" message as required by moveToPose
gripper_pose_stamped = PoseStamped()
gripper_pose_stamped.header.frame_id = 'base_link'

# Creat planning scene and move group to interact with robot
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

# rospy.init_node("move_torso_up")

# Create move group interface for a fetch robot
move_group = MoveGroupInterface("arm_with_torso", "base_link")

# Define ground plane
# This creates objects in the planning scene that mimic the ground
# If these were not in place gripper could hit the ground
planning_scene = PlanningSceneInterface("base_link")
planning_scene.removeCollisionObject("my_front_ground")
planning_scene.removeCollisionObject("my_back_ground")
planning_scene.removeCollisionObject("my_right_ground")
planning_scene.removeCollisionObject("my_left_ground")
planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
# Add table + object --> Hard coded, ! Have to lift arm beforehand, else stuck
planning_scene.removeCollisionObject("table")
planning_scene.addCube("table", 2, 1.1, 0.0, 0.82+0.1-1)

# Execute interpolated poses
for pose in approach_poses:
    # Finish building the Pose_stamped message
    # If the message stamp is not current it could be ignored
    gripper_pose_stamped.header.stamp = rospy.Time.now()
    # Set the message pose
    gripper_pose_stamped.pose = pose

    # Move gripper frame to the pose specified
    move_group.moveToPose(gripper_pose_stamped, gripper_frame)
    result = move_group.get_move_action().get_result()
    planning_scene.removeCollisionObject("table") # Only consider table for first move

    if result:
        # Checking the MoveItErrorCode
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Hello there!")
        else:
            # If you get to this point please search for:
            # moveit_msgs/MoveItErrorCodes.msg
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt! failure no result returned.")

# This stops all arm movement goals
# It should be called when a program is exiting so movement stops
move_group.get_move_action().cancel_all_goals()

# Close gripper
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
client.wait_for_server()
print("Gripper control server online and connected")
# Close
goal = GripperCommandGoal()
goal.command.position = 0  # Distance between fingers in m, max: 0.1
goal.command.max_effort = 40  # Force by fingers in N, max: 60
client.send_goal(goal)
client.wait_for_result(rospy.Duration.from_sec(5.0))

# Retract
# Execute interpolated poses
for pose in reversed(approach_poses):
    # Finish building the Pose_stamped message
    # If the message stamp is not current it could be ignored
    gripper_pose_stamped.header.stamp = rospy.Time.now()
    # Set the message pose
    gripper_pose_stamped.pose = pose

    # Move gripper frame to the pose specified
    move_group.moveToPose(gripper_pose_stamped, gripper_frame)
    result = move_group.get_move_action().get_result()
    planning_scene.removeCollisionObject("table") # Only consider table for first move

    if result:
        # Checking the MoveItErrorCode
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Hello there!")
        else:
            # If you get to this point please search for:
            # moveit_msgs/MoveItErrorCodes.msg
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt! failure no result returned.")

# This stops all arm movement goals
# It should be called when a program is exiting so movement stops
move_group.get_move_action().cancel_all_goals()

print "Object picked up"