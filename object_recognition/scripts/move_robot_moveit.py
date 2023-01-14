import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time

subscriber = None

object_classification = ""


#====================MAIN PROGRAM===================#

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('seven_dof_arm_planner', anonymous=True)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


# Get information about the robot
print ("Robot information:")
print (robot.get_current_state())

# Get name of groups
group_names = robot.get_group_names()
print ("Robot Groups:", robot.get_group_names())

# Arm Group
arm_group = moveit_commander.MoveGroupCommander('arm')
arm_group.set_max_acceleration_scaling_factor (1)
arm_group.set_max_velocity_scaling_factor(1)

# Gripper Group
gripper_group = moveit_commander.MoveGroupCommander('gripper')

# Get end effectors of the arm
eef_link = arm_group.get_end_effector_link()
print ("End effector: %s" % eef_link)

# Topic to publish the trajectory
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)





# Function to change manually the values of the joints
def move_joint(j1,j2,j3,j4,j5,j6,j7):
    # Get the joint values of the arm group
    joint_goal = arm_group.get_current_joint_values()

    # Change the joint values
    joint_goal[0] = j1
    joint_goal[1] = j2
    joint_goal[2] = j3
    joint_goal[3] = j4
    joint_goal[4] = j5
    joint_goal[5] = j6
    joint_goal[6] = j7

    arm_group.go(joint_goal, wait=True)
    arm_group.stop()


# Function to reach a point in the space, without managing the joints
# Get error if it's impossible to do a motion planning
# example set_pose_goal(1.0, 0.4, 0.1, 0.4) 
def set_pose_goal(w, x, y, z):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = w
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm_group.set_pose_target(pose_goal)

    plan = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()


# Funtion to move only the gripper, using pre-built pose
def set_named_goal(pose_name):
    gripper_group.set_named_target(pose_name)

    plan = gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()

# function to get classification 
def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
    global data 
    object_classification = msg.data
    
    # Read only one message
    subscriber.unregister()


# Open the gripper
set_named_goal("open")

#print(arm_group.get_current_pose())

# Execute the grasping phase for three objects
while True:
    
    # Reach the start pose
    move_joint(0,-0.0178,0,1.0444,0,1.4121,0)
    
    # Move the arm above the object
    move_joint(0,0.2731,0,1.6209,0,1.1138,0)

    # Subscription to topic to read the object classification
    subscriber = rospy.Subscriber("object_classification", String, callback)

    # Close the gripper
    set_named_goal("close")
    
    print("========")
    print(object_classification)
    
    
    # Check the type and set the goal pose

    i = input("Write class: ")
    
    if i=="coke":
        # Coke goal
        move_joint(0.7041,1.0746,-0.0614,0.1374,0.0748,1.1156,0)
    
    elif i == "pepsi":
        # Pepsi goal     
        move_joint(1.2759,0.3593,-0.0417,1.0560,0.0748,1.1156,0)
    
    elif i == "sprite":
        # Sprite goal
        move_joint(1.9839,0.7445,0,0.6385,0.0748,1.1156,0)
  
    # Open the gripper
    set_named_goal("open")
    
    i = input("Go next, press something: ")
    
 

