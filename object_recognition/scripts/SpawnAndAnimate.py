from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import random

rospy.init_node('insert_object',log_level=rospy.INFO)
print(rospy.INFO)

poseSpawn=Pose()
tincan2y= -0.200000000000
tincan3y= -0.400000000000

#creating pose
poseSpawn=Pose()

#===============positionTincan3===============================
#the other position will ve calculated starting from this point

poseSpawn.position.x=1.000000000000
poseSpawn.position.y=0.000000000000
poseSpawn.position.z=0.051262000000

#===============orientation=============
poseSpawn.orientation.x=0.000000000000
poseSpawn.orientation.y=0.000000000000
poseSpawn.orientation.z=0.000000000000

name="tincan"


#==============SPAWN THE FIRST 3 DRINK=============
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name=name+'1',
    model_xml=open('/home/dan/.gazebo/models/sprite/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)

poseSpawn.position.y = -0.2000000000
spawn_model_client(model_name=name+'2',
    model_xml=open('/home/dan/.gazebo/models/pepsi/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)

poseSpawn.position.y = -0.4000000000
spawn_model_client(model_name=name+'3',
    model_xml=open('/home/dan/.gazebo/models/coke/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)

#===============wait for user input ======================
go_next=input("write Spawn if u want to spawn another drink")
i=0
while go_next == "Spawn":
	
	print(name+str((1+i)))
	#============== tincan 1 initial state ==============
	state_msg = ModelState()
	state_msg.model_name = name+str((1+i))
	state_msg.pose.position.x = poseSpawn.position.x
	state_msg.pose.position.y = poseSpawn.position.y
	state_msg.pose.position.z = poseSpawn.position.z
	state_msg.pose.orientation.x = 0.000000000000
	state_msg.pose.orientation.y = 0.000000000000
	state_msg.pose.orientation.z = 0.000000000000
	state_msg.pose.orientation.w = 0.000000000000

	print(name+str((2+i)))
	#============== tincan 2 initial state ==============
	state_msg2=ModelState()
	state_msg2.model_name = name +str((2+i))
	state_msg2.pose.position.x = poseSpawn.position.x
	state_msg2.pose.position.y = tincan2y
	state_msg2.pose.position.z = poseSpawn.position.z
	state_msg2.pose.orientation.x = 0.000000000000
	state_msg2.pose.orientation.y = 0.000000000000
	state_msg2.pose.orientation.z = 0.000000000000
	state_msg2.pose.orientation.w = 0.000000000000


	print(name+str((3+i)))
	#============== tincan 3 initial state ==============
	state_msg3=ModelState()
	state_msg3.model_name = name +str((3+i))
	state_msg3.pose.position.x = poseSpawn.position.x
	state_msg3.pose.position.y = tincan3y
	state_msg3.pose.position.z = poseSpawn.position.z
	state_msg3.pose.orientation.x = 0.000000000000
	state_msg3.pose.orientation.y = 0.000000000000
	state_msg3.pose.orientation.z = 0.000000000000
	state_msg3.pose.orientation.w = 0.000000000000


	#=========wait the movement of robot====================
	#here we simulate it
	
	print("moving the lattina",state_msg.model_name)
	print(state_msg.pose.position.x)
	while state_msg.pose.position.x < 1.5 :
		print(state_msg.pose.position.x)
		state_msg.pose.position.y=0.00000000000	
		state_msg.pose.position.x=state_msg.pose.position.x + 0.00050000000
		

		rospy.wait_for_service('/gazebo/set_model_state')
		set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		resp = set_state( state_msg )
		
	
	
	#====== move the 2 tincan =========
	print("moving the lattina",state_msg3.model_name)
	print("moving the lattina",state_msg2.model_name)
	
	while state_msg2.pose.position.y < 0.000000000000:
		state_msg2.pose.position.y=state_msg2.pose.position.y + 0.00050000000
		state_msg3.pose.position.y=state_msg3.pose.position.y + 0.00050000000
			
		
		rospy.wait_for_service('/gazebo/set_model_state')
		set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		resp = set_state( state_msg2 )
		resp = set_state( state_msg3)
	
	print("moved the other 2")
	i= i + 1
	
	tincantype=["coke","sprite","pepsi"]
	
	
	poseSpawn.position.y = -0.4000000000
	spawn_model_client(model_name=''+name+str((3+i)),
   	model_xml=open('/home/dan/.gazebo/models/'+tincantype[random.randint(0,2)]+'/model.sdf', 'r').read(),
            robot_namespace='/foo',
            initial_pose=poseSpawn,
            reference_frame='world'
        )
	go_next=input("write Spawn if u want to spawn another drink")
