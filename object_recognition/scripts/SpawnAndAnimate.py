from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import random

rospy.init_node('insert_object',log_level=rospy.INFO)
print(rospy.INFO)

user = "benedettosimone"
#user = "dan"

#creating pose
poseSpawn=Pose()

#position of the second tin can
tincan2y= -0.200000000000

#position of the third tin can
tincan3y= -0.400000000000

name="tin_can"

#=============== Posision of the first tin can ===============================#
#the other position will be calculated starting from this point

poseSpawn.position.x=0.281779000000
poseSpawn.position.y=0.000596000000
poseSpawn.position.z=0.437266000000

poseSpawn.orientation.x=0.000000000000
poseSpawn.orientation.y=0.000000000000
poseSpawn.orientation.z=0.000000000000




#============== SPAWN THE FIRST 3 DRINK ==============#
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name=name+'1',
    model_xml=open('/home/'+user+'/.gazebo/models/sprite/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)

poseSpawn.position.y = tincan2y
spawn_model_client(model_name=name+'2',
    model_xml=open('/home/'+user+'/.gazebo/models/pepsi/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)

poseSpawn.position.y = tincan3y
spawn_model_client(model_name=name+'3',
    model_xml=open('/home/'+user+'/.gazebo/models/coke/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)

#=============== Moving forward the second and the third tin can and spawn another tin can======================
go_next=input("Write y to spawn another can: ")
i = 0
while go_next == "y":
	
	#============== Tin can 1 initial state ==============#
	state_msg = ModelState()
	state_msg.model_name = name+str((1+i))
	state_msg.pose.position.x = poseSpawn.position.x
	state_msg.pose.position.y = poseSpawn.position.y
	state_msg.pose.position.z = poseSpawn.position.z
	state_msg.pose.orientation.x = 0.000000000000
	state_msg.pose.orientation.y = 0.000000000000
	state_msg.pose.orientation.z = 0.000000000000
	state_msg.pose.orientation.w = 0.000000000000

	#============== Tin can 2 initial state ==============#
	state_msg2=ModelState()
	state_msg2.model_name = name +str((2+i))
	state_msg2.pose.position.x = poseSpawn.position.x
	state_msg2.pose.position.y = tincan2y
	state_msg2.pose.position.z = poseSpawn.position.z
	state_msg2.pose.orientation.x = 0.000000000000
	state_msg2.pose.orientation.y = 0.000000000000
	state_msg2.pose.orientation.z = 0.000000000000
	state_msg2.pose.orientation.w = 0.000000000000

	#============== Tin can 3 initial state ==============#
	state_msg3=ModelState()
	state_msg3.model_name = name +str((3+i))
	state_msg3.pose.position.x = poseSpawn.position.x
	state_msg3.pose.position.y = tincan3y
	state_msg3.pose.position.z = poseSpawn.position.z
	state_msg3.pose.orientation.x = 0.000000000000
	state_msg3.pose.orientation.y = 0.000000000000
	state_msg3.pose.orientation.z = 0.000000000000
	state_msg3.pose.orientation.w = 0.000000000000

	
	#====== Move the two tin can =========#
	print("Moving tin can",state_msg2.model_name)
	print("Moving tin can", state_msg3.model_name)
	
	while state_msg2.pose.position.y < 0.000000000000:
		state_msg2.pose.position.y=state_msg2.pose.position.y + 0.00050000000
		state_msg3.pose.position.y=state_msg3.pose.position.y + 0.00050000000
			
		rospy.wait_for_service('/gazebo/set_model_state')
		set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
		resp = set_state( state_msg2 )
		resp = set_state( state_msg3)
	
	print("Moved the other two tin can.")
	i = i + 1
	
	tincantype=["coke","sprite","pepsi"]

	poseSpawn.position.y = tincan3y
	spawn_model_client(model_name=''+name+str((3+i)),
   	model_xml=open('/home/'+user+'/.gazebo/models/'+tincantype[random.randint(0,2)]+'/model.sdf', 'r').read(),
            robot_namespace='/foo',
            initial_pose=poseSpawn,
            reference_frame='world'
        )
	
	go_next=input("Write y to spawn another can: ")
