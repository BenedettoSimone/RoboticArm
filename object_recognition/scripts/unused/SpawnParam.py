from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
import sys
drink=sys.argv[1]
name=sys.argv[2]
## to call it specify param drink and name of model 
##3 type of drink coke,sprite or pepsi#


rospy.init_node('insert_object',log_level=rospy.INFO)
print(rospy.INFO)

##creating pose
poseSpawn=Pose()

#===============position================

poseSpawn.position.x=1.000000000000
poseSpawn.position.y=0.000000000000
poseSpawn.position.z=0.051262000000

#===============orientation=============
poseSpawn.orientation.x=0.000000000000
poseSpawn.orientation.y=0.000000000000
poseSpawn.orientation.z=0.000000000000

#=============calling service============
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name=name,
    model_xml=open('/home/dan/.gazebo/models/'+drink+'/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)


