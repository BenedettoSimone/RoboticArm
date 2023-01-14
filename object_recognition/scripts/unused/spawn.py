from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
rospy.init_node('insert_object',log_level=rospy.INFO)
print(rospy.INFO)

poseSpawn=Pose()

poseSpawn.position.x=0
poseSpawn.position.y=2.0
poseSpawn.position.z=0

poseSpawn.orientation.x=0
poseSpawn.orientation.y=0
poseSpawn.orientation.z=0

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name='sprite',
    model_xml=open('/home/dan/.gazebo/models/sprite/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)

poseSpawn.position.x = 3.0
spawn_model_client(model_name='pepsi',
    model_xml=open('/home/dan/.gazebo/models/pepsi/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)

poseSpawn.position.x = 4.0
spawn_model_client(model_name='coke',
    model_xml=open('/home/dan/.gazebo/models/coke/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)
