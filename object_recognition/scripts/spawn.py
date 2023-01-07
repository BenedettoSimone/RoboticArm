from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
rospy.init_node('insert_object',log_level=rospy.INFO)
print(rospy.INFO)

poseSpawn=Pose()
poseSpawn.position.x=1
poseSpawn.position.y=1
poseSpawn.position.z=0

poseSpawn.orientation.x=0
poseSpawn.orientation.y=0
poseSpawn.orientation.z=0

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name='lattina1',
    model_xml=open('/home/dan/catkin_ws/src/project_gazebo/worlds/sprite/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=poseSpawn,
    reference_frame='world'
)
