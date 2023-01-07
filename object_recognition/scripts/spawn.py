from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
rospy.init_node('insert_object',log_level=rospy.INFO)
print(rospy.INFO)

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name='prova3',
    model_xml=open('/home/dan/catkin_ws/src/project_gazebo/worlds/sprite/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=Pose(),
    reference_frame='world'
)
