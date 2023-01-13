import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


rospy.init_node('set_pose')

state_msg = ModelState()
state_msg.model_name = 'sprite12'
state_msg.pose.position.x = 1.00000000000
state_msg.pose.position.y = 0.000000000000
state_msg.pose.position.z = 0.051262000000
state_msg.pose.orientation.x = 0.000000000000
state_msg.pose.orientation.y = 0.000000000000
state_msg.pose.orientation.z = 0.000000000000
state_msg.pose.orientation.w = 0.000000000000


state_msg2=ModelState()
state_msg2.model_name = 'coke2'
state_msg2.pose.position.x = 1.00000000000
state_msg2.pose.position.y = 0.300000000000
state_msg2.pose.position.z = 0.051262000000
state_msg2.pose.orientation.x = 0.000000000000
state_msg2.pose.orientation.y = 0.000000000000
state_msg2.pose.orientation.z = 0.000000000000
state_msg2.pose.orientation.w = 0.000000000000

while state_msg.pose.position.x < 1.5 :
	state_msg.pose.position.x=state_msg.pose.position.x + 0.00050000000
	state_msg2.pose.position.x=state_msg2.pose.position.x + 0.00050000000

	rospy.wait_for_service('/gazebo/set_model_state')
	set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	resp = set_state( state_msg )
	resp = set_state(state_msg2)
	
