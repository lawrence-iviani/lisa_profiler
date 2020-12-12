import rospy
import subprocess
import datetime
import os.path
import signal

from std_msgs.msg import Empty

ROS_QUEUE_SIZE = 5
DEFAULT_TOPICS = ['start_wakeup', 'stop_wakeup', 'start_intent', 'stop_intent']

########### ROS ###########
def ros_init():
	ret_dict = {}
	ret_dict['pub_start_wakeup'] = rospy.Publisher('start_wakeup', Empty, queue_size=ROS_QUEUE_SIZE)
	ret_dict['pub_stop_wakeup'] = rospy.Publisher('stop_wakeup', Empty, queue_size=ROS_QUEUE_SIZE)
	ret_dict['pub_start_intent'] = rospy.Publisher('start_intent', Empty, queue_size=ROS_QUEUE_SIZE)
	ret_dict['pub_stop_intent'] = rospy.Publisher('stop_intent', Empty, queue_size=ROS_QUEUE_SIZE)
	rospy.init_node('lisa_profiler', anonymous=True)
	return ret_dict


def ros_start_lisa_rosbag(folder="/home/pi", filename="", topics = DEFAULT_TOPICS):
	assert isinstance(topics, list)
	if filename is None or len(filename)==0:
		filename = os.path.join(folder, datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S_LISA_Rosbag.bag"))
#		filename = datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S_LISA_Rosbag.bag")

	cmd = ["rosbag", "record", "-O",  '__name:=lisa_bag', filename]
	print(cmd + topics)
	proc = subprocess.Popen(cmd + topics)

	return proc


def ros_stop_lisa_rosbag(rosbag_process):

	killcommand = ["rosnode", "kill", "lisa_bag"]
	print("Killing: ", str(rosbag_process.pid))
	subprocess.Popen(killcommand, )

	rosbag_process.wait(timeout=3)
