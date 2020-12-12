		

#import sys
#from datetime import datetime
from .ros import rospy
from .audio import read_file_and_play
from ._helper import _print_debug



#################
### Constants ###
#################
DEFAULT_PAUSE_BEFORE = 0.0 # silence before in sec.
DEFAULT_PAUSE_AFTER = 0.75 # silence after in sec. 


#################
### Functions ###
#################


########### RUN TEST ###########
def run_file_test(json_tests_list, audio_params, ros_publishers_dict):

	def _run_test(filename, pause_before, pause_after, audio_params, rosmsg_before=None, rosmsg_after=None):

		# 1. Pause before		
		_print_debug(function='run_file_test', msg="sleeping BEFORE for {} sec.".format(pause_before ))
		rospy.sleep(pause_before)

		# 2. Signal before, play file and signal after
		_print_debug(function='run_file_test', msg="Play file {}".format(filename))
		if rosmsg_before is not None:
			 rosmsg_before.publish()
		read_file_and_play(filename, audio_params)
		if rosmsg_after is not None:
			 rosmsg_after.publish()

		# 3. Pause after
		_print_debug(function='run_file_test', msg="sleeping AFTER for {} sec.".format(pause_after ))
		rospy.sleep(pause_before)
		
	for t in json_tests_list:
		_print_debug(function='run_file_test', msg="===== START TEST ITEM ====\n=============================\n")
		# wakeup
		_print_debug(function='run_file_test', msg="===== WAKEUP ====")
		_run_test(t['wakeup']['filename'], t['wakeup']['pause_before'], t['wakeup']['pause_after'], 
					audio_params, ros_publishers_dict['pub_start_wakeup'], ros_publishers_dict['pub_stop_wakeup'])
		
		# intent
		_print_debug(function='run_file_test', msg="\n===== INTENT ====")
		_run_test(t['intent']['filename'], t['intent']['pause_before'], t['intent']['pause_after'], 
					audio_params, ros_publishers_dict['pub_start_intent'], ros_publishers_dict['pub_stop_intent'])

		_print_debug(function='run_file_test', msg="===== END TEST ITEM ====\n=============================\n")

