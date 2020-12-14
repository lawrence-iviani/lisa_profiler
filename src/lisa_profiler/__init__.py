

#import sys
#from datetime import datetime
import rospy

from .audio import sd
from .audio import read_file_and_play
from ._helper import _print_debug
from transitions.core import MachineError

#################
### Constants ###
#################
DEFAULT_PAUSE_BEFORE = 0.0 # silence before in sec.
DEFAULT_PAUSE_AFTER = 0.75 # silence after in sec.
TAG_MSG_WAKEUP = "wakeup"
TAG_MSG_INTENTS = "intents"

# TPC Topics
TPC_WAKEUP_START = 'start_wakeup'
TPC_WAKEUP_STOP = 'stop_wakeup'
TPC_WAKEUP_IDENTIFIED = 'lisa/waked_up'

TPC_INTENT_START = 'start_intent'
TPC_INTENT_STOP = 'stop_intent'
TPC_INTENT_IDENTIFIED = 'lisa/intent'
TPC_INTENT_NOT_IDENTIFIED = 'lisa/not_recognized'

TPC_TEXT_CAPTURED = 'lisa/text_captured'

DEFAULT_TOPICS = [TPC_WAKEUP_START, TPC_WAKEUP_STOP, TPC_WAKEUP_IDENTIFIED,
				  TPC_INTENT_START, TPC_INTENT_STOP, TPC_INTENT_IDENTIFIED, TPC_INTENT_NOT_IDENTIFIED,
				  TPC_TEXT_CAPTURED]


#################
### Functions ###
#################

########### RUN TEST ###########
def run_file_test(json_tests_list, audio_params, ros_publishers_dict):

	def _run_test(filename, pause_before, pause_after, audio_params, rosmsg_before=None, rosmsg_after=None, msg_tag=''):

		# 1. Pause before
		_print_debug(function='run_file_test', msg="sleeping BEFORE for {} sec.".format(pause_before ))
		#rospy.sleep(pause_before)
		sd.sleep(int(pause_before*1000))
		# 2. Signal before, play file and signal after
		_print_debug(function='run_file_test', msg="Play file {}".format(filename))
		if rosmsg_before is not None:
			 rosmsg_before.publish("Start " + filename + "\t" + msg_tag)
		read_file_and_play(filename, audio_params)
		if rosmsg_after is not None:
			 rosmsg_after.publish("Stop "+ filename + "\t" + msg_tag)

		# 3. Pause after
		_print_debug(function='run_file_test', msg="sleeping AFTER for {} sec.".format(pause_after ))
		#rospy.sleep(pause_after)
		sd.sleep(int(pause_after*1000))

	for t in json_tests_list:
		_print_debug(function='run_file_test', msg="===== START TEST ITEM ====\n=============================\n")
		# wakeup
		_print_debug(function='run_file_test', msg="===== WAKEUP ====")
		_run_test(filename=t['wakeup']['filename'],
				  pause_before=t['wakeup']['pause_before'],
				  pause_after=t['wakeup']['pause_after'],
				  audio_params=audio_params,
				  rosmsg_before=ros_publishers_dict['pub_start_wakeup'],
				  rosmsg_after=ros_publishers_dict['pub_stop_wakeup'],
				  msg_tag=TAG_MSG_WAKEUP)

		# intent
		_print_debug(function='run_file_test', msg="===== INTENT ====")
		_run_test(filename=t['intent']['filename'],
				  pause_before=t['intent']['pause_before'],
				  pause_after=t['intent']['pause_after'],
				  audio_params=audio_params,
				  rosmsg_before=ros_publishers_dict['pub_start_intent'],
				  rosmsg_after=ros_publishers_dict['pub_stop_intent'],
				  msg_tag=TAG_MSG_INTENTS)

		_print_debug(function='run_file_test', msg="===== END TEST ITEM ====\n=============================\n")


def _check_for_topic(pd_serie, search_topics_list):
	"""
	Check the header of the serie (all topics) are the same and present in these
	list to be searched
	"""
	topic = None

	topics_and_data = [i[0] for i in pd_serie.items()]

	for t in search_topics_list:
		for n_d, t_d in enumerate(topics_and_data):
			if n_d == 0:
				if t_d.startswith(t):
					topic = t # a candidate has been found
				else:
					# if not available at the first index there is something wrong
					# skip it
					break
			else:
				# if n_d > 0 the topic must be congruent with the value found for n_d=0, sanity check
				assert t_d.startswith(topic), "Incongruent topic {} for {}".format(topic, t_d)
		if topic is not None:
			# I found the topic, can stop
			break
	return topic


def _get_data_from_topic(topic, pd_serie):
	ret_dict = {}
	for i in pd_serie.items():
		data_name = i[0][len(topic)+1:]
		ret_dict[data_name] = i[1]
		#print(data_name,":" ,ret_dict[data_name], end=" --- ")
	return ret_dict

def analyse_df(df):
	from .wakeup_test_processing import wake_up_test_model
	print(wake_up_test_model.state)
	for n, r in enumerate(df.iterrows()):
		sm_data = {}
		data = r[1] # get the row as tuple timestamp, topic, payload
		sm_data['transition'] = n
		sm_data['timestamp'] = float(data[0])
		sm_data['topic'] = data[1]
		sm_data['payload'] = data[2]
		enter_state = wake_up_test_model.state
		print("Transition[{}] {}  from state: {}".format(str(sm_data['transition']), sm_data['topic'] , enter_state))
		#print("received transition {} [{}]-{}> |{}|".format(
		#		sm_data['transition'], sm_data['timestamp'], sm_data['topic'], sm_data['payload']))
		transitions_has_happened = True
		try:
			if sm_data['topic']==TPC_WAKEUP_IDENTIFIED:
				wake_up_test_model.waked_up_received(data=sm_data)
			elif sm_data['topic']==TPC_WAKEUP_START:
				wake_up_test_model.wakeup_playback_started(data=sm_data)
			elif sm_data['topic']==TPC_WAKEUP_STOP:
				wake_up_test_model.wakeup_playback_stopped(data=sm_data)
			elif sm_data['topic']==TPC_TEXT_CAPTURED:
				wake_up_test_model.text_recieved(data=sm_data)
			elif sm_data['topic']==TPC_INTENT_START:
				wake_up_test_model.intent_playback_started(data=sm_data)
			elif sm_data['topic']==TPC_INTENT_STOP:
				wake_up_test_model.intent_playback_stopped(data=sm_data)
			elif sm_data['topic']==TPC_INTENT_IDENTIFIED:
				wake_up_test_model.intent_recognized_recieved(data=sm_data)
			elif sm_data['topic']==TPC_INTENT_NOT_IDENTIFIED:
				wake_up_test_model.intent_not_recognized_recieved(data=sm_data)
			else:
				transitions_has_happened = False
		except MachineError as e:
			print("Transition[{}] {} FAILED: {} ".format(str(sm_data['transition']), sm_data['topic'] , e))
			transitions_has_happened = False
		# just check!
		if enter_state!=wake_up_test_model.state:
			print("\tTransition from {} ---> to {}".format(enter_state, wake_up_test_model.state))
		elif transitions_has_happened:
			print("\tInternal Transition from {} ---> to {}".format(enter_state, wake_up_test_model.state))
		else:
			print('\tErro or no transition available, remainaning in state ' + enter_state)

		print('=======================')

import pandas as pd
import numpy as np
def run_process_bag(df): # pandas dataframe
	df_notnull = df.notnull()
	datas = {'timestamp': [], 'topic': [], 'payload': [] }
	for n, r in enumerate(df.iterrows()):
		#print("hasattr(lisa/text_captured/text): " + str(hasattr(r, "lisa/text_captured/text")))
		#for t in DEFAULT_TOPICS:
		#	 if df.
		not_null_idxs = list(np.where(df_notnull.iloc[n]==True)[0]) # Get only the valid value in row n
		ts = r[0]
		pd_serie = r[1].iloc[not_null_idxs] #__class__
		topic = _check_for_topic(pd_serie, DEFAULT_TOPICS)
		# _print_debug(function='run_process_bag', msg="Decoding topic {}".format(topic))
		if topic is None:
			_print_debug(function='run_process_bag', msg="Cannot find a valid topic for line {} - data: ".format(n, pd_serie))
			exit(-1)

		data = _get_data_from_topic(topic, pd_serie)
		# _print_debug(function='run_process_bag', msg="Data decoded {}".format(data))

		_print_debug(function='run_process_bag', msg="|{}|{}|{}|".format(ts, topic, data))
		datas['timestamp'].append(ts)
		datas['topic'].append(topic)
		datas['payload'].append(data) # str??


	return pd.DataFrame(datas)
