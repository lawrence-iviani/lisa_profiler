import rospy
from transitions.core import MachineError
import pandas as pd
import numpy as np


#################
### Constants ###
#################
DEFAULT_PAUSE_BEFORE = 0.0 # silence before in sec.
DEFAULT_PAUSE_AFTER = 0.75 # silence after in sec.
TAG_MSG_WAKEUP = "wakeup"
TAG_MSG_INTENTS = "intents"

# TPC, Topics. Used for publish topic in player and detect in rosbags
TPC_WAKEUP_START = 'start_wakeup'
TPC_WAKEUP_STOP = 'stop_wakeup'
TPC_WAKEUP_IDENTIFIED = 'lisa/waked_up'

TPC_INTENT_START = 'start_intent'
TPC_INTENT_STOP = 'stop_intent'
TPC_INTENT_IDENTIFIED = 'lisa/intent'
TPC_INTENT_NOT_IDENTIFIED = 'lisa/not_recognized'

TPC_TEXT_CAPTURED = 'lisa/text_captured'

# Topics that are processed
DEFAULT_TOPICS = [TPC_WAKEUP_START, TPC_WAKEUP_STOP, TPC_WAKEUP_IDENTIFIED,
				  TPC_INTENT_START, TPC_INTENT_STOP, TPC_INTENT_IDENTIFIED, TPC_INTENT_NOT_IDENTIFIED,
				  TPC_TEXT_CAPTURED]

INVALID_VALUE = np.nan

##########################
### Init local library ###
##########################
from .audio import sd
from .audio import read_file_and_play
from ._helper import _print_debug
from .wakeup_test_processing import wake_up_test_model

# The outcome analised results will be provided classifeid in this form
# Check in wake_up_test_model for other values
RESULT_TAGS = [wake_up_test_model.WAKEUP_DETECTED,
			   wake_up_test_model.WAKEUP_NOT_DETECTED,
			   wake_up_test_model.TEXT_TRANSCRIPTED,
			   wake_up_test_model.INTENT_RECOGNIZED,
			   wake_up_test_model.INTENT_WRONG_RECOGNIZED,
		 	   wake_up_test_model.INTENT_NOT_RECOGNIZED]

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
			 rosmsg_before.publish("Start " + msg_tag + "\t" + filename)
		read_file_and_play(filename, audio_params)
		if rosmsg_after is not None:
			 rosmsg_after.publish("Stop " + msg_tag + "\t" + filename)

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


def analyse_df(df, result_tags_list=RESULT_TAGS):


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
			wake_up_test_model.reset(data=sm_data)
			transitions_has_happened = False
		# just check!
		if enter_state!=wake_up_test_model.state:
			print("\tTransition from {} ---> to {}".format(enter_state, wake_up_test_model.state))
		elif transitions_has_happened:
			print("\tInternal Transition from {} ---> to {}".format(enter_state, wake_up_test_model.state))
		else:
			print('\tError or no transition available, remainaning in state ' + enter_state)

		print('=======================')

	#dict_tagged_results = {key: list() for key in result_tags_list}
	dict_tagged_results = {key: dict() for key in result_tags_list}
	for transition_id, result in wake_up_test_model.get_all_results().items():
		#{'transition_6': {'outcome': 'TEXT ACQUIRED', 'wakeup_time': 0.1695864200592041, 'stt_time': 3.7553460597991943, 'intent_recognition_time': 'NaN'},
		if result['outcome'] in result_tags_list:
			k = result['outcome']
			# v = {transition_id: result}
			# print("++++\nAdding transition: {} --- {} \n++++".format(k, v))
			# dict_tagged_results[k].append(v)    # [transition_id] = result
			print("++++\nAdding transition: {} transition {} --- {} \n++++".format(k, transition_id, result))
			dict_tagged_results[k][transition_id] = result    # [transition_id] = result

	# transform dict in pandas df
	pd_tagged_results = {}
	for outcome, transitions in  dict_tagged_results.items():
		#print(transitions)
		data_frame = pd.DataFrame(transitions)
		pd_tagged_results[outcome] = data_frame.dropna(axis=0, how='all')
		#pd_tagged_results[outcome]#axis=0, how='any') # drop a row if all are NaN
		# print(pd.DataFrame(transitions))
	return pd_tagged_results



def run_process_bag(df, topics=DEFAULT_TOPICS): # pandas dataframe
	df_notnull = df.notnull()
	datas = {'timestamp': [], 'topic': [], 'payload': [] }
	for n, r in enumerate(df.iterrows()):
		not_null_idxs = list(np.where(df_notnull.iloc[n]==True)[0]) # Get only the valid value in row n
		ts = r[0]
		pd_serie = r[1].iloc[not_null_idxs] #__class__
		topic = _check_for_topic(pd_serie, topics)
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
