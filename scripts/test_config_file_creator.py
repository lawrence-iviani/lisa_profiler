#!/usr/bin/env python3
'''
Use as example to create a json file for testing Lisa intent recognition.
Can be used by player and others
Run as normal script (not really in the ROS environment)
'''
import argparse
from collections import namedtuple
from os.path import join
from random import randrange

from lisa_profiler.json import get_test_token, save_to_json_file

WAVE_FOLDER_INTENTS = '/media/sf_Thesis/test track/intents/'
WAVE_FOLDER_WAKEUP = '/media/sf_Thesis/test track/wakeup_words/'

PAUSE_BEFORE_WAKEUP = 0.25
PAUSE_AFTER_WAKEUP = 1.5
PAUSE_BEFORE_INTENT = 0.25
PAUSE_AFTER_INTENT = 3.0


MALE = 'male'
FEMALE = 'female'
WAKEUP_WORD = 'snowboy'

testers_intent = namedtuple('testers_intent', ['intent_name', 'pay_load'])
testers_wakeup = namedtuple('testers_wakeup', ['name', 'id_wakeup'])


INTENT_MOTION_PARAM_SET = 'MotionParamSet'
INTENT_EXECUTE_TRAJECTORY = 'ExecuteTrajectory'
INTENT_EXECUTE_POINT_N = 'ExecutePointN'
INTENT_LOAD_TRAJ = 'LoadTrajectory'
INTENT_LOAD_INV_TRAJ = 'LoadInversedTrajectory'
INTENT_MOTION_PARAM_STEP = 'MotionParamStep'
INTENT_SELECT_NEXT_POINT = 'SelectNextPoint'


# the payload of synthetised voices
INTENT_MOTION_PARAM_SET_SYNTHETHISED_PL =  [(2, 3), (1, 5), (1,7), (1,9), (1,9),]
INTENT_EXECUTE_POINT_N_PL = [(8, ), (7, ), ]

## Utilities to create dict usable as intents
def _create_motion_param_set(payload_list):
	dict_val = {}
	for n, p in enumerate(payload_list):
		dict_val[n+1] = testers_intent('MotionParamSet', {'pay_load/1': 'target=velocity','pay_load/2': 'decimal_part='+str(p[0]),'pay_load/3': 'fractional_part='+str(p[1])} )
	return dict_val

def _create_intent_execute_point_n(payload_list):
	dict_val = {}
	for n, p in enumerate(payload_list):
		dict_val[n+1] = testers_intent('MotionParamSet', {'pay_load/1': 'action=start','pay_load/2': 'target='+str(p[0]),} )
	return dict_val

def _create_intent_run_traj(len_dict):
	dict_val = {}
	for n in range(len_dict):
		dict_val[n+1] = testers_intent('ExecuteTrajectory', {} )
	return dict_val

def _create_intent_execute_traj(len_dict):
	dict_val = {}
	for n in range(len_dict):
		dict_val[n+1] = testers_intent('ExecuteTrajectory', {} )
	return dict_val

def _create_intent_select_next_point(len_dict):
	dict_val = {}
	for n in range(len_dict):
		dict_val[n+1] = testers_intent('SelectNextPoint', {} )
	return dict_val

def _create_intent_motion_param_step(len_dict):
	dict_val = {}
	for n in range(len_dict):
		dict_val[n+1] = testers_intent('MotionParamStep', {} )
	return dict_val

def _create_intent_load_traj(len_dict):
	dict_val = {}
	for n in range(len_dict):
		dict_val[n+1] = testers_intent('LoadTrajectory', {} )
	return dict_val

def _create_intent_load_inv_traj(len_dict):
	dict_val = {}
	for n in range(len_dict):
		dict_val[n+1] = testers_intent('LoadInversedTrajectory', {} )
	return dict_val

def _create_intent_exec_traj(len_dict):
	dict_val = {}
	for n in range(len_dict):
		dict_val[n+1] = testers_intent('ExecuteTrajectory', {} )
	return dict_val


# the tester and what can be expected
testers_intents_synth_male_dict = {
	's_charles_uk':
		{
			'gender': MALE,
			'wakeup': testers_wakeup('m__' + WAKEUP_WORD, [1]),
			INTENT_MOTION_PARAM_SET: _create_motion_param_set(INTENT_MOTION_PARAM_SET_SYNTHETHISED_PL),
			INTENT_EXECUTE_TRAJECTORY: _create_intent_execute_traj(4),
			INTENT_EXECUTE_POINT_N: _create_intent_execute_point_n(INTENT_EXECUTE_POINT_N_PL),
			INTENT_LOAD_TRAJ: _create_intent_load_traj(2),
			INTENT_LOAD_INV_TRAJ: _create_intent_load_inv_traj(2),
			INTENT_MOTION_PARAM_STEP: _create_intent_motion_param_step(2),
			INTENT_SELECT_NEXT_POINT: _create_intent_select_next_point(3),
		},
	's_graham_uk':
		{
			'gender': MALE,
			'wakeup': testers_wakeup('m__' + WAKEUP_WORD, [1]),
			INTENT_MOTION_PARAM_SET: _create_motion_param_set(INTENT_MOTION_PARAM_SET_SYNTHETHISED_PL),
			INTENT_EXECUTE_TRAJECTORY: _create_intent_execute_traj(4),
			INTENT_EXECUTE_POINT_N: _create_intent_execute_point_n(INTENT_EXECUTE_POINT_N_PL),
			INTENT_LOAD_TRAJ: _create_intent_load_traj(2),
			INTENT_LOAD_INV_TRAJ: _create_intent_load_inv_traj(2),
			INTENT_MOTION_PARAM_STEP: _create_intent_motion_param_step(2),
			INTENT_SELECT_NEXT_POINT: _create_intent_select_next_point(3),
		},
	}
testers_intents_synth_female_dict = {
	's_lucy_uk':
		{
			'gender': FEMALE,
			'wakeup': testers_wakeup('f__' + WAKEUP_WORD, [1]),
			INTENT_MOTION_PARAM_SET: _create_motion_param_set(INTENT_MOTION_PARAM_SET_SYNTHETHISED_PL),
			INTENT_EXECUTE_TRAJECTORY: _create_intent_execute_traj(4),
			INTENT_EXECUTE_POINT_N: _create_intent_execute_point_n(INTENT_EXECUTE_POINT_N_PL),
			INTENT_LOAD_TRAJ: _create_intent_load_traj(2),
			INTENT_LOAD_INV_TRAJ: _create_intent_load_inv_traj(2),
			INTENT_MOTION_PARAM_STEP: _create_intent_motion_param_step(2),
			INTENT_SELECT_NEXT_POINT: _create_intent_select_next_point(3),
		},
	's_rachel_uk':
		{
			'gender': FEMALE,
			'wakeup': testers_wakeup('f__' + WAKEUP_WORD, [1]),
			INTENT_MOTION_PARAM_SET: _create_motion_param_set(INTENT_MOTION_PARAM_SET_SYNTHETHISED_PL),
			INTENT_EXECUTE_TRAJECTORY: _create_intent_execute_traj(4),
			INTENT_EXECUTE_POINT_N: _create_intent_execute_point_n(INTENT_EXECUTE_POINT_N_PL),
			INTENT_LOAD_TRAJ: _create_intent_load_traj(2),
			INTENT_LOAD_INV_TRAJ: _create_intent_load_inv_traj(2),
			INTENT_MOTION_PARAM_STEP: _create_intent_motion_param_step(2),
			INTENT_SELECT_NEXT_POINT: _create_intent_select_next_point(3),
		},
	}
testers_intents_human_male_dict = {
	'it_m':
		{
			'gender': MALE,
			'wakeup': testers_wakeup('' + WAKEUP_WORD, list(range(1,18+1))),
			INTENT_MOTION_PARAM_SET: _create_motion_param_set([(1,7), (1,9), (1,9)]),
			INTENT_EXECUTE_TRAJECTORY: _create_intent_execute_traj(3),
			INTENT_EXECUTE_POINT_N: _create_intent_execute_point_n([(21,), (9,), (5,)]),
			INTENT_LOAD_TRAJ: _create_intent_load_traj(3),
			INTENT_LOAD_INV_TRAJ: _create_intent_load_inv_traj(3),
			INTENT_MOTION_PARAM_STEP: _create_intent_motion_param_step(3),
			INTENT_SELECT_NEXT_POINT: _create_intent_select_next_point(3),
		},
	'it_m1':
		{
			'gender': MALE,
			'wakeup': testers_wakeup('' + WAKEUP_WORD, list(range(1,8+1))),
			INTENT_MOTION_PARAM_SET: _create_motion_param_set([(23,7), (24,15), (5,64)]),
			INTENT_EXECUTE_TRAJECTORY: _create_intent_execute_traj(7),
			INTENT_EXECUTE_POINT_N: _create_intent_execute_point_n([(12,), (15,), (23,), (12,), (12,), (25,)]),
			# INTENT_LOAD_TRAJ: _create_intent_load_traj(3),
			INTENT_LOAD_INV_TRAJ: _create_intent_load_inv_traj(4),
			INTENT_MOTION_PARAM_STEP: _create_intent_motion_param_step(4),
			INTENT_SELECT_NEXT_POINT: _create_intent_select_next_point(7),
		},
	'us_m':
		{
			'gender': MALE,
			'wakeup': testers_wakeup('' + WAKEUP_WORD, list(range(1,21+1))),
			INTENT_MOTION_PARAM_SET: _create_motion_param_set([(1,9), (6,5), (1,4)]),
			INTENT_EXECUTE_TRAJECTORY: _create_intent_execute_traj(4),
			INTENT_EXECUTE_POINT_N: _create_intent_execute_point_n([(5,), (7,), (7,), (1,)]),
			INTENT_LOAD_TRAJ: _create_intent_load_traj(3),
			INTENT_LOAD_INV_TRAJ: _create_intent_load_inv_traj(4),
			INTENT_MOTION_PARAM_STEP: _create_intent_motion_param_step(5),
			INTENT_SELECT_NEXT_POINT: _create_intent_select_next_point(5),
		},
	}
testers_intents_human_female_dict = {
	'at_f':
		{
			'gender': FEMALE,
			'wakeup': testers_wakeup('' + WAKEUP_WORD, list(range(1,16+1))),
			INTENT_MOTION_PARAM_SET: _create_motion_param_set([(3,5), (2,3), (1,5)]),
			INTENT_EXECUTE_TRAJECTORY: _create_intent_execute_traj(4),
			INTENT_EXECUTE_POINT_N: _create_intent_execute_point_n([(8,), (7,), (3,)]),
			INTENT_LOAD_TRAJ: _create_intent_load_traj(3),
			INTENT_LOAD_INV_TRAJ: _create_intent_load_inv_traj(3),
			INTENT_MOTION_PARAM_STEP: _create_intent_motion_param_step(3),
			INTENT_SELECT_NEXT_POINT: _create_intent_select_next_point(3),
		},
}

testers_intents_dict = {**testers_intents_human_male_dict,
						**testers_intents_human_female_dict,
						**testers_intents_synth_female_dict,
						**testers_intents_synth_male_dict}

INTENTS = [
			INTENT_MOTION_PARAM_SET,
		   	INTENT_EXECUTE_TRAJECTORY,
   			INTENT_EXECUTE_POINT_N,
   			INTENT_LOAD_TRAJ,
   			INTENT_LOAD_INV_TRAJ,
   			INTENT_MOTION_PARAM_STEP,
   			INTENT_SELECT_NEXT_POINT,
		   ]

def setup_test(wave_folder_intents = WAVE_FOLDER_INTENTS, wave_folder_wakeup = WAVE_FOLDER_WAKEUP):
	json_tests_list = []
	for i in INTENTS:
		for t_name, t in testers_intents_dict.items():
			if i in t.keys():
				intent = t[i]
			else:
				continue
			gender = t['gender']
			wakeup = t['wakeup']
			for k, v in intent.items():
				# pick up  a random activation signal
				id_wakeup = t['wakeup'].id_wakeup[randrange(len(t['wakeup'].id_wakeup))]
				wake_up_folder = wave_folder_wakeup
				wakeup_fn = t_name + '_' + wakeup.name + "_" + str(id_wakeup) + ".wav"
				args_wakeup={'expected_wakeup_word': WAKEUP_WORD,
							 'pause_before': PAUSE_BEFORE_WAKEUP,
							 'pause_after': PAUSE_AFTER_WAKEUP,
							 'filename': wakeup_fn}

				# pick up the intnent with the payload
				intent_fn =  t_name + '__' + v.intent_name + '_'+ str(k) + ".wav"
				intent_folder = join(wave_folder_intents, gender, v.intent_name)
				intent_dict = {**{'intent_name': v.intent_name}, **v.pay_load}
				args_intent={'filename': intent_fn,
							 'pause_before': PAUSE_BEFORE_INTENT,
							 'pause_after': PAUSE_AFTER_INTENT,
							 'expected_intents': [intent_dict],}
				#print('wake_up_folder',wake_up_folder)
				#print('args_wakeup',args_wakeup)
				#print('intent_folder',intent_folder)
				#print('args_intent',args_intent)
				json_tests_list.append(get_test_token(wake_up_folder, args_wakeup, intent_folder, args_intent))

	return json_tests_list

if __name__ == "__main__":

	parser = argparse.ArgumentParser()
	parser.add_argument(
		'filename', metavar='FILENAME',
		help='The output JSON file with the description of the audio test playback')
	args = parser.parse_args()
	json_tests_list = setup_test()
	save_to_json_file(args.filename, json_tests_list)
