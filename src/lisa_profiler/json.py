import json
import os.path

from . import DEFAULT_PAUSE_BEFORE, DEFAULT_PAUSE_AFTER

########### JSON ###########
def get_wakeup_token(wave_folder, args):
	'''
	Helper function for create a dict representing a wakeup token of the test
	'''
	ret_dict = {'filename':  '',
				'expected_wakeup_word': '',
				'pause_before': DEFAULT_PAUSE_BEFORE,
				'pause_after': DEFAULT_PAUSE_AFTER}
	for key, value in args.items():
		#print(key, value)
		if key == 'filename':
			value = os.path.join(wave_folder, value)
		ret_dict[key] = value
	return ret_dict


def get_intent_token(wave_folder, args):
	'''
	Helper function for create a dict representing an intent token of the test
	'''
	ret_dict = {'filename':  '',
				'expected_intents': [], # with an empty intent as example
				'pause_before': DEFAULT_PAUSE_BEFORE,
				'pause_after': DEFAULT_PAUSE_AFTER}
	for key, value in args.items():
		if key == 'filename':
			value = os.path.join(wave_folder, value)
		elif key == 'expected_intents':
# { 'intent_name': 'MotionParamSet',  'pay_load/1': ' target=velocity', 'pay_load/2': 'decimal_part=one', 'pay_load/3': 'fractional_part=five'}
			#if isinstance(value, str):
			#	value = [value]
			if isinstance(value, dict):
				value = [value]
			# cvalue should be a list of dict at this point
		ret_dict[key] = value
	return ret_dict


def get_test_token(wave_folder, args_wakeup, intent_folder, args_intent):
	'''
	Helper function for create a dict representing one token of the test
	'''
	return {'wakeup': get_wakeup_token(wave_folder, args_wakeup),
			'intent': get_intent_token(intent_folder, args_intent)}


def get_json_test_token(wave_folder, args_wakeup, args_intent):
	return json.dumps(get_test_token(wave_folder, args_wakeup, args_intent))


def save_to_json_file(filename, json_tests_list):

	with open(filename, 'w') as outfile:
		json.dump(json_tests_list, outfile, indent=2, sort_keys=True)


def load_from_json_file(filename):
	with open(filename, 'r') as infile:
		data = json.load(infile)
	# TODO: here some check should be performed
	return data
