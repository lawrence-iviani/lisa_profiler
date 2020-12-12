#!/usr/bin/env python3
'''
Use as example to create a json file for testing Lisa intent recognition.
Can be used by player and others
Run as normal script (not really in the ROS environment)
'''
import argparse
from lisa_profiler.json import get_test_token, save_to_json_file


if __name__ == "__main__": 

	parser = argparse.ArgumentParser()
	parser.add_argument(
		'filename', metavar='FILENAME',
		help='The output JSON file with the description of the audio test playback')
	args = parser.parse_args()
	
	json_tests_list = []
	json_tests_list.append(get_test_token(wave_folder='/media/sf_Thesis/test track', 
								   args_wakeup={'expected_wakeup_word': 'snowboy', 'filename': 'snowboy_test_1.wav'}, 
								   args_intent={'expected_intents': ['set_acceleration'], 'filename': 'intent_test_1.wav'}))
	
	json_tests_list.append(get_test_token(wave_folder='/media/sf_Thesis/test track', 
								   args_wakeup={'expected_wakeup_word': 'snowboy', 'filename': 'snowboy_test_2.wav'}, 
								   args_intent={'expected_intents': ['set_acceleration'], 'filename': 'intent_test_2.wav'}))

	save_to_json_file(args.filename, json_tests_list)
