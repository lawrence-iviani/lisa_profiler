#!/usr/bin/env python3
"""Play an audio file using a limited amount of memory.

The soundfile module (https://PySoundFile.readthedocs.io/) must be
installed for this to work.  NumPy is not needed.

In contrast to play_file.py, which loads the whole file into memory
before starting playback, this example program only holds a given number
of audio blocks in memory and is therefore able to play files that are
larger than the available RAM.

A similar example could of course be implemented using NumPy,
but this example shows what can be done when NumPy is not available.

example from https://github.com/spatialaudio/python-sounddevice/blob/0.4.1/examples/play_long_file.py

"""

from lisa_profiler import AudioParametersStruct, read_file_and_play,  query_devices
from lisa_profiler import get_test_token


import argparse
import queue
import sys
# todo: replace with ros sleep
from time import sleep


def int_or_str(text):
	"""Helper function for argument parsing."""
	try:
		return int(text)
	except ValueError:
		return text


if __name__ == "__main__":
	print("Starting with python {}".format( sys.version_info))
	if sys.version_info[0] < 3:
		raise Exception("Must be using Python 3")

	parser = argparse.ArgumentParser(add_help=False)
	parser.add_argument(
		'-l', '--list-devices', action='store_true',
		help='show list of audio devices and exit')
	args, remaining = parser.parse_known_args()
	if args.list_devices:
		print(query_devices())
		parser.exit(0)
	parser = argparse.ArgumentParser(
    	description=__doc__,
    	formatter_class=argparse.RawDescriptionHelpFormatter,
		parents=[parser])
	parser.add_argument(
		'filename', metavar='FILENAME',
		help='JSON audio description of the playback')
	parser.add_argument(
		'-d', '--device', type=int_or_str,
    help='output device (numeric ID or substring)')
	parser.add_argument(
		'-b', '--blocksize', type=int, default=2048,
		help='block size (default: %(default)s)')
	parser.add_argument(
		'-q', '--buffersize', type=int, default=15,
		help='number of blocks used for buffering (default: %(default)s)')
	args = parser.parse_args(remaining)
	if args.blocksize == 0:
		parser.error('blocksize must not be zero')
	if args.buffersize < 1:
		parser.error('buffersize must be at least 1')

	
	audio_params = AudioParametersStruct( device=args.device, buffersize=args.buffersize, blocksize=args.blocksize)
	# open instead of this snippet creating from: args.filename
	json_tests_list = []
	json_tests_list.append(get_test_token(wave_folder='/media/sf_Thesis/test track', 
								   args_wakeup={'expected_wakeup_word': 'snowboy', 'filename': 'snowboy_test_1.wav'}, # 'long.wav'}, # 'snowboy_test_1.wav'}, 
								   args_intent={'expected_intents': ['set_acceleration'], 'filename': 'intent_test_1.wav'}))
	json_tests_list.append(get_test_token(wave_folder='/media/sf_Thesis/test track', 
								   args_wakeup={'expected_wakeup_word': 'snowboy', 'filename': 'snowboy_test_2.wav'}, 
								   args_intent={'expected_intents': ['set_acceleration'], 'filename': 'intent_test_2.wav'}))

	def _run_test(filename, pause_before, pause_after, buffersize):
		
		print("sleeping BEFORE for {} sec.".format(pause_before ))
		sleep(pause_before)
		
		# TODO: emit ros message start wakeup reproduction
		print("Play file {}".format(filename))
		read_file_and_play(filename, audio_params, buffer_queue_size=buffersize )
		
		# TODO: wakeup expected, emit ros message end wakeup reproduction
		print("sleeping AFTER for {} sec.".format(pause_after ))
		sleep(pause_after)


	try:
		for t in json_tests_list:
			print("===== START TEST ITEM ====\n=============================\n")
			# wakeup
			print("===== WAKEUP ====")
			_run_test(t['wakeup']['filename'], t['wakeup']['pause_before'], t['wakeup']['pause_after'], args.buffersize)
			# sleep(2)

			#####
			print("\n===== INTENT ====")
			_run_test(t['intent']['filename'], t['intent']['pause_before'], t['intent']['pause_after'], args.buffersize)

			print("===== END TEST ITEM ====\n=============================\n")

	except KeyboardInterrupt:
		parser.exit('\nInterrupted by user')
	except Exception as e:
		parser.exit('\nError catched: ' + type(e).__name__ + ': ' + str(e))

