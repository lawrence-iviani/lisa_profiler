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

import argparse
import queue
import sys
# todo: replace with ros sleep
from time import sleep

from lisa_profiler import AudioParametersStruct, run_file_test,  query_devices, load_from_json_file, ros_init


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
		'-q', '--buffersize', type=int, default=20,
		help='number of blocks used for buffering (default: %(default)s)')
	args = parser.parse_args(remaining)
	if args.blocksize == 0:
		parser.error('blocksize must not be zero')
	if args.buffersize < 1:
		parser.error('buffersize must be at least 1')
	
	audio_params = AudioParametersStruct( device=args.device, buffersize=args.buffersize, blocksize=args.blocksize)
	try:
		json_tests_list = load_from_json_file(args.filename)
		ros_publishers_dict = ros_init()
		print("Loaded " + str(len(json_tests_list)) + " test items...\nStarting test")
		run_file_test(json_tests_list, audio_params, ros_publishers_dict)
	except KeyboardInterrupt:
		parser.exit('\nInterrupted by user')
	except Exception as e:
		parser.exit('\nError catched: ' + type(e).__name__ + ': ' + str(e))

