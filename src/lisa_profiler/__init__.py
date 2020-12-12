						
import sys		
from collections import namedtuple
import sounddevice as sd
import soundfile as sf
import json
import os.path
import queue
import threading
import rospy

# only temporary for debug
from datetime import datetime


########################
### Data Definitions ###
########################
AudioParametersStruct = namedtuple('AudioParametersStruct', ['device', 'buffersize', 'blocksize'])


#################
### Constants ###
#################
DEFAULT_PAUSE_BEFORE = 0.0 # silence before in sec.
DEFAULT_PAUSE_AFTER = 0.75 # silence after in sec. 


#################
### Functions ###
#################

def _print_debug(msg='', function="", intent_level=0):
	#datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
	msg = "[" + datetime.utcnow().strftime('%M:%S.%f')[:-3] + "]"+ intent_level * '\t' + function + "> " + msg
	print(msg, file=sys.stderr)


########### AUDIO ###########
def query_devices():
	return sd.query_devices()
	

def read_file_and_play(filename, audio_params): #, data_queue, event):
	IntenalStruct = namedtuple('IntenalStruct', ['max_retry', 'already_exiting' ])
	# need to share with the play_callback
	global data_queue, event, sample_per_block, duration_per_block, latency
	
	data_queue = queue.Queue(maxsize=audio_params.buffersize)
	event = threading.Event()

	# these are init later
	internal_data = IntenalStruct(max_retry = 10, already_exiting=False)
	sample_per_block = 1 
	duration_per_block = 1 
	latency = 0

	def play_callback(outdata, frames, time, status):
		'''
		Callback called by the system for audio reproduction
		Takes the data from the internal data_queue and ship to the sound card
		'''
		#_print_debug(function='play_callback', intent_level=2, msg='ENTER --- time={}, frames_to_play={} - status=>{}<'.format(time, frames, status))
		assert frames == audio_params.blocksize, "Mismatch in frames number (received {} frames) expected are {}".format(frames, audio_params.blocksize)
		if status.output_underflow:
			# TODO: during stop this can happen
			if internal_data.already_exiting:
				return
			_print_debug(function='play_callback', intent_level=2, msg='EXIT --- Output underflow: increase blocksize?')
			raise sd.CallbackAbort
		assert not status
		try:
			data = data_queue.get_nowait()
			#_print_debug(function='play_callback', intent_level=2, msg='got data={} from {} which size is {}'.format(data, data_queue, data_queue.qsize()))
			_print_debug(function='play_callback', intent_level=2, msg='got data chunk of {} msec.'.format(1000*duration_per_block*len(data)/len(outdata)))
			len(data)
		except queue.Empty as e:
			if internal_data.already_exiting:
				return
			_print_debug(function='play_callback', intent_level=2, msg='Buffer is empty: increase buffersize?')
			_print_debug(function='play_callback', intent_level=2, msg='queue.Empty! RETRY='+str(internal_data.max_retry))
			if internal_data.max_retry==0:		
				_print_debug(function='play_callback', intent_level=2, msg='EXIT EMPTY')
				raise sd.CallbackAbort from e
			internal_data.max_retry -= 1
			return

		if len(data) < len(outdata):
			_print_debug(function='play_callback', intent_level=2, msg='prepare for stop len(data)={} < len(outdata)={}'.format( len(data) , len(outdata) ))
			# The final data are copied to the sound card ...
			outdata[:len(data)] = data
			outdata[len(data):] = b'\x00' * (len(outdata) - len(data))
			# ... and wait before the stop, otherwise it cuts the latest reproduction!
			# ... or get an exception... 
			# somehow the sum of the latest chunk plus the latency for reproduction. Empiric but seems work
			already_exiting = True
			wait_ms = int((latency + duration_per_block)*1000)
			_print_debug(function='play_callback', intent_level=2, msg='EXIT STOP! wait {} msec.'.format(wait_ms))
			sd.sleep(wait_ms) # in msec
		else:
			#_print_debug(function='play_callback', intent_level=2, msg='EXIT outdata regular {}'.format(data))
			outdata[:] = data

	def play_finished():
		_print_debug(function="play_finished", intent_level=3, msg="event.set() {}".format(event))
		event.set()
		_print_debug(function="play_finished", intent_level=3, msg="exit")

	_print_debug(function='read_file_and_play', msg="Open file: {}".format(sf.info(filename)) )
	with sf.SoundFile(filename) as f:
		_file_chunk_size = 2 # TODO: this must be derived from the format, now we assume PCM_16, but a different format would lead wrong calculation		
		_duration_file = f.frames /  f.samplerate # see https://pysoundfile.readthedocs.io/en/latest/_modules/soundfile.html#info
		_total_file_read_sec = 0.0 # keep memory of the file read in seconds
		_print_debug(function='read_file_and_play', msg="file length: {} samples - {} sec.".format(f.frames, _duration_file))
		for _n in range(audio_params.buffersize):
			data = f.buffer_read(audio_params.blocksize, dtype='float32')
			if not data:
				#_print_debug(function='read_file_and_play', msg="break no data to prefill")
				break
			# _print_debug(function='read_file_and_play', msg="Prefill queue[{}] buffer_{} with {} len={}".format(data_queue, _n, data, len(data[:])))
			data_queue.put_nowait(data)  # Pre-fill queue
			#_print_debug(function='read_file_and_play', msg="Prefill queue[{}] qsize is {}".format(data_queue,data_queue.qsize()))
		# At this point the prefilled buffer is _total_file_read_sec
		# If the prebuffering is longer than the whole file the pre buffered part is the duration of the file
		_total_file_read_sec = min(_duration_file * data_queue.qsize() * audio_params.blocksize/f.frames, _duration_file)		
		_print_debug(function='read_file_and_play', msg="Prefill ended ---  prefilled {} sec. - buffer queue obj {},  -- now opening rawstream ".format(
					 _total_file_read_sec, data_queue))
		stream = sd.RawOutputStream(samplerate=f.samplerate, blocksize=audio_params.blocksize,
									device=audio_params.device, channels=f.channels, dtype='float32',
									callback=play_callback, finished_callback=play_finished) 

		# update the value of duration per block, it is used in the callback
		sample_per_block = round(stream.blocksize / (stream.channels * stream.samplesize))
		duration_per_block = sample_per_block /  stream.samplerate
		latency = stream.latency  # latency of the output
		_print_debug(function='read_file_and_play', msg="Output Audio Stream opened: {} - device={} sr={}Hz,blocksize={},n_channels={},samplesize={} ".format(
						stream, audio_params.device, stream.samplerate, stream.blocksize, stream.channels, stream.samplesize))
		_print_debug(function='read_file_and_play', msg="stream data block length: {} samples - {} sec. - latency = {}".format(
						sample_per_block, duration_per_block, latency))
		try:
			with stream:
				timeout = audio_params.blocksize * audio_params.buffersize/ _file_chunk_size / f.samplerate
				_print_debug(function='read_file_and_play', 
							 msg="START STREAM FROM FILE with a timeout {} sec. = blocksize({}) * buffersize({}) / samplerate({})".format(
							 	  timeout, audio_params.blocksize, audio_params.buffersize, f.samplerate))
				#_print_debug(function='read_file_and_play', msg="START STREAM FROM FILE with a timeout {} sec.".format(timeout))
				data = f.buffer_read(audio_params.blocksize, dtype='float32')
				while data:
					data = f.buffer_read(audio_params.blocksize, dtype='float32')
					#_print_debug(function='read_file_and_play', msg="stream from file data obj: {} - len {} bytes".format(data, len(data[:]) ))
					time_chunk = _duration_file * len(data[:])/f.frames / _file_chunk_size
					_total_file_read_sec += time_chunk				
					_print_debug(function='read_file_and_play', msg="stream from file data chunk: {} len is {} sec., total read is {} sec.".format(
									data, time_chunk, _total_file_read_sec ))
					try:
						data_queue.put(data, timeout=timeout)
					except queue.Full as e:
						wait_ms = 1000*int(len(data[:])/f.samplerate)
						_print_debug(function='read_file_and_play', msg='data available in file but queue is full... wait {} msec. and retry'.format(wait_ms))	
						sd.sleep(wait_ms)
						data_queue.put(data, timeout=timeout)
					# _print_debug(function='read_file_and_play', msg="stream from file queue[{}] qsize is {}".format(data_queue,data_queue.qsize()))
				_print_debug(function='read_file_and_play', msg="----- STOP STREAM FROM FILE! data are gone. Wait for event {}".format(event))
				event.wait()  # Wait until playback is finished
				_print_debug(function='read_file_and_play', msg="----- EXIT CLEAN - EVENT ALL DONE - ")
		except Exception as e:
			_print_debug(function='read_file_and_play', msg='---- END ERROR - Got exception in read_file_and_play:' + type(e).__name__ + ': ' + str(e))
			raise e


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
		print(key, value)
		if key == 'filename':
			value = os.path.join(wave_folder, value)        
		ret_dict[key] = value
	return ret_dict


def get_intent_token(wave_folder, args):
	'''
	Helper function for create a dict representing an intent token of the test
	'''
	ret_dict = {'filename':  '',
				'expected_intents': [], 
				'pause_before': DEFAULT_PAUSE_BEFORE, 
				'pause_after': DEFAULT_PAUSE_AFTER}
	for key, value in args.items():
		if key == 'filename':
			value = os.path.join(wave_folder, value)    
		elif key == 'expected_intents':
			if isinstance(value,str):
				value = [value]
		ret_dict[key] = value
	return ret_dict


def get_test_token(wave_folder, args_wakeup, args_intent):
	'''
	Helper function for create a dict representing one token of the test
	'''
	return {'wakeup': get_wakeup_token(wave_folder, args_wakeup), 
			'intent': get_intent_token(wave_folder, args_intent)}


def get_json_test_token(wave_folder, args_wakeup, args_intent):
	return json.dumps(get_test_token(wave_folder, args_wakeup, args_intent))


def save_to_json_file(filename, json_tests_list):
	with open(filename, 'w') as outfile:
		json.dump(json_tests_list, outfile)


def load_from_json_file(filename):
	with open(filename, 'r') as infile:
		data = json.load(infile)
	# TODO: here some check should be performed
	return data


########### ROS ###########
from std_msgs.msg import Empty
def ros_init():
	ret_dict = {}
	ret_dict['pub_start_wakeup'] = rospy.Publisher('start_wakeup', Empty, queue_size=10)
	ret_dict['pub_stop_wakeup'] = rospy.Publisher('stop_wakeup', Empty, queue_size=10)
	ret_dict['pub_start_intent'] = rospy.Publisher('start_intent', Empty, queue_size=10)
	ret_dict['pub_stop_intent'] = rospy.Publisher('stop_intent', Empty, queue_size=10)
	rospy.init_node('lisa_profiler', anonymous=True)
	return ret_dict

