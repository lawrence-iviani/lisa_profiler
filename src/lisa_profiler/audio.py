

import sounddevice as sd
import soundfile as sf
import queue
from collections import namedtuple
from threading import Event

from ._helper import _print_debug

########################
### Data Definitions ###
########################
AudioParametersStruct = namedtuple('AudioParametersStruct', ['device', 'buffersize', 'blocksize'])

########### AUDIO ###########
def query_devices():
	return sd.query_devices()
	

def read_file_and_play(filename, audio_params): #, data_queue, event):
	IntenalStruct = namedtuple('IntenalStruct', ['max_retry', 'already_exiting' ])
	# need to share with the play_callback
	global data_queue, event, sample_per_block, duration_per_block, latency
	
	data_queue = queue.Queue(maxsize=audio_params.buffersize)
	event = Event()

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
