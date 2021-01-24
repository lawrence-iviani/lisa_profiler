from transitions import Machine
from word2number import w2n
import json

from . import INVALID_VALUE

states=['wait_wakeup', 'wait_text', 'wait_intent']

# using Internal transitions, they  will never actually leave the state.
# This means that transition-related callbacks such as before or after will be processed while state-related callbacks exit or enter will not.
# To define a transition to be internal, set the destination to None.

transitions = [
    ## From wait_wakeup
    { 'trigger': 'waked_up_received', 'source': 'wait_wakeup', 'dest': 'wait_text','before': 'set_wakedup' },
    # Internal transition in wakeup (not leaving the state!)
    { 'trigger': 'wakeup_playback_started', 'source': 'wait_wakeup', 'dest': None, 'before': 'set_wakeup_playback_started'},
    { 'trigger': 'wakeup_playback_stopped', 'source': 'wait_wakeup', 'dest': None, 'before': 'set_wakeup_playback_stopped'},

    ## From wait_text
    { 'trigger': 'text_recieved', 'source': 'wait_text', 'dest': 'wait_intent', 'before': 'set_text_captured' },
    # Internal transition in wakeup (not leaving the state!)
    { 'trigger': 'intent_playback_started', 'source': 'wait_text', 'dest': None, 'before': 'set_start_playback_intent'},
    { 'trigger': 'intent_playback_stopped', 'source': 'wait_text', 'dest': None, 'before': 'set_stop_playback_intent'},
    # it can happen that a wakeup is detected after the execution?
    { 'trigger': 'wakeup_playback_stopped', 'source': 'wait_text', 'dest': None, 'before': 'set_wakeup_playback_stopped'},

    ## From wait_intent
    # Regular end
    { 'trigger': 'intent_recognized_recieved', 'source': 'wait_intent', 'dest': 'wait_wakeup', 'before': 'set_intent_recognized'},
    { 'trigger': 'intent_not_recognized_recieved', 'source': 'wait_intent', 'dest': 'wait_wakeup', 'before': 'set_intent_not_recognized' },

    ## Backup trigger, used if none of the above are executed.
    # ORDER MATTERS: Accordinlgly to transitions documentation:
    # (Note that only the first matching transition will execute; thus, the transition defined in the last line above won't do anything.)

    # Entering in wait_wake up reset the actual session, the after set_wakeup_playback_started persist the time value of playbacl start
    { 'trigger': 'wakeup_playback_started', 'source': '*', 'dest': 'wait_wakeup', 'after': 'set_wakeup_playback_started'},

    # From these states is legal go back to wait_wake up. The test has been succesful for
    # missed wake up, both are needed
    #{ 'trigger': 'intent_playback_started', 'source': 'wait_wakeup', 'dest': 'wait_wakeup'},
    #{ 'trigger': 'intent_playback_stopped', 'source': 'wait_wakeup', 'dest': 'wait_wakeup'},

    { 'trigger': 'reset', 'source': '*', 'dest': 'wait_wakeup'},
]

class WakeupTestModel(object):

    # possible outcomes
    NOT_STARTED = "NOT STARTED"
    WAKEUP_DETECTED = "WAKEDUP"
    WAKEUP_NOT_DETECTED = "NOT WAKEDUP"
    WAKEUP_WRONG_DETECTION = "WRONG WAKEDUP"
    TEXT_TRANSCRIPTED = "TEXT ACQUIRED"
    INTENT_RECOGNIZED = "RECOGNIZED"
    INTENT_WRONG_RECOGNIZED = "WRONG RECOGNIZED"
    INTENT_NOT_RECOGNIZED = "NOT RECOGNIZED"
    UNKNOWN = "UNKNWON"

    _test_state_dict = {}

    def __init__(self):
        self._reset_state()

    #############################
    ###  SM functions related ###
    #############################
    def on_enter_wait_wakeup(self, data=None):
        self.persist_state_and_reset("transition_" + str(data['transition']) )

    def set_wakeup_playback_started(self, data=None):
        self._set_wakeup_expected(data)
        self._set_timing('start_wakeup_pb', data['timestamp'])

    def set_wakeup_playback_stopped(self, data=None):
        self._set_timing('stop_wakeup_pb', data['timestamp'])

    def set_wakedup(self, data=None):
        self._set_wakeup_received(data)
        self._set_timing('waked_up', data['timestamp'])

    def set_start_playback_intent(self, data=None):
        self._set_intent_expected(data)
        self._set_timing('start_intent_pb', data['timestamp'])

    def set_stop_playback_intent(self, data=None):
        self._set_timing('stop_intent_pb', data['timestamp'])

    def set_text_captured(self, data=None):
        self._set_timing('text_captured', data['timestamp'])

    def set_intent_recognized(self, data=None):
        self._set_intent_received(data)
        self._set_timing('intent_recognized', data['timestamp'])

    def set_intent_not_recognized(self, data=None):
        self._set_timing('intent_not_recognized', data['timestamp'])

    ###########################################
    ###  utilities functions and properties ###
    ###########################################
    def _set_timing(self, timer_name, value):
        _self_timer_name = "_t_" + timer_name
        #print('_set_timing', timer_name, value)
        if hasattr(self, _self_timer_name):
            assert isinstance(value, float), "Time value must be a float, instead is " + str(value.__class__)
            setattr(self, _self_timer_name, value)
        else:
            print("!!!Warning!!! not existent timing " + _self_timer_name + " skipping")

    def _set_intent_received(self, data):
        assert isinstance(data, dict) and 'payload' in data.keys(), "Payload must be a dict containing a key payload" + str(payload)
        self._intent_received_data = data['payload']

    def _set_wakeup_received(self, data):
        assert isinstance(data, dict) and 'payload' in data.keys(), "Payload must be a dict containing a key payload" + str(payload)
        self._wakeup_received_data = data['payload']

    def _set_intent_expected(self, data):
        assert isinstance(data, dict) and 'payload' in data.keys(), "Payload must be a dict containing a key payload" + str(payload)
        self._intent_expected_data = json.loads(data['payload']['data'])

        ### HOTFIXs  ###
        ### for ExecutePointN the item pay_load/1 is not properly defined and not necessary.
        ### it cause false recognition and improper counting
        # print(self._intent_expected_data)
        for _expected_itent in self._intent_expected_data['expected_intents']:
            if _expected_itent['intent_name'] == 'ExecutePointN':
                if 'pay_load/1' in _expected_itent.keys():
                    print('Hot fix for ExecutePointN, removing key '.format('pay_load/1', _expected_itent['pay_load/1']))
                    del  _expected_itent['pay_load/1'] # This will raise a KeyError if the key is not in the dictionary.
                if 'pay_load/2' in _expected_itent.keys():
                    _expected_itent['pay_load/2'] = _expected_itent['pay_load/2'].replace('target', 'N')
                    print('Hot fix for ExecutePointN, changing  key from target to N - {}'.format(_expected_itent['pay_load/2']))

            # in the description file is set as velocity, but the tester was asked to utter acceleration :(
            elif _expected_itent['intent_name'] == 'MotionParamSet':
                if 'pay_load/1' in _expected_itent.keys():
                    print('Hot fix for MotionParamSet, changing value in key {}  from |{}| to |{}|'.format('pay_load/1', _expected_itent['pay_load/1'], 'target=acceleration'))
                    _expected_itent['pay_load/1'] = 'target=acceleration'

    def _set_wakeup_expected(self, data):
        assert isinstance(data, dict) and 'payload' in data.keys(), "Payload must be a dict containing a key payload" + str(payload)
        self._wakeup_expected_data =  json.loads(data['payload']['data'])

    def _reset_state(self):
        self._t_start_wakeup_pb = None
        self._t_stop_wakeup_pb = None
        self._t_waked_up = None
        self._t_start_intent_pb = None
        self._t_stop_intent_pb = None
        self._t_text_captured = None
        self._t_intent_recognized = None
        self._t_intent_not_recognized = None
        self._intent_expected_data = None
        self._intent_received_data = None
        self._wakeup_expected_data = None
        self._wakeup_received_data = None

    def _are_compatible_expected_and_received_intent(self):
        _expected_intents = self._intent_expected_data['expected_intents']
        _rcv_data =  self._intent_received_data
        assert isinstance(_expected_intents, list)
        print("_are_compatible_expected_and_received_intent: expected intent\n\t==== {}\nReceived intent\n\t==== {}".format(
                    _expected_intents,  self._intent_received_data))
        for _exp_intent in _expected_intents: # there can be more than one intent??
            if _exp_intent['intent_name'] != _rcv_data['intent_name']:
                continue
            # check if a payload is as expected
            _correct_pl = True
            for pl in  [p for p in _exp_intent if p.startswith('pay_load')]:
                print("Searching for {} ".format(pl), end = ' ')

                if pl in _rcv_data.keys():
                    try:
                        _kval = _rcv_data[pl].split('=')
                        assert len(_kval) == 2, 'Bad format? {} - {}'.format(pl, _rcv_data[pl])
                        # print("Modifying |{}| ".format(_kval[1]), end = "+++ ")
                        _rcv_data[pl] = str(_kval[0]) + "=" + str(w2n.word_to_num(_kval[1]))
                        print('Modified pl[{}]={}'.format(pl, _rcv_data[pl]), end = "+++ ")
                    # except TypeError as e:
                    #     print('Unmodified pl[{}]={}'.format( pl, _rcv_data[pl]), end = "+++ ")
                    except ValueError as e:
                        print('Unmodified pl[{}]={}'.format( pl, _rcv_data[pl]), end = "+++ ")

                    if _rcv_data[pl] == _exp_intent[pl]:
                        print('\tFound it, ', pl, _exp_intent[pl])
                        continue
                    else:
                        print('\tFound it pl[{}] but wrong value {} - expected was {}'.format(pl,  _rcv_data[pl]  ,_exp_intent[pl], ))
                        _correct_pl = False
                        break
                else:
                    print('\tNot Correct pl {} - '.format(pl))
                    _correct_pl = False
                    break
            # Found pl?
            if _correct_pl:
                print("Correct!")
                return True

        print("FALSE !!!!!!!!!!!!!!!!!!!!!!!!")
        return False

    @property
    def result(self):
        # Running in sequence of posible events. It is a simple state machine
        # where the reset is the possible outcome from every states
        # to restart the sm from the state on the top
        if self._t_start_wakeup_pb is None:
            return self.NOT_STARTED
        assert self._wakeup_expected_data is not None, "Cannot be a wakeup recognized without expected data"
        if self._t_stop_wakeup_pb is None:
            return self.NOT_STARTED
        if self._t_waked_up is None:
            return self.WAKEUP_NOT_DETECTED
        assert self._wakeup_received_data is not None, "Cannot be a wakeup recognized without received data"
        if self._wakeup_expected_data['expected_wakeup_word'] != self._wakeup_received_data['wakeup_word']:
            return self.WAKEUP_WRONG_DETECTION
        if self._t_start_intent_pb is None:
            return self.WAKEUP_DETECTED
        if self._t_stop_intent_pb is None:
            return self.WAKEUP_DETECTED
        if self._t_text_captured is None:
            return self.WAKEUP_DETECTED
        if self._t_intent_recognized is None and self._t_intent_recognized is None :
            return self.TEXT_TRANSCRIPTED
        if self._t_intent_recognized is not None:
            assert self._intent_received_data is not None, "Cannot be an intent recognized without received data"
            if self._are_compatible_expected_and_received_intent():
                return self.INTENT_RECOGNIZED
            else:
                return self.INTENT_WRONG_RECOGNIZED
            return self.INTENT_RECOGNIZED
        if self._t_intent_not_recognized is not None:
            return self.INTENT_NOT_RECOGNIZED

        return self.UNKNOWN

    @staticmethod
    def _perform_difference(b, a):
        if b is not None and a is not None:
            if a > b:
                print("!!!Warning!!! time difference must be monotonic. Forcing to 0.0 instead of " + str(b-a))
                return 0.0
            return b - a
        else:
            return INVALID_VALUE

    @property
    def time_for_waking_up(self):
        return self._perform_difference(self._t_waked_up, self._t_stop_wakeup_pb )

    @property
    def time_for_stt(self): # text to speech
        return self._perform_difference(self._t_text_captured, self._t_stop_intent_pb )

    @property
    def time_for_tti(self): # text to intent
        assert self._t_intent_recognized is None or self._t_intent_not_recognized is None, "Intent recognition times must be mutual exclusive"
        if self._t_intent_not_recognized is None:
            _recogn_time = self._t_intent_recognized
        else:
            _recogn_time = self._t_intent_not_recognized
        return self._perform_difference(_recogn_time, self._t_text_captured )

    def persist_state_and_reset(self, name=""):
        self.persist_state(name=name)
        self._reset_state()

    def persist_state(self, name=""):
        assert name is not None and isinstance(name, str) and len(name), "Invalid name to persist test result: " + str(name)
        if name in self._test_state_dict.keys():
            print("!!!Warning!!! existing result, overwriting result " + name)
        self._test_state_dict[name] = self.get_result()

    def get_result(self):
        retval_dict = {}
        retval_dict['outcome'] = self.result
        retval_dict['wakeup_time'] = self.time_for_waking_up
        retval_dict['stt_time'] = self.time_for_stt
        retval_dict['intent_recognition_time'] = self.time_for_tti
        #print("----->>>>>> Results >{}< wakeup_time={}s. stt_time={}s. intent_recognition_time={}s.".format(
        #        retval_dict['outcome'], retval_dict['wakeup_time'], retval_dict['stt_time'], retval_dict['intent_recognition_time'], ))
        # TODO: CPU performance
        return retval_dict

    def get_all_results(self):
        return self._test_state_dict


wake_up_test_model = WakeupTestModel()
wake_up_test_sm = Machine(wake_up_test_model, queued=True,  states=states, transitions=transitions, initial='wait_wakeup')
