from transitions import Machine

INVALID_VALUE = "NaN"

states=['wait_wakeup', 'wait_text', 'wait_intent']

# using Internal transitions, they  will never actually leave the state.
# This means that transition-related callbacks such as before or after will be processed while state-related callbacks exit or enter will not.
# To define a transition to be internal, set the destination to None.

transitions = [
    ## From wait_wakeup
    { 'trigger': 'waked_up_received', 'source': 'wait_wakeup', 'dest': 'wait_text','before': 'set_wakedup' },
    # Internal transition in wakeup (not leaving the state!)
    { 'trigger': 'wakeup_playback_started', 'source': 'wait_wakeup', 'dest': None, 'before': 'set_wakeup_playback_started'}, #'wait_wakeup' },
    { 'trigger': 'wakeup_playback_stopped', 'source': 'wait_wakeup', 'dest': None, 'before': 'set_wakeup_playback_stopped'}, #'wait_wakeup' },

    # From wait_text
    { 'trigger': 'text_recieved', 'source': 'wait_text', 'dest': 'wait_intent', 'before': 'set_text_captured' },
    # Internal transition in wakeup (not leaving the state!)
    { 'trigger': 'intent_playback_started', 'source': 'wait_text', 'dest': None, 'before': 'set_start_playback_intent' },
    { 'trigger': 'intent_playback_stopped', 'source': 'wait_text', 'dest': None, 'before': 'set_stop_playback_intent' },

    # From wait_intent
    { 'trigger': 'intent_recognized_recieved', 'source': 'wait_intent', 'dest': 'wait_wakeup', 'before': 'set_intent_recognized'},
    { 'trigger': 'intent_not_recognized_recieved', 'source': 'wait_intent', 'dest': 'wait_wakeup', 'before': 'set_intent_not_recognized' },
]

class WakeupTestModel(object):

    # possible outcomes
    NOT_STARTED = "NOT STARTED"
    WAKEUP_DETECTED = "WAKEDUP"
    WAKEUP_NOT_DETECTED = "NOT WAKEDUP"
    TEXT_TRANSCRIPTED = "TEXT ACQUIRED"
    INTENT_RECOGNIZED = "RECOGNIZED"
    INTENT_WRONG_RECOGNIZED = "WRONG RECOGNIZED"
    INTENT_NOT_RECOGNIZED = "NOT RECOGNIZED"
    UNKNOWN = "UNKNWON"

    _test_state_dict = {}

    def __init__(self):
        self._result_counter = 0
        self._reset_timers()

    #############################
    ###  SM functions related ###
    #############################
    def on_enter_wait_wakeup(self, data=None):
        print('on_enter_wait_wakeup', data)
        self.persist_state_and_reset("item_" + str(self._result_counter) )
        self._result_counter += 1

    def set_wakeup_playback_started(self, data=None):
        self._set_timing('start_wakeup_pb', data['timestamp'])

    def set_wakeup_playback_stopped(self, data=None):
        self._set_timing('stop_wakeup_pb', data['timestamp'])

    def set_wakedup(self, data=None):
        self._set_timing('waked_up', data['timestamp'])

    def set_start_playback_intent(self, data=None):
        self._set_timing('start_intent_pb', data['timestamp'])

    def set_stop_playback_intent(self, data=None):
        self._set_timing('stop_intent_pb', data['timestamp'])

    def set_text_captured(self, data=None):
        self._set_timing('text_captured', data['timestamp'])

    def set_intent_recognized(self, data=None):
        self._set_timing('intent_recognized', data['timestamp'])

    def set_intent_not_recognized(self, data=None):
        self._set_timing('intent_not_recognized', data['timestamp'])


    #def on_exit_wait_wakeup(self, data=None):
    #    print('on_exit_wait_wakeup',data)

    #def on_enter_wait_text(self, data=None):
    #    print('on_enter_wait_text',data)

    ###########################################
    ###  utilities functions and properties ###
    ###########################################
    def _set_timing(self, timer_name, value):
        _self_timer_name = "_t_" + timer_name
        print('_set_timing', timer_name, value)
        if hasattr(self, _self_timer_name):
            assert isinstance(value, float), "Time value must be a float, instead is " + str(value.__class__)
            setattr(self, _self_timer_name, value)
        else:
            print("!!!Warning!!! not existent timing " + _self_timer_name + " skipping")

    def _reset_timers(self):
        self._t_start_wakeup_pb = None
        self._t_stop_wakeup_pb = None
        self._t_waked_up = None
        self._t_start_intent_pb = None
        self._t_stop_intent_pb = None
        self._t_text_captured = None
        self._t_intent_recognized = None
        self._t_intent_not_recognized = None

    @property
    def result(self):
        if self._t_start_wakeup_pb is None:
            return self.NOT_STARTED
        if self._t_stop_wakeup_pb is None:
            return self.NOT_STARTED
        if self._t_waked_up is None:
            return self.WAKEUP_NOT_DETECTED
        if self._t_start_intent_pb is None:
            return self.WAKEUP_DETECTED
        if self._t_stop_intent_pb is None:
            return self.WAKEUP_DETECTED
        if self._t_text_captured is None:
            return self.WAKEUP_DETECTED
        if self._t_intent_recognized is None and self._t_intent_recognized is None :
            return self.TEXT_TRANSCRIPTED
        if self._t_intent_recognized is not None:
            # TOOD: check the expected intnent and payload were recognized
            return self.INTENT_RECOGNIZED
        if self._t_intent_not_recognized is not None:
            return self.INTENT_NOT_RECOGNIZED

        return self.UNKNOWN

    @staticmethod
    def _perform_difference(b, a):
        if b is not None and a is not None:
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
        self._reset_timers()

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
        print("----->>>>>> Results >{}< wakeup_time={}s. stt_time={}s. intent_recognition_time={}s.".format(
                retval_dict['outcome'], retval_dict['wakeup_time'], retval_dict['stt_time'], retval_dict['intent_recognition_time'], ))
        # TODO: CPU performance
        return retval_dict

    def set_value(self):
        pass

wake_up_test_model = WakeupTestModel()
wake_up_test_sm = Machine(wake_up_test_model, queued=True,  states=states, transitions=transitions, initial='wait_wakeup')
