import sys
from datetime import datetime

def _print_debug(msg='', function="", intent_level=0):
	#datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
	msg = "[" + datetime.utcnow().strftime('%M:%S.%f')[:-3] + "]"+ intent_level * '\t' + function + "> " + msg
	print(msg, file=sys.stderr)

