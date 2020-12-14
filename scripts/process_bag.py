#!/usr/bin/env python3

import rosbag_pandas
import os
import argparse

from lisa_profiler import run_process_bag, analyse_df


def do_work(bag, include, exclude, output, fill, header):
	# covert a lenght one value to a regex
	if include is not None and len(include) == 1:
		include = include[0]

	# covert a lenght one value to a regex
	if exclude is not None and len(exclude) == 1:
		exclude = exclude[0]
	df = rosbag_pandas.bag_to_dataframe(bag, include=include, exclude=exclude, parse_header=header)
	if fill:
		df = df.ffill().bfill()

	if output is None:
		base, _ = os.path.splitext(bag)
		output = base + '.csv'
	print(df)
	df = rosbag_pandas.clean_for_export(df)

	df.to_csv(output)

def build_parser():
	"""
	Builds the parser for reading the command line arguments
	:return: Argument parser
	"""
	parser = argparse.ArgumentParser(description='Print one or multiple bag keys')
	parser.add_argument('-b', '--bag', help='Bag file to read',
	                    required=True, type=str)
	parser.add_argument('-k', '--key',
	                    help='Key you would like to print',
	                    required=False, nargs='*')
	parser.add_argument('-o', '--output', help='Output in CSV',
	                    required=False, type=str)
	parser.add_argument('-v', '--verbose',
	                    help="Log verbose",
	                    default=False, action="store_true")
	return parser


if __name__ == '__main__':
	import rosbag_pandas
	parser = build_parser()
	args = parser.parse_args()

	if args.verbose:
	    logging.getLogger().setLevel(logging.DEBUG)
	if args.key is None:
		df = rosbag_pandas.bag_to_dataframe(args.bag)
	else:
		topics = rosbag_pandas.topics_from_keys(args.key)
		print("Key", args.key,"\nTopics:", topics)
		df = rosbag_pandas.bag_to_dataframe(args.bag, include=topics)
	df.info()

	df_processed = run_process_bag(df)
	analyse_df(df_processed)

	if hasattr(args, 'output'):
		df_processed.to_csv(args.bag + ".csv")
	else:
		df_processed.info()
		print(df_processed)
