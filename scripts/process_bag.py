#!/usr/bin/env python3

import rosbag_pandas
import os
import argparse

from lisa_profiler import run_process_bag, analyse_df


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
	                    action="store_true", default=False)
	parser.add_argument('-v', '--verbose',
	                    help="Log verbose",
	                    default=False, action="store_true")
	return parser


if __name__ == '__main__':
	import rosbag_pandas
	parser = build_parser()
	args = parser.parse_args()
	print(args.output)

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
	dict_analised = analyse_df(df_processed)

	if args.output:
		df_processed.to_csv(args.bag + ".csv")
		for k,v in dict_analised.items():
			df_a = v
			df_a.to_csv(args.bag +"_"+ k +".csv")
	else:
		df_processed.info()
		print(df_processed)
		for k,v in dict_analised.items():
			print('Result {}\n{}\n-----'.format(k,v))
