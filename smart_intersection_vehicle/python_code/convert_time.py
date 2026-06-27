import argparse
import datetime

# How to run this code in the terminal:
# "python convert_time.py 1677891782"

# create an argument parser
parser = argparse.ArgumentParser(description='Convert a Unix timestamp to a human-readable date and time format')

# add an argument for the timestamp
parser.add_argument('timestamp', type=int, help='a Unix timestamp in seconds')

# parse the arguments
args = parser.parse_args()

# convert the timestamp to a date and time object
date = datetime.datetime.fromtimestamp(args.timestamp)
# args.your_timestamp =  1677891782 
# print the date and time object
print(args.timestamp)
print(date)

