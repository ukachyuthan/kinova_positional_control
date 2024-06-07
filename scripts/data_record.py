import os
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument(
    'filename',
    metavar='N',
    type=str,
    nargs='+',
    help='an integer for the accumulator'
)
args = parser.parse_args()
name_of_file = args.filename[-1]

os.system(
    'rosbag record -O' + name_of_file +
    '.bag /updated_physical_workload /Pupil /left/disengage_time /right/disengage_time'
)
