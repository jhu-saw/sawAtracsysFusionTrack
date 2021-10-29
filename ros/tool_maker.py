#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2021-10-29

# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import time
import sys
import argparse
import rospy
import numpy
import scipy.spatial

import geometry_msgs.msg

# using global variables to communicate with callback thread
collecting = False
number_of_markers = 0
records = []

if sys.version_info.major < 3:
    input = raw_input

def pose_array_callback(msg):
    global records
    # do something only if we are recording
    if collecting:
        # make sure the number of poses matches the number of expected markers
        if (len(msg.poses) == number_of_markers):
            record = []
            for marker in range(number_of_markers):
                record.append([msg.poses[marker].position.x,
                               msg.poses[marker].position.y,
                               msg.poses[marker].position.z])
            records.append(record)
            sys.stdout.write('\rNumber of samples collected: %i' % len(records))
            sys.stdout.flush()

# ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
rospy.init_node('tool_maker', anonymous=True)
# strip ros arguments
argv = rospy.myargv(argv=sys.argv)

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-t', '--topic', type = str, required = True,
                    help = 'topic to use to receive PoseArray without namespace.  Use __ns:= to specify the namespace')
parser.add_argument('-n', '--number-of-markers', type = int, choices = range(3, 10), required = True,
                    help = 'number of markers on the tool.  This will use to filter messages with incorrect number of markers')

args = parser.parse_args(argv[1:]) # skip argv[0], script name

# create the callback that will collect data
pose_array_subscriber = rospy.Subscriber(args.topic,
                                         geometry_msgs.msg.PoseArray,
                                         pose_array_callback)

number_of_markers = args.number_of_markers

input("Press Enter to start collection using topic %s" % args.topic)
print("Collection started\nPress Enter to stop")
collecting = True

input("")
collecting = False

nb_records = len(records)

if nb_records < 10:
     sys.exit("Not enough records (10 minimum)") # this is totally arbitrary

# now the fun part, each record has n poses but we don't know if they are sorted by markers

# create n lists to store the pose of each marker based on distance, using the last record as reference
reference = records.pop()

# create a records with markers sorted by proximity to reference order
sorted_records = []
sorted_records.append(reference)

# iterate through rest of list
for record in records:
    correspondence = [] # index of closest reference
    # find correspondence
    for marker in record:
        # find closest reference
        min = 100000.0 # arbitrary high
        closest_reference = -1
        for index_reference in range(0, len(reference)):
            distance = scipy.spatial.distance.euclidean(marker, reference[index_reference])
            if distance < min:
                min = distance
                closest_to_reference = index_reference
        correspondence.append(closest_to_reference)
    # create sorted record
    sorted_record = record # just to make sure we have the correct size
    for index in correspondence:
        sorted_record[correspondence[index]] = record[index]
    sorted_records.append(sorted_record)


print("------------- to do, compute sum/average for each value, then isocenter and finally subtract isocenter, then create ini or json file")
