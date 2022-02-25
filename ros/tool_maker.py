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
import json
import scipy.spatial

import geometry_msgs.msg

# using global variables to communicate with callback thread
collecting = False
number_of_markers = 0
records = []

# Arbitrary number to make sure we have enough records to average out noise etc.
minimum_records_required = 10

if sys.version_info.major < 3:
    input = raw_input


def pose_array_callback(msg):
    global records

    # exit if not recording marker pose messages
    if not collecting:
        return

    # make sure the number of poses matches the number of expected markers
    if len(msg.poses) == number_of_markers:
        record = []
        for marker in range(number_of_markers):
            record.append(
                [
                    msg.poses[marker].position.x,
                    msg.poses[marker].position.y,
                    msg.poses[marker].position.z,
                ]
            )
        records.append(record)
        sys.stdout.write("\rNumber of samples collected: %i" % len(records))
        sys.stdout.flush()


# ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
rospy.init_node("tool_maker", anonymous=True)
# strip ros arguments
argv = rospy.myargv(argv=sys.argv)

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument(
    "-t",
    "--topic",
    type=str,
    required=True,
    help="topic to use to receive PoseArray without namespace.  Use __ns:= to specify the namespace",
)
parser.add_argument(
    "-n",
    "--number-of-markers",
    type=int,
    choices=range(3, 10),
    required=True,
    help="number of markers on the tool.  This will use to filter messages with incorrect number of markers",
)
parser.add_argument(
    "-p",
    "--planar",
    action="store_true",
    help="indicates all markers lie in a plane",
)
parser.add_argument("-o", "--output", type=str, required=True, help="output file name")


args = parser.parse_args(argv[1:])  # skip argv[0], script name
number_of_markers = args.number_of_markers

# create the callback that will collect data
pose_array_subscriber = rospy.Subscriber(
    args.topic, geometry_msgs.msg.PoseArray, pose_array_callback
)

input("Press Enter to start collection using topic %s" % args.topic)
print("Collection started\nPress Enter to stop")
collecting = True

input("")
collecting = False

nb_records = len(records)

if nb_records < minimum_records_required:
    sys.exit("Not enough records ({} minimum)".format(minimum_records_required))

# now the fun part, each record has n poses but we don't know if they are sorted by markers

# create n lists to store the pose of each marker based on distance, using the last record as reference
reference = records[-1]

# create a records with markers sorted by proximity to reference order
sorted_records = [
    [
        record[scipy.spatial.distance.cdist([reference_marker], record).argmin()]
        for reference_marker in reference
    ]
    for record in records
]

# average position of each marker
averaged_marker_poses = numpy.mean(sorted_records, axis=0)
# center (average) of individual average marker positions
isocenter = numpy.mean(averaged_marker_poses, axis=0)
# center coordinate system on isocenter
points = averaged_marker_poses - isocenter

# SVD for PCA
_, sigma, Vt = numpy.linalg.svd(points, full_matrices=False)
planar_threshold = 1e-2

# Project markers to best-fit plane
if args.planar:
    print("Planar flag enabled, projecting markers onto plane...")
    Vt[2, :] = 0 # Remove 3rd (smallest) principal componenent to collapse points to plane

planarity = sigma[2]/sigma[1]
print(planarity)
print(sigma)
if args.planar and planarity > planar_threshold:
    print("WARNING: planar flag is enabled, but markers don't appear to be planar!")
elif not args.planar and planarity < planar_threshold:
    print("Markers appear to be planar. If so, add '--planar' flag")

# Apply PCA to align markers, and if planar to project to plane
points = numpy.matmul(points, Vt.T)

fiducials = [
    {"x": x, "y": y, "z": z}
    for [x, y, z] in points
]

data = {
    "count": number_of_markers,
    "fiducials": fiducials,
}

with open(args.output, "w") as f:
    json.dump(data, f, indent=4)
    f.write("\n")

print("Generated tool geometry file {}".format(args.output))
