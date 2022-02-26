#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2021-10-29

# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import sys
import argparse
import rospy
import numpy as np
import json
import math
import scipy.spatial
import scipy.optimize

import geometry_msgs.msg

if sys.version_info.major < 3:
    input = raw_input


# create subscription with callback to handle marker messages
def get_pose_data(ros_topic, expected_marker_count):
    records = []
    collecting = False

    def display_sample_count():
        sys.stdout.write("\rNumber of samples collected: %i" % len(records))
        sys.stdout.flush()

    def pose_array_callback(msg):
        # skip if not recording marker pose messages
        if not collecting:
            return

        # make sure the number of poses matches the number of expected markers
        if len(msg.poses) == expected_marker_count:
            record = []
            for marker in range(expected_marker_count):
                record.append(
                    (
                        msg.poses[marker].position.x,
                        msg.poses[marker].position.y,
                        msg.poses[marker].position.z,
                    )
                )

            records.append(record)
            display_sample_count()

    pose_array_subscriber = rospy.Subscriber(
        ros_topic, geometry_msgs.msg.PoseArray, pose_array_callback
    )

    input("Press Enter to start collection using topic %s" % ros_topic)
    print("Collection started\nPress Enter to stop")
    display_sample_count()
    collecting = True

    input("")
    collecting = False
    pose_array_subscriber.unregister()

    return records


# Apply PCA to align markers, and if is_planar to project to plane.
# Points data should have mean zero (i.e. be centered at origin).
# planar_threshold is maximium relative variance along third axis that is considerd planar
def principal_component_analysis(points, is_planar, planar_threshold=1e-2):
    # SVD for PCA
    _, sigma, Vt = np.linalg.svd(points, full_matrices=False)

    # Orientation should be (close to) +/-1
    basis_orientation = np.linalg.det(Vt)
    # Select positive orientation of basis
    if basis_orientation < 0.0:
        Vt[2, :] = -Vt[2, :]

    # Project markers to best-fit plane
    if is_planar:
        print("Planar flag enabled, projecting markers onto plane...")
        Vt[
            2, :
        ] = 0  # Remove 3rd (smallest) principal componenent to collapse points to plane

    planarity = sigma[2] / sigma[1]
    if is_planar and planarity > planar_threshold:
        print("WARNING: planar flag is enabled, but markers don't appear to be planar!")
    elif not is_planar and planarity < planar_threshold:
        print(
            "Markers appear to be planar. If so, add '--planar' flag for better results"
        )

    return np.matmul(points, Vt.T)


# now the fun part, each record has n poses but we don't know if they are sorted by markers
def process_marker_records(records, is_planar):
    # create n lists to store the pose of each marker based on distance, using the last record as reference
    reference = records[-1]

    # Find correspondence between reference and record markers that minimizes pair-wise Euclidean
    # distances, and put record markers into the same order as the corresponding markers in reference.
    ordered_records = [
        ordered_record
        for ordered_record in (
            [
                record[
                    scipy.spatial.distance.cdist([reference_marker], record).argmin()
                ]
                for reference_marker in reference
            ]
            for record in records
        )
        # skip records where naive-correspondence isn't one-to-one
        if len(set(ordered_record)) == len(reference)
    ]

    # average position of each marker
    averaged_marker_poses = np.mean(ordered_records, axis=0)
    # center (average) of individual average marker positions
    isocenter = np.mean(averaged_marker_poses, axis=0)
    # center coordinate system on isocenter
    points = averaged_marker_poses - isocenter
    # align using PCA and project to plane is is_planar flag is set
    points = principal_component_analysis(points, is_planar)

    return points


def write_data(points, output_file_name):
    fiducials = [{"x": x, "y": y, "z": z} for [x, y, z] in points]
    origin = {"x": 0.0, "y": 0.0, "z": 0.0}

    data = {
        "count": len(fiducials),
        "fiducials": fiducials,
        "pivot": origin,
    }

    with open(output_file_name, "w") as f:
        json.dump(data, f, indent=4, sort_keys=True)
        f.write("\n")

    print("Generated tool geometry file {}".format(output_file_name))


if __name__ == "__main__":
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
    parser.add_argument(
        "-o", "--output", type=str, required=True, help="output file name"
    )

    args = parser.parse_args(argv[1:])  # skip argv[0], script name

    # Arbitrary number to make sure we have enough records to average out noise etc.
    minimum_records_required = 10

    # create the callback that will collect data
    records = get_pose_data(args.topic, args.number_of_markers)
    if len(records) < minimum_records_required:
        sys.exit("Not enough records ({} minimum)".format(minimum_records_required))

    points = process_marker_records(records, args.planar)
    write_data(points, args.output)
