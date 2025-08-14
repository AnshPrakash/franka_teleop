#!/usr/bin/env python3

"""
Adapted from Sophie's recording script

Node to record a rosbag with start/stop/pause control through service calls.
Example call:
    rosrun utilities rosbag_controlled_recording.py _rosbag_command:="rosbag record -o /home/foo/test_bag /bar_topic" _record_from_startup:=false
Then start/pause/resume/stop can be controlled through:
    rosservice call /rosbag_controlled_recording/start
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/stop
Note that pausing does not modify the recorded time of messages, i.e. the bag's total length is unaffected. A list of
  pause-resume times is logged when stopping, in case the paused period needs to be (manually) removed afterwards.
If this node is killed recording is also stopped. If recording was paused, it is momentarily resumed before stopping.
"""
from __future__ import annotations


import rospy
import h5py
from datetime import datetime
import os
import numpy as np



def format_to_columns(input_list, cols):
    """Adapted from https://stackoverflow.com/questions/171662/formatting-a-list-of-text-into-columns"""
    max_width = max(map(len, input_list))
    justify_list = map(lambda x: x.ljust(max_width + 4), input_list)
    lines = (''.join(justify_list[i:i + cols]) for i in range(0, len(justify_list), cols))
    return '\n'.join(lines)


class RobomimicControlledRecorder(object):
    """Record a rosbag with service calls to control start, stop and pause"""

    def __init__(self, save_folder, topics):
        os.makedirs(save_folder, exist_ok=True)
        self.save_folder = save_folder
        self.topics = topics

        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []
        self.hdf5_file: str = None
        self.h5 : h5py.File = None

        # Internal buffers
        self.actions = []
        self.obs = {topic: [] for topic in topics}
        self.states = []

        if not self.topics:
            raise ValueError("No topics provided for recording.")

    def start_recording(self):
        if self.recording_started:
            rospy.logwarn("Recording has already started - nothing to be done")
            return

        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.hdf5_file = os.path.join(self.save_folder, f"demo_{current_time}.hdf5")
        self.h5 = h5py.File(self.hdf5_file, 'w')

        rospy.loginfo(f"Started recording into {self.hdf5_file}")
        self.recording_started = True
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times.clear()

    def snap(self, action: np.array):
        if not self.recording_started or self.recording_paused:
            rospy.logwarn("Cannot snap — recording not started or paused")
            return

        # Record observations from ROS topics
        for topic in self.topics:
            msg = rospy.wait_for_message(topic, rospy.AnyMsg)  # Replace with specific msg type if known
            data = np.frombuffer(msg._buff, dtype=np.uint8) if hasattr(msg, "_buff") else np.array(msg.data)
            self.obs[topic].append(data)

        # Record action
        self.actions.append(action)

        # Record dummy state (replace with real robot state gathering)
        # self.states.append(np.random.rand(47))  # Example

    def pause_resume_recording(self):
        if self.recording_started:
            if self.recording_paused:
                self.recording_paused = False
                rospy.loginfo("Recording resumed")
            else:
                self.recording_paused = True
                rospy.loginfo("Recording paused")
            self.pause_resume_times.append(rospy.get_time())
        else:
            rospy.logwarn("Recording not yet started - nothing to be done")

    def stop_recording(self):
        if self.recording_stopped:
            rospy.logwarn("Recording has already Stopped - nothing to be done")
            return

        if self.recording_paused:
            self.pause_resume_recording()

        if self.pause_resume_times:
            pause_resume_str = list(map(str, self.pause_resume_times))
            pause_resume_str[0:0] = ['PAUSE', 'RESUME']
            rospy.logwarn("List of pause and resume times:\n%s\n", format_to_columns(pause_resume_str, 2))

        # Write datasets to HDF5 in robomimic format
        demo_group = self.h5.create_group("data/demo_0")
        demo_group.create_dataset("actions", data=np.array(self.actions))
        obs_group = demo_group.create_group("obs")
        for topic, values in self.obs.items():
            obs_group.create_dataset(topic.replace('/', '_'), data=np.array(values))
        # demo_group.create_dataset("states", data=np.array(self.states))

        self.h5.close()
        rospy.loginfo(f"Saved the hdf5 file in robomimic style format at {self.hdf5_file}")

        self.recording_started = False
        self.recording_stopped = True
