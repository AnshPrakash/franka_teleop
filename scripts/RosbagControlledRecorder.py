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

import psutil
import subprocess
import shlex
import signal
from __future__ import annotations

import rospy
from std_srvs.srv import Empty, EmptyResponse
from datetime import datetime
import os


def signal_process_and_children(pid, signal_to_send, wait=False):
    process = psutil.Process(pid)
    for children in process.children(recursive=True):
        if signal_to_send == 'suspend':
            children.suspend()
        elif signal_to_send == 'resume':
            children.resume()
        else:
            children.send_signal(signal_to_send)
    if wait:
        process.wait()


def format_to_columns(input_list, cols):
    """Adapted from https://stackoverflow.com/questions/171662/formatting-a-list-of-text-into-columns"""
    max_width = max(map(len, input_list))
    justify_list = map(lambda x: x.ljust(max_width + 4), input_list)
    lines = (''.join(justify_list[i:i + cols]) for i in range(0, len(justify_list), cols))
    return '\n'.join(lines)


class RosbagControlledRecorder(object):
    """Record a rosbag with service calls to control start, stop  and pause"""

    def __init__(self, save_folder, topics, is_video=False, complementary_recorder: RosbagControlledRecorder=None):
        """
            All topics should be either video topics or all should be non-video topics
        """
        os.makedirs(save_folder, exist_ok=True)
        self.save_folder = save_folder
        self.topics = topics
        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []
        self.process_pids = None
        self.commands = None
        self.is_video = is_video
        self.complementary_recorder = complementary_recorder # so current recorder of different types can use the same folder as this recorder
        self.data_folder:str = None 
        
        
        

    def start_recording(self):
        if self.recording_started:
            rospy.logwarn("Recording has already started - nothing to be done")
        else:
            self.commands, self.data_folder = self._generate_command()
            processes = [subprocess.Popen(command,cwd=cwd) for command, cwd in self.commands]
            self.process_pids = [ process.pid for process in processes ]
            self.recording_started = True
            self.recording_stopped = False
            rospy.loginfo("Started recording rosbag")
            
    def _generate_command(self):
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        prefix = f"data-{current_time}.bag"
        if self.complementary_recorder and not self.complementary_recorder.data_folder:
            raise Exception("Start recording of the complementary recording first")
        data_folder = (
            self.complementary_recorder.data_folder
            if self.complementary_recorder
            else os.path.join(self.save_folder, f"data-{current_time}") )
        

        os.makedirs(data_folder, exist_ok=True)
        if self.is_video:
            # For each video topic create a video sub-folder
            # required for storing imtermediate frames
            commands = []
            for topic_i in range(len(self.topics)):
                intermediate_folder = os.path.join(data_folder,f"tmp_{topic_i.replace('/','_')}")
                os.makedirs( intermediate_folder, exist_ok=True)
                commands.append( [
                    shlex.split(f"rosrun image_view extract_images image:={topic_i} _image_transport:=raw"),
                    intermediate_folder] )

        else:
            output_path = os.path.join(data_folder, prefix)
            commands = [shlex.split(f"rosbag record -o {output_path} {' '.join(self.topics)}"), None]
        return commands, data_folder
        
    def pause_resume_recording(self):
        if self.recording_started:
            if self.recording_paused:
                for process_pid in self.process_pids:
                    signal_process_and_children(process_pid, 'resume')
                self.recording_paused = False
                rospy.loginfo("Recording resumed")
            else:
                for process_pid in self.process_pids:
                    signal_process_and_children(process_pid, 'suspend')
                self.recording_paused = True
                rospy.loginfo("Recording paused")
            self.pause_resume_times.append(rospy.get_time())
        else:
            rospy.logwarn("Recording not yet started - nothing to be done")

    def stop_recording(self):
        if self.recording_stopped:
            rospy.logwarn("Recording has already Stopped - nothing to be done")
            return
        if self.process_pid is not None:
            if self.recording_paused:  # need to resume process in order to cleanly kill it
                self.pause_resume_recording()
            if self.pause_resume_times:  # log pause/resume times for user's reference
                pause_resume_str = map(str, self.pause_resume_times)
                pause_resume_str[0:0] = ['PAUSE', 'RESUME']
                rospy.logwarn("List of pause and resume times:\n%s\n", format_to_columns(pause_resume_str, 2))
            for process_pid in self.process_pids:
                signal_process_and_children(process_pid, signal.SIGINT, wait=True)
            self.process_pids = None
            
            if self.is_video:
                video_processes = []
                for topic_i in range(len(self.topics)):
                    # generate a final video file and delete the imtermediate folder containing all frames for each topic
                    folder_path = os.path.join(self.data_folder,f"""tmp_{topic_i.replace("/","_")}""")
                    frame_rate = 15 # In Hz
                    convert_to_video_command = shlex.split(f"ffmpeg -r {frame_rate} -i frame%06d.jpg -c:v libx264 {topic_i.replace('/','_')}.mp4")
                    video_processes.append(subprocess.Popen(convert_to_video_command))
                
                for process in video_processes:
                    exit_code = process.wait()
                    if exit_code != 0:
                        rospy.logerr(f"Unable to process the recording failed with exit code {exit_code}")
                    
                for topic_i in range(len(self.topics)):
                    if os.path.exists(folder_path) and os.path.isdir(folder_path):
                        os.rmdir(folder_path)
            rospy.loginfo("Stopped recording rosbag")
        self.recording_started = False
        self.recording_stopped = True


if __name__ == '__main__':
    rospy.init_node('rosbag_controlled_recording')

    # Get parameters
    rosbag_command = rospy.get_param('~rosbag_command','rosbag record -o /home/data_collection/data/ /rosout')  # str with rosbag command line command to be issued
    record_from_startup = rospy.get_param('~record_from_startup', False)  # whether to start node already recording

    # Start recorder object
    recorder = RosbagControlledRecorder(rosbag_command, record_from_startup)

    

    # Recording is also stopped on node shutdown. This allows stopping to be done via service call or regular Ctrl-C
    rospy.on_shutdown(recorder.stop_recording_srv)

    while not rospy.is_shutdown():
        #if recorder.recording_stopped:  # stop main node if recording has finished
            #break
        rospy.sleep(1.0)