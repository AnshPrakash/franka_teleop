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

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from datetime import datetime
import psutil
import subprocess
import shlex
import signal
import shutil
import os
import time


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
    lines = (''.join(list(justify_list)[i:i + cols]) for i in range(0, len(list(justify_list)), cols))
    return '\n'.join(lines)


class RosbagControlledRecorder(Node):
    """Record a rosbag with service calls to control start, stop  and pause"""
    def __init__(self, save_folder, topics, is_video=False, complementary_recorder: RosbagControlledRecorder=None, debug=False):
        """
            All topics should be either video topics or all should be non-video topics
        """
        super().__init__('rosbag_controlled_recording')
        os.makedirs(save_folder, exist_ok=True)
        self.save_folder = save_folder
        self.topics = topics
        self.is_video = is_video
        self.complementary_recorder = complementary_recorder # so current recorder of different types can use the same folder as this recorder

        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []
        self.process_pids = None
        self.commands = None
        self.data_folder = None
        self.debug = debug
        if not self.topics:
            raise ValueError("No topics provided for recording.")

        # Services for start/pause/resume/stop
        self.srv_start = self.create_service(Empty, 'start', self.start_service_callback)
        self.srv_pause_resume = self.create_service(Empty, 'pause_resume', self.pause_resume_service_callback)
        self.srv_stop = self.create_service(Empty, 'stop', self.stop_service_callback)

    # ---------------- Service callbacks ------------------
    def start_service_callback(self, request, response):
        self.start_recording()
        return response

    def pause_resume_service_callback(self, request, response):
        self.pause_resume_recording()
        return response

    def stop_service_callback(self, request, response):
        self.stop_recording()
        return response

    # ---------------- Recording logic ------------------
    def start_recording(self):
        if self.recording_started:
            self.get_logger().warn("Recording has already started - nothing to be done")
        else:
            self.commands, self.data_folder = self._generate_command()
            processes = [subprocess.Popen(command, cwd=cwd, preexec_fn=os.setsid) for command, cwd in self.commands]
            self.process_pids = [p.pid for p in processes]
            self.recording_started = True
            self.recording_stopped = False
            self.get_logger().info("Started recording rosbag")

    def _generate_command(self):
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        if self.complementary_recorder and not self.complementary_recorder.data_folder:
            raise Exception("Start recording of the complementary recording first")

        data_folder = self.complementary_recorder.data_folder if self.complementary_recorder else os.path.join(self.save_folder, f"data-{current_time}")
        os.makedirs(data_folder, exist_ok=True)

        if self.is_video:
            # For each video topic create a video sub-folder
            # required for storing imtermediate frames
            commands = []
            for topic_i in self.topics:
                intermediate_folder = os.path.join(data_folder, f"tmp_{topic_i.replace('/', '_')}")
                os.makedirs(intermediate_folder, exist_ok=True)
                # Only for debugging purposes to check if recording is working. Don't rely on the saved videos. Only use rosbags for data.
                log_level = "--log-level error" if not self.debug else ""
                commands.append( (
                    shlex.split(f"ros2 run image_view extract_images --ros-args {log_level} -r image:={topic_i} -p filename_format:=frame%04d.jpg"),
                    intermediate_folder) )
        else:
            prefix = f"data-{current_time}.bag"
            output_path = os.path.join(data_folder, prefix)
            commands = [(shlex.split(f"ros2 bag record -o {output_path} {' '.join(self.topics)}"), None)]
        return commands, data_folder

    def pause_resume_recording(self):
        if not self.recording_started:
            self.get_logger().warn("Recording not yet started - nothing to be done")
            return
        if self.recording_paused:
            for pid in self.process_pids:
                signal_process_and_children(pid, 'resume')
            self.recording_paused = False
            self.get_logger().info("Recording resumed")
        else:
            for pid in self.process_pids:
                signal_process_and_children(pid, 'suspend')
            self.recording_paused = True
            self.get_logger().info("Recording paused")
        self.pause_resume_times.append(self.get_clock().now().nanoseconds / 1e9)

    def stop_recording(self):
        if self.recording_stopped:
            self.get_logger().warn("Recording has already Stopped - nothing to be done")
            return

        if self.process_pids is not None:
            if self.recording_paused:
                self.pause_resume_recording()
            if self.pause_resume_times:
                pause_resume_str = map(str, self.pause_resume_times)
                pause_resume_str = ['PAUSE', 'RESUME'] + list(pause_resume_str)
                self.get_logger().warn("List of pause and resume times:\n%s", format_to_columns(pause_resume_str, 2))

            for pid in self.process_pids:
                signal_process_and_children(pid, signal.SIGINT, wait=True)
            self.process_pids = None

            if self.is_video:
                video_processes = []
                # generate a final video files and delete the imtermediate folders containing all frames for each topic
                for topic_i in self.topics:
                    folder_path = os.path.join(self.data_folder, f"tmp_{topic_i.replace('/', '_')}")
                    frame_rate = 15
                    output_file = os.path.join(self.data_folder, f"{topic_i.replace('/', '_').strip('_')}.mp4")
                    convert_to_video_command = shlex.split(f"ffmpeg -r {frame_rate} -i frame%04d.jpg -c:v libx264 {output_file}")
                    video_processes.append(subprocess.Popen(convert_to_video_command, cwd=folder_path))

                for process in video_processes:
                    exit_code = process.wait()
                    if exit_code != 0:
                        self.get_logger().error(f"Video processing failed with exit code {exit_code}")

                for topic_i in self.topics:
                    folder_path = os.path.join(self.data_folder, f"tmp_{topic_i.replace('/', '_')}")
                    if os.path.exists(folder_path) and os.path.isdir(folder_path):
                        try:
                            shutil.rmtree(folder_path)
                        except PermissionError:
                            self.get_logger().error("Permission denied")
                        except Exception as e:
                            self.get_logger().error(f"Error: {e}")
                self.get_logger().info(f"Videos saved at {self.data_folder}")

        self.recording_started = False
        self.recording_stopped = True


# ---------------- Main -----------------
def main(args=None):
    # Example usage
    # Run  "ros2 launch zed_wrapper zed_camera.launch.py  camera_model:=zed2" in another terminal first to have ZED camera topics available
    rclpy.init(args=args)

    save_folder = '/home/ansh/Teleop_Panda_system/data_collection/data'
    # topics = ['/rosout']  # default topics, can be extended
    topics = ['/zed/zed_node/stereo_raw/image_raw_color']  # default topics, can be extended
    

    # recorder = RosbagControlledRecorder(save_folder, topics)
    recorder = RosbagControlledRecorder(save_folder, topics, is_video=True)
    recorder.start_recording()

    # recorder.on_shutdown(recorder.stop_recording)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.stop_recording()
    finally:
        # rclpy.shutdown()
        pass


if __name__ == '__main__':
    main()