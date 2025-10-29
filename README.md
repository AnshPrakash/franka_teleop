# Franka Teleoperation

This repo provides a script for teleoperating the Franka Panda robot using the VR setup from VIVE. It is necessary to set up the [`vive_ros`](https://github.com/robosavvy/vive_ros) streamer repo that connects to STEAM VR to stream the data from the controllers to ROS. Hopefully this repo can help in robotic manipulation research and dataset creation.

## Requirements
You will need ROS1 (tested on ROS Noetic), `libfranka`,`franka_ros`, [Franka Interactive Controllers](https://github.com/sophiamoyen/franka_interactive_controllers) and [`franka_zed_gazebo`](https://github.com/pearl-robot-lab/franka_zed_gazebo). The `pose_impedance_control_additional_params.yaml` contains tunable parameters for external tool compensation and nullspace stiffness. When the `cartesian_pose_impedance_controller` is launched in the teleoperation, it looks up that file that should be tuned accordingly (with the ZED2 camera attached to the wrist, for example).


## 0. Docker image
You can get the current docker image of the environment being used for tests. You can follow the instructions on how to download docker and set up an alias for running a container with the required arguments in this repo: [Docker_env](https://github.com/pearl-robot-lab/Docker_env). Pull a docker image and create a container:
```
docker push levoz/franka_teleop:19082025
```
```
docker_run_nvidia --name="franka_teleop" -v <data_collection_folder>:/opt/ros_ws/src/data_collection levoz/franka_teleop:29072025 bash
```
You should now be inside the container. To exit it, just type `exit` in the cmd. From now on, every time you want to enter the container, start it then execute it:
```
docker start franka_teleop
```
```
docker exec -it franka_teleop bash
git remote set-url origin https://github.com/AnshPrakash/franka_teleop.git
git pull --recurse-submodules
```


## Update Robot IP 

Update the robot IP in the bashrc


### Build & Source

```
cd /opt/ros_ws
catkin_make -DFranka_DIR=/opt/libfranka/build/
source devel/setup.bash
```

### Troubleshooting

UDP timeout error

```
sudo iptables -I INPUT 1 -s <robot-ip> -j ACCEPT
```

## Data Collection

### Step 1:  Record Rosbag


During teleoperation data will get recorded under `data-collection` folder. Since this is mounted it can be accessed outside of docker change the permission of all the data from your system(outside docker) `chmod -R 777 *`.

`Adjust recording Config for your usecase`:

update the following yaml => `/opt/ros_ws/src/franka_teleop/config/recorder.yaml`

```
# List of topics to be recored along with the type

save_folder: <location of data> keep in mind this is within the docker container, so don't forgot to mount it

topics:
  - <topic name1> : <type1>
  - <topic_name2>: <type2>
  ...
  ...

video_topics: If you want to store it as a video
  - ..
```

> Note! `Step 2 and Step 3 can be done outside docker`

## Step 2: Resample

For feeding the data into a learning alogrithm the observations needs to be downsampled to a desired frequency(hyper-parameter), and all of them to have same length. 

As of now, we only do down-sampling so maximum frquency is limited by the lowest frequency observation.(Analysis of frequencies of various topics can also be easily done with the provide code [Check Visualisation section Readme for Sampler])

Use the following [Sampler](https://github.com/AnshPrakash/MimicPlay/tree/main/sampler)

Output is again in Rosbag format, but data is downsampled to a desired frequency.

## Step 3: RLDS or robomimic format


Most of the learning alogorithms expect data to be in certain format. 
Now we convert the Rosbag format(After Step2) to either `RLDS`, or `Robomimic` format.

Follow the steps in the following module: [Rosbag2hdf5](https://github.com/AnshPrakash/MimicPlay/blob/main/rosbag2hdf5/README.md)


<p align="center">
  <img src="images/data-processing-pipeline.png" width="500"/>
</p>




## Teleoperation with Quest


## 1. Start Franka controllers
### Simulation
If you haven't already, launch the simulation with the controllers. You need to have the package `franka_zed_gazebo` to launch in simulation (the launch file is getting the world and the robot description from that package)

With the ZED2 attached to the end-effector:

```
roslaunch franka_teleop franka_interactive_teleop_simulation.launch
```

Without camera:
```
roslaunch franka_teleop franka_interactive_teleop_wo_camera_simulation.launch
```

### Real World

To launch in the real world (not yet teste - not yet recommended), unlock the joints in the desk application, active FCI and put it in Execution mode. Then launch the controllers:
```
roslaunch franka_teleop franka_interactive_teleop_real.launch
```

## 2. Stream VR controllers
In Kasos PC (second Windows partition), open Steam, start Steam VR. If needed, change room setup in the dropdown menu. Choose Standing Only option and follow instructions. Then open a terminal (not windows powershell), on top of the terminal, there is an option to open a terminal with ROS Noetic connected to the Aegina PC. Make sure the controllers are turned on and are being detected. Then start streaming (make sure there is already a ros master to connect to):

```
roslaunch vive_launcher vive.launch
```

## 3. Start teleop script
Inside the scripts folder, run the python file:
```
python3 teleop.py
```

The robot will first open the gripper and go to standard pose using the `effort_joint_trajectory_controller`, originally `franka_ros`. Then it will switch to the `cartesian_pose_impedance_controller`. To start/stop the teleop, press the Home button in the VR controller (`1` in the drawing below). To go back to standard pose, press the pad button (`2` in the drawing). To open or close the gripper, press the trigger (`7` in the drawing).

<p align="center">
  <img src="images/controller.png" width="500"/>
</p>

### To do still:
- Include max/min workspace for the xyz values
- Tune impedance controller considering external tools (camera)
- Implement a better way to close and open gripper
- Check out if there is a better way to implement the rotation of the gripper
- Not in here, but in the recording node read button state for starting/stopping recording