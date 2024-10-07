# Franka Teleoperation

You will need ROS1 (tested on ROS Noetic), `libfranka`,`franka_ros`, [Franka Interactive Controllers](https://github.com/nbfigueroa/franka_interactive_controllers/tree/main) and [`franka_zed_gazebo`](https://github.com/pearl-robot-lab/franka_zed_gazebo).

## 0. Docker image
You can get the current docker image of the environment being used for tests. You can follow the instructions on how to download docker and set up an alias for running a container with the required arguments in this repo: [Docker_env](https://github.com/pearl-robot-lab/Docker_env). Pull a docker image and create a container:
```
docker pull sophiamoyen/franka_teleop:07102024
```
```
docker_run_nvidia --name="franka_teleop" sophiamoyen/franka_teleop:07102024 bash
```
You should now be inside the container. To exit it, just type `exit` in the cmd. From now on, every time you want to enter the container, start it then execute it:
```
docker start franka_teleop
```
```
docker exec -it franka_teleop bash
```

## 1. Start Franka controllers
### Simulation
If you haven't already, launch the simulation with the controllers. You need to have the package `franka_zed_gazebo` to launch in simulation (the launch file is getting the world and the robot description from that package, if you want to launch the robot wihtout the zed camera, then just edit the robot description)


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

### To do still:
- Include max/min workspace for the xyz values
- Add the teleop for rotation
- Map the buttons for the controller to start teleop, stop teleop, reposition, open gripper, close gripper
- Tune impedance controller considering external tools (camera)
- Implement opening and closing of gripper