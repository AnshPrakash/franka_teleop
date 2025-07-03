

# Create a ROS1 workspace


mkdir -p <workspace-name>/src
cd src
git clone --recurse-submodules https://github.com/AnshPrakash/franka_teleop.git
git clone --recurse-submodules https://github.com/frankarobotics/franka_ros.git
cd franka_ros
git checkout 0.10.0
cd ..

git clone --recurse-submodules https://github.com/sophiamoyen/franka_interactive_controllers.git
git clone --recurse-submodules https://github.com/pearl-robot-lab/franka_zed_gazebo.git
git clone --recurse-submodules https://github.com/moveit/panda_moveit_config.git


# Install libfranka

```
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev

git clone --recurse-submodules https://github.com/frankaemika/libfranka.git
cd libfranka
git checkout 0.10.0
git submodule update
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
make

cpack -G DEB
sudo dpkg -i libfranka*.deb
cd ../../
```

## Before building the Workspace


rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka


