
conda create -n single_franka_ws python=3.11
conda activate single_franka_ws

conda install -c conda-forge -c robostack-noetic ros-noetic-desktop

conda install conda-forge::pinocchio

conda install -c conda-forge -c robostack-noetic \
  ros-noetic-ros-control \
  ros-noetic-controller-interface \
  ros-noetic-controller-manager \
  ros-noetic-ros-controllers \
  ros-noetic-realtime-tools \
  ros-noetic-joint-limits-interface \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-control \
  ros-noetic-moveit-simple-controller-manager \
  ros-noetic-rviz-imu-plugin \
  ros-noetic-depthimage-to-laserscan \
  ros-noetic-rtabmap-ros \
  ros-noetic-image-transport-plugins 


export CONDA_PREFIX=$(conda info --base)/envs/single_franka_ws
export CMAKE_PREFIX_PATH=$CONDA_PREFIX:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
export CMAKE_EXE_LINKER_FLAGS="-Wl,-rpath,$CONDA_PREFIX/lib -Wl,-rpath-link,$CONDA_PREFIX/lib"
export CMAKE_SHARED_LINKER_FLAGS="$CMAKE_EXE_LINKER_FLAGS"



<!-- conda install -c conda-forge libfranka  -->

## Install libfranka
mkdir -p ~/lib/
cd ~/lib
git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git
cd libfranka
git checkout 0.15.0
git submodule update

### Build libfranka


Install dependecies:
```
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev
```

Then: 

```
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_INSTALL_PREFIX=/home/ansh/lib/libfranka/build \
      ..
make -j$(nproc)
```


mkdir -p ~/single_franka_ws/src
cd ~/single_franka_ws/src

git clone --recursive https://github.com/AnshPrakash/franka_teleop.git

git clone -b sophia_dev https://github.com/sophiamoyen/franka_interactive_controllers.git

git clone -b develop https://github.com/frankaemika/franka_ros


git clone https://github.com/nbfigueroa/rosbag_to_mat.git

git clone -b latest-franka https://github.com/nbfigueroa/easy-kinesthetic-recording

git clone https://github.com/stereolabs/zed-ros-examples.git

git clone https://github.com/stereolabs/zed-ros-wrapper.git

git clone -b noetic-devel https://github.com/moveit/panda_moveit_config.git


git clone https://github.com/iROSA-lab/franka_zed_gazebo.git

git clone https://github.com/stereolabs/zed-ros-interfaces.git

git clone https://github.com/PickNikRobotics/boost_sml.git



cd franka_teleop/droid
pip install -e .

cd ..


python -m pip install cython numpy opencv-python pyopengl

cd ~/single_franka_ws


catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DFranka_DIR=/home/ansh/lib/libfranka/build/

catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DFranka_DIR=<your-libfranka-installation>libfranka/build/


catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DFranka_DIR=/home/ansh/lib/libfranka/build/  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"

source devel/setup.bash

