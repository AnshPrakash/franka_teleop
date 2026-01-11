cmake_minimum_required(VERSION 3.8)
project(franka_teleop)

# Required for Python packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install executable scripts (if any are outside the module)
# install(
#   PROGRAMS
#     franka_teleop/teleop_quest.py
#     franka_teleop/oculus_transforms_vrpolicy.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
