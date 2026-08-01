cmake_minimum_required(VERSION 3.8)
project(srmauv_description)

find_package(ament_cmake REQUIRED)

# Install resource folders so launch files can locate them at runtime.
install(
  DIRECTORY urdf meshes launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Install ROS 2 Python nodes as executables (ros2 run srmauv_description <node>).
install(PROGRAMS
  nodes/thruster_controller.py
  nodes/teleop_keyboard.py
  nodes/autopilot.py
  nodes/depth_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
