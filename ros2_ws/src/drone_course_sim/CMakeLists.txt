cmake_minimum_required(VERSION 3.8)
project(drone_course_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch urdf rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/verify_camera_frame.py
  scripts/gimbal_interface_node.py
  scripts/vehicle_tf_node.py
  scripts/detector_node.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
