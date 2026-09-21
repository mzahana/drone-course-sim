cmake_minimum_required(VERSION 3.8)
project(lab4_perception)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/target_locator.py
  scripts/gimbal_pointer.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
