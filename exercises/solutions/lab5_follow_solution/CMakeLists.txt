cmake_minimum_required(VERSION 3.8)
project(lab5_follow_solution)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/target_tracker.py
  scripts/follow_guidance.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
