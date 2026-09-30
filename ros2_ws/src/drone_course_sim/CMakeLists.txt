cmake_minimum_required(VERSION 3.8)
project(drone_course_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch urdf rviz DESTINATION share/${PROJECT_NAME})

# The meshes RViz draws the aircraft with. They are the Gazebo models' own
# files, installed from wherever the models are rather than copied into this
# package, so git holds one copy. In a repo checkout that is ../../../models;
# the image copies only this package into its workspace, so the Dockerfile
# points COURSE_MODELS_DIR at the models it installed into the PX4 tree.
# Missing meshes fail the build: an RViz with no aircraft is a silent failure.
set(COURSE_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../models"
    CACHE PATH "Directory holding the course_* Gazebo models")
foreach(model course_x500_base course_gimbal)
  if(NOT EXISTS "${COURSE_MODELS_DIR}/${model}/meshes")
    message(FATAL_ERROR "no meshes at ${COURSE_MODELS_DIR}/${model}/meshes; "
                        "set -DCOURSE_MODELS_DIR to the course models directory")
  endif()
  install(DIRECTORY "${COURSE_MODELS_DIR}/${model}/meshes/"
          DESTINATION share/${PROJECT_NAME}/meshes/${model})
endforeach()
install(PROGRAMS
  scripts/verify_camera_frame.py
  scripts/gimbal_interface_node.py
  scripts/vehicle_tf_node.py
  scripts/detector_node.py
  scripts/target_route_node.py
  scripts/scenario_test.py
  scripts/scoring_node.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
