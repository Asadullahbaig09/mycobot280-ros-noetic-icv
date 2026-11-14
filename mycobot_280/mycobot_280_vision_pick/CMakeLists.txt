cmake_minimum_required(VERSION 3.0.2)
project(mycobot_280_vision_pick)

find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros std_msgs)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# World plugin that welds a grasped cube to the gripper (see src/gripper_attach_plugin.cpp)
add_library(gripper_attach_plugin src/gripper_attach_plugin.cpp)
target_link_libraries(gripper_attach_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Plain install (not catkin_install_python): its devel-space wrapper breaks the
# sibling import of vision_geometry.py. The scripts already have python3 shebangs.
install(PROGRAMS scripts/cube_detector.py scripts/pick_place.py scripts/vision_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch urdf worlds config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS gripper_attach_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
