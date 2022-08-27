cmake_minimum_required(VERSION 2.8.3)
project(panda_simulation)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
rosbag
message_generation
sensor_msgs
tf2_msgs
pcl_ros
)
find_package(PCL REQUIRED)

# generate_messages(
#    DEPENDENCIES
#    std_msgs
#    geometry_msgs
#    sensor_msgs
# )

catkin_package(
)


include_directories(
# include
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
   scripts/move_franka.py
   
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcl_processor_node src/pcl_processing.cpp)
target_link_libraries(pcl_processor_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

