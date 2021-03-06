cmake_minimum_required(VERSION 2.8.3)
project(ssf_updates)

find_package(catkin REQUIRED COMPONENTS roscpp ssf_core geometry_msgs message_generation)
find_package (Eigen3 REQUIRED)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions (-Wall -O3)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_message_files(FILES PositionWithCovarianceStamped.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS roscpp ssf_core geometry_msgs)

# add_executable(pose_sensor src/main.cpp src/pose_sensor.cpp)
# set_property(TARGET pose_sensor PROPERTY COMPILE_DEFINITIONS POSE_MEAS)

# target_link_libraries(pose_sensor ${catkin_LIBRARIES})

# add_executable(position_sensor src/main.cpp src/position_sensor.cpp)
# set_property(TARGET position_sensor PROPERTY COMPILE_DEFINITIONS POSITION_MEAS)

# target_link_libraries(position_sensor ${catkin_LIBRARIES})


# add_executable(viconpose_sensor src/main.cpp src/viconpose_sensor.cpp)
# set_property(TARGET viconpose_sensor PROPERTY COMPILE_DEFINITIONS VICONPOSE_MEAS)

# set_target_properties(viconpose_sensor PROPERTIES COMPILE_FLAGS "-O3")
# target_link_libraries(viconpose_sensor ${catkin_LIBRARIES})

add_executable(visionpose_sensor src/main.cpp src/visionpose_sensor.cpp)
set_property(TARGET visionpose_sensor PROPERTY COMPILE_DEFINITIONS VISIONPOSE_MEAS)

set_target_properties(visionpose_sensor PROPERTIES COMPILE_FLAGS "-O3")
target_link_libraries(visionpose_sensor ${catkin_LIBRARIES})

#add_dependencies(visionpose_sensor ${PROJECT_NAME}_gencpp)
add_dependencies(visionpose_sensor ${catkin_EXPORTED_TARGETS}) # this is cleaner?