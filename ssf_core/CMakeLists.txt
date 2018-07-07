cmake_minimum_required(VERSION 2.8.3)
project(ssf_core)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy 
sensor_msgs 
dynamic_reconfigure 
cmake_modules 
sensor_fusion_comm 
std_msgs 
message_generation 
tf2 
tf2_ros
message_filters
)

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

# Generate dynamic parameters

add_message_files(FILES visensor_imu.msg)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

add_definitions (-Wall -O3)

# get eigen
find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(cfg/SSF_Core.cfg)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure sensor_fusion_comm message_runtime tf2 tf2_ros
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
    LIBRARIES ssf_core
)

add_library(ssf_core src/SSF_Core.cpp src/measurement.cpp src/state.cpp)
add_dependencies(ssf_core ${PROJECT_NAME}_gencfg ssf_core_generate_messages_cpp)
target_link_libraries(ssf_core ${catkin_LIBRRIES})

