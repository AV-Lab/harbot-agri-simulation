cmake_minimum_required(VERSION 2.8.3)
project(realsense_d435_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    sensor_msgs
    image_transport
    camera_info_manager
)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        realsense_gazebo_plugin
        realsense_gazebo_accel_sensor
        realsense_gazebo_gyro_sensor
    CATKIN_DEPENDS
        roscpp
        gazebo_ros
        sensor_msgs
        image_transport
        camera_info_manager
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(realsense_gazebo_plugin
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
)
target_link_libraries(realsense_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(realsense_gazebo_plugin ${catkin_EXPORTED_TARGETS})

add_library(realsense_gazebo_accel_sensor src/gazebo_ros_accel_sensor.cpp)
target_link_libraries(realsense_gazebo_accel_sensor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(realsense_gazebo_accel_sensor ${catkin_EXPORTED_TARGETS})

add_library(realsense_gazebo_gyro_sensor src/gazebo_ros_gyro_sensor.cpp)
target_link_libraries(realsense_gazebo_gyro_sensor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(realsense_gazebo_gyro_sensor ${catkin_EXPORTED_TARGETS})

install(
    TARGETS
        realsense_gazebo_plugin
        realsense_gazebo_accel_sensor
        realsense_gazebo_gyro_sensor
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/realsense_gazebo_plugin/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/realsense_gazebo_plugin
)

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
