cmake_minimum_required(VERSION 2.8.3)
project(visual_servo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  rospy
  baxter_core_msgs
)

find_package(VISP REQUIRED)

if(VISP_FOUND)
    include(${VISP_USE_FILE})
endif(VISP_FOUND)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(baxter-video-listener src/baxter-video-listener.cpp)
target_link_libraries(baxter-video-listener ${catkin_LIBRARIES})
