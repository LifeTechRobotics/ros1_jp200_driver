cmake_minimum_required(VERSION 3.0.2)
project(jp200_driver)


find_package(catkin REQUIRED COMPONENTS
  jp200_msgs
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(jp200_driver src/main.cpp)

target_link_libraries(jp200_driver
  ${catkin_LIBRARIES}
)