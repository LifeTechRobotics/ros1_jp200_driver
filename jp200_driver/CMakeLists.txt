cmake_minimum_required(VERSION 3.0.2)
project(jp200_driver)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Cmd.msg
  Gain.msg
  JP200.msg
  JP200MultiArray.msg
  JP200Response.msg
  JP200Responses.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(jp200_utils src/utils.cpp)

add_executable(jp200_driver src/main.cpp)
add_dependencies(jp200_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(jp200_driver
  jp200_utils
  ${catkin_LIBRARIES}
)

add_executable(example_publisher src/example_publisher.cpp)
add_dependencies(example_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_publisher
  jp200_utils
  ${catkin_LIBRARIES}
)