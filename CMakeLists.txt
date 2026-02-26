cmake_minimum_required(VERSION 3.8)
project(my_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# This section "installs" your folders so ROS can find your files
install(
  DIRECTORY
    config
    description
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()