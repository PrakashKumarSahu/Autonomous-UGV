cmake_minimum_required(VERSION 3.8)
project(ugv_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds models scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install relay script as a runnable node executable
install(
  PROGRAMS scripts/joint_state_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
