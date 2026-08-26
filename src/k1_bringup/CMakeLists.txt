cmake_minimum_required(VERSION 3.10)
project(k1_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Python node, installed without the .py suffix so launch files can refer to it
# as executable='cmd_vel_relay'
install(PROGRAMS
  scripts/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_relay
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
