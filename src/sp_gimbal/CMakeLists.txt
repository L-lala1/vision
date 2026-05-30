cmake_minimum_required(VERSION 3.10)
project(sp_gimbal)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# serial 库的头文件路径（在 src/serial/include 下面）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/serial/include)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gimbal_node.cpp
  src/crc.cpp
  src/serial/src/serial.cc
  src/serial/src/impl/unix.cc
  src/serial/src/impl/list_ports/list_ports_linux.cc
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN sp_gimbal::GimbalNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
)
