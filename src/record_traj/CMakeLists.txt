cmake_minimum_required(VERSION 3.5)
project(record_traj)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
		src/${PROJECT_NAME}_node.cpp

)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-useless-cast -Wno-double-promotion -Wno-old-style-cast -Wno-sign-conversion -Wno-unused-variable -Wno-unused-parameter -Wno-conversion)

ament_auto_add_executable(${PROJECT_NAME}_exe 
		src/${PROJECT_NAME}_node.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  param
)
