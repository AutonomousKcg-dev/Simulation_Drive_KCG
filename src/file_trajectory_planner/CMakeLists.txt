cmake_minimum_required(VERSION 3.5)
project(file_trajectory_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#add_compile_options(-Wno-error)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
		include/file_trajectory_planner/file_trajectory_planner.hpp
		src/file_trajectory_planner_node.cpp
		src/file_trajectory_planner.cpp
		
)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-useless-cast -Wno-double-promotion -Wno-old-style-cast -Wno-sign-conversion -Wno-unused-variable -Wno-unused-parameter  -Wno-float-conversion -Wno-unused-but-set-variable)


#rclcpp_components_register_node(${PROJECT_NAME}
#  PLUGIN "GeneralController"
#  EXECUTABLE ${PROJECT_NAME}_exe
#)

#set(GENERAL_CONTROLLER_NODE pure_pursuit_integration_test_exe)
ament_auto_add_executable(${PROJECT_NAME}_exe 
		src/file_trajectory_planner_node.cpp
src/file_trajectory_planner.cpp
include/file_trajectory_planner/file_trajectory_planner.hpp

)
#add_executable(general_controller src/general_controller_node.cpp)
#$ament_target_dependencies(general_controller rclcpp std_msgs autoware_auto_msgs)





ament_auto_package(
  INSTALL_TO_SHARE
  config
  param
  launch
)
