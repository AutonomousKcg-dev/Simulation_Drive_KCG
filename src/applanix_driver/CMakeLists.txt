cmake_minimum_required(VERSION 3.5)
project(applanix_driver)

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
		src/read_gsof/socket_node.cpp
		src/read_gsof/rx.cpp
		include/read_gsof/rx.hpp

		#src/parse_gsof/gsof_parser_node.cpp
		#src/parse_gsof/gsof_parser.cpp
		#src/parse_gsof/geo_pos_conv.cpp
		include/parse_gsof/geo_pos_conv.hpp
		include/parse_gsof/gsofParser.hpp
)


target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-useless-cast -Wno-double-promotion -Wno-old-style-cast -Wno-sign-conversion -Wno-unused-variable -Wno-unused-parameter -Wno-format -Wno-unused-value -Wno-unused-but-set-variable) 

#rclcpp_components_register_node(${PROJECT_NAME}
#  PLUGIN "GeneralController"
#  EXECUTABLE ${PROJECT_NAME}_exe
#)

#set(GENERAL_CONTROLLER_NODE pure_pursuit_integration_test_exe)
ament_auto_add_executable(read_gsof_exe 
		src/read_gsof/socket_node.cpp
		src/read_gsof/rx.cpp
		include/read_gsof/rx.hpp
)
ament_target_dependencies(read_gsof_exe rclcpp std_msgs autoware_auto_msgs  tf2_geometry_msgs)

ament_auto_add_executable(parse_gsof_exe 
		src/parse_gsof/gsof_parser_node.cpp
		src/parse_gsof/gsof_parser.cpp
		src/parse_gsof/geo_pos_conv.cpp
		include/parse_gsof/geo_pos_conv.hpp
		include/parse_gsof/gsofParser.hpp
)
#add_executable(general_controller src/general_controller_node.cpp)
ament_target_dependencies(read_gsof_exe rclcpp std_msgs autoware_auto_msgs tf2_geometry_msgs )





ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
