cmake_minimum_required(VERSION 3.8)
project(misora2_operation)

# Default to C++20
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_definitions(USE_TINYXML2)
  add_compile_definitions(RCLCPP_USING_CUSTOM_TYPE_ADAPTERS)

endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

include_directories(
  include
)

set(dependencies
    rclcpp
    rclcpp_components
    cv_bridge
    OpenCV
    sensor_msgs
    std_msgs
)

add_library(operator_gui_library SHARED src/operator_gui_component.cpp src/gui_tool.cpp)
ament_target_dependencies(operator_gui_library ${dependencies})
rclcpp_components_register_nodes(operator_gui_library "component_operator_gui::DistributeImage")

add_executable(operator_gui_node src/operator_gui_component.cpp src/operator_gui_node.cpp src/gui_tool.cpp)
ament_target_dependencies(operator_gui_node ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  operator_gui_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  operator_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
