cmake_minimum_required(VERSION 3.5)
project(plansys2_suave)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mros2_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_components
    plansys2_msgs
    plansys2_executor
    std_msgs
    mros2_msgs
)

add_executable(search_action_node src/search_action_node.cpp)
ament_target_dependencies(search_action_node ${dependencies})

add_executable(follow_action_node src/follow_action_node.cpp)
ament_target_dependencies(follow_action_node ${dependencies})

add_executable(recharge_action_node src/recharge_action_node.cpp)
ament_target_dependencies(recharge_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  search_action_node
  follow_action_node
  recharge_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()