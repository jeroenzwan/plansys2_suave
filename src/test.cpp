
#include <memory>
#include <algorithm>

#include<iostream>
#include<sstream>
#include<chrono>
#include<ctime>
#include "/opt/ros/humble/include/rclcpp/rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  rclcpp::Node node = rclcpp::Node("node1")
  rclcpp::Time start_time = node->get_clock()->now();
  std::cout<<start_time<<std::endl;
  std::stringstream obj_1;
  std::stringstream obj_2;
  auto function_1 = "f_mock1";
  auto function_2 = "f_mock1";
  obj_1<<"obj_"<<function_1;
  obj_2<<"obj_"<<function_2;
  std::cout<<obj_1.str()<<std::endl;
  std::cout<<obj_2.str()<<std::endl;
  return 0;
}