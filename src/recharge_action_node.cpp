// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mros2_msgs/action/ControlQos.hpp"
#include "std_msgs/msg/Bool.hpp"
#include<string.h>
#include<ctime>

using namespace std::chrono_literals;

class RechargeAction : public plansys2::ActionExecutorClient
{
public:
  RechargeAction()
  : plansys2::ActionExecutorClient("recharge_pipeline", 250ms)
  {
    not_started = false;
    pipeline_detected = false;
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    detected_subscriber = this->create_subscription<std_msgs::msg::Bool>(
      "pipeline/recharged", 10, std::bind(&RechargeAction::detected_callback, this, _1), client_cb_group_
    );
    client = this->create_cliet<mros2_msgs::srv::ControlQos>("mros/objective");
    while (!client->wait_for_service(0s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return -1;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
  }

private:
  void do_work()
  {
    send_feedback(0.0, "Started recharge action");
    std::stringstream obj_1;
    std::stringstream obj_2;
    auto function_1 = get_arguments()[2];
    auto function_2 = get_arguments()[3];
    obj_1<<"obj_"<<function_1;
    obj_2<<"obj_"<<function_2;

    if not_started {
      mros2_msgs::srv::ControlQos request = std::make_shared<mros2_msgs:srv::ControlQos::request>();
      request.qos_expected.objective_type = function_1;
      obj_1<<"_"<<this->get_clock()->now();
      request.qos_expected.objective_id = obj_1.str();
      request.qos_expected.select_mode = "";
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending adaptation goal "+obj_1);
      auto goal_future = client->async_send_request(request);
      // if (rclcpp::spin_until_future_complete(this, result) ==
      //   rclcpp::executor::FutureReturnCode::SUCCESS)
      // {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adaptation goal sent!!!");
      // } else {
      //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service for setting objective");    // CHANGE
      // }
    }

    if pipeline_recharged {
      goal_future.result().async_cancel_goal();
      finish(true, 1.0, "Recharge completed");
    }
  }

  void detected_callback(const std_msgs::msg::Bool::SharedPtr msg) const{
    pipeline_recharged = msg.data;
    RCLCPP_INFO(this->get_logger(), "Pipeline Recharg Received");
  }

  bool not_started;
  bool pipeline_recharged;
  rclcpp::Client<mros2_msgs::srv::ControlQos>::SharedPtr client; 
  rclcpp::Subscription<<<std_msgs::msg::Bool>::SharedPtr detected_subscriber;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RechargeAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "recharge_pipeline"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
