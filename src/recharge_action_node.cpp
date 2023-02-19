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

#include "rclcpp_components/register_node_macro.hpp"

#include "mros2_msgs/action/control_qos.hpp"
#include "std_msgs/msg/bool.hpp"
#include<sstream>
#include<string.h>
#include<ctime>

using namespace std::chrono_literals;

class RechargeAction : public plansys2::ActionExecutorClient
{
public:
  using ControlQos = mros2_msgs::action::ControlQos;
  using GoalHandleControlQos = rclcpp_action::ClientGoalHandle<ControlQos>;

  RechargeAction()
  : plansys2::ActionExecutorClient("recharge", 500ms)
  {
    RCLCPP_INFO(this->get_logger(), "At beginning of Startup");
    not_started = true;
    pipeline_detected = false;
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = client_cb_group_;
    detected_subscriber = this->create_subscription<std_msgs::msg::Bool>(
      "pipeline/detected", 10, std::bind(&RechargeAction::detected_callback, this, std::placeholders::_1), sub_opt
    );
    this->client = rclcpp_action::create_client<ControlQos>(this,"mros/objective");
    RCLCPP_INFO(this->get_logger(), "At end of Startup");
  }

private:
  void do_work()
  {
    using namespace std::placeholders;
    send_feedback(0.0, "Started recharge action");
    std::stringstream obj_1;
    std::stringstream obj_2;
    auto function_1 = get_arguments()[1];
    auto function_2 = get_arguments()[2];
    obj_1<<"obj_"<<function_1;
    obj_2<<"obj_"<<function_2;

    if (not_started) {
      if (!this->client->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      RCLCPP_INFO(this->get_logger(), "Sending objective");
      auto send_goal_options = rclcpp_action::Client<ControlQos>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        std::bind(&RechargeAction::goal_response_callback, this, _1);
      send_goal_options.feedback_callback =
        std::bind(&RechargeAction::feedback_callback, this, _1, _2);
      send_goal_options.result_callback =
        std::bind(&RechargeAction::result_callback, this, _1);

      auto goal_msg_1 = ControlQos::Goal();
      goal_msg_1.qos_expected.objective_type = function_1;
      obj_1<<"_"<<this->get_clock()->now().seconds();
      goal_msg_1.qos_expected.objective_id = obj_1.str();
      goal_msg_1.qos_expected.selected_mode = "";

      RCLCPP_INFO_STREAM(this->get_logger(), "Sending objective 1: "<<obj_1.str());
      this->client->async_send_goal(goal_msg_1, send_goal_options);

      auto goal_msg_2 = ControlQos::Goal();
      goal_msg_2.qos_expected.objective_type = function_2;
      obj_2<<"_"<<this->get_clock()->now().seconds();
      goal_msg_2.qos_expected.objective_id = obj_2.str();
      goal_msg_2.qos_expected.selected_mode = "";

      RCLCPP_INFO_STREAM(this->get_logger(), "Sending objective 2: "<<obj_2.str());
      this->client->async_send_goal(goal_msg_2, send_goal_options);

      not_started = false;
    }

    if (pipeline_detected) {
      if (!this->client->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      this->client->async_cancel_all_goals();
      RCLCPP_INFO(this->get_logger(), "Recharge is done");

      finish(true, 1.0, "Recharge completed");
    }
  }

  void detected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    pipeline_detected = msg->data;
    RCLCPP_INFO(this->get_logger(), "Pipeline Detected Received");
  }

  bool not_started;
  bool pipeline_detected;
  rclcpp_action::Client<ControlQos>::SharedPtr client; 
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detected_subscriber;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  void goal_response_callback(GoalHandleControlQos::SharedPtr future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleControlQos::SharedPtr,
    const std::shared_ptr<const ControlQos::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Performing Recharge Action");
  }

  void result_callback(const GoalHandleControlQos::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Recharge is done");
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RechargeAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "recharge"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
