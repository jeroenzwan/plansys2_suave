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
#include "mros2_msgs/msg/qo_s.hpp"
#include "mros2_msgs/msg/qo_s_list.hpp"
#include "std_msgs/msg/bool.hpp"
#include<sstream>
#include<string.h>
#include<ctime>

using namespace std::chrono_literals;

class FollowAction : public plansys2::ActionExecutorClient
{
public:
  using ControlQos = mros2_msgs::action::ControlQos;
  using GoalHandleControlQos = rclcpp_action::ClientGoalHandle<ControlQos>;

  FollowAction()
  : plansys2::ActionExecutorClient("follow_pipeline", 500ms)
  {
    RCLCPP_INFO(this->get_logger(), "At beginning of Startup");
    old_fd_1 = "nothing";
    old_fd_2 = "nothing";
    started = false;
    pipeline_inspected = false;
    message = mros2_msgs::msg::QoSList();
    msg_1 = mros2_msgs::msg::QoS();
    msg_2 = mros2_msgs::msg::QoS();
    // client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    // sub_opt.callback_group = client_cb_group_;
    inspected_subscriber = this->create_subscription<std_msgs::msg::Bool>(
      "pipeline/inspected", 10, std::bind(&FollowAction::inspected_callback, this, std::placeholders::_1), sub_opt
    );
    // this->client = rclcpp_action::create_client<ControlQos>(this,"mros/objective");
    obj_pub = this->create_publisher<mros2_msgs::msg::QoSList>("mros/sub_objective", 10);
    obj_pub->on_activate();
    RCLCPP_INFO(this->get_logger(), "At end of Startup");
  }

private:
  void do_work()
  {
    using namespace std::placeholders;
    send_feedback(0.0, "Started follow action");
    std::stringstream obj_1;
    std::stringstream obj_2;
    auto function_1 = get_arguments()[1];
    auto function_2 = get_arguments()[2];
    obj_1<<"obj_"<<function_1;
    obj_2<<"obj_"<<function_2;

    auto fd_1 = get_arguments()[3];
    auto fd_2 = get_arguments()[4];

    if ((old_fd_1!=fd_1)||(old_fd_2!=fd_2)) {
      // if (!this->client->wait_for_action_server()) {
      //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      //   rclcpp::shutdown();
      // }
      if (started){
        RCLCPP_INFO_STREAM(this->get_logger(), "Same action executed");
        // this->client->async_cancel_all_goals();
        RCLCPP_INFO_STREAM(this->get_logger(), "Canceling objective 1: "<<msg_1.objective_id);
        RCLCPP_INFO_STREAM(this->get_logger(), "Canceling FD 1: "<<msg_1.selected_mode);
        RCLCPP_INFO_STREAM(this->get_logger(), "Canceling objective 2: "<<msg_2.objective_id);
        RCLCPP_INFO_STREAM(this->get_logger(), "Canceling FD 2: "<<msg_2.selected_mode);
        message.cancel = true;
        obj_pub->publish(message);
        rclcpp::sleep_for(1s);
      }
      // auto send_goal_options = rclcpp_action::Client<ControlQos>::SendGoalOptions();
      // send_goal_options.goal_response_callback =
      //   std::bind(&FollowAction::goal_response_callback, this, _1);
      // send_goal_options.feedback_callback =
      //   std::bind(&FollowAction::feedback_callback, this, _1, _2);
      // send_goal_options.result_callback =
      //   std::bind(&FollowAction::result_callback, this, _1);

      message.cancel = false;

      msg_1.objective_type = function_1;
      obj_1<<"_"<<this->get_clock()->now().seconds();
      msg_1.objective_id = obj_1.str();
      msg_1.selected_mode = fd_1;

      // auto goal_msg_1 = ControlQos::Goal();
      // goal_msg_1.qos_expected.objective_type = function_1;
      // obj_1<<"_"<<this->get_clock()->now().seconds();
      // goal_msg_1.qos_expected.objective_id = obj_1.str();
      // goal_msg_1.qos_expected.selected_mode = fd_1;

      msg_2.objective_type = function_2;
      obj_2<<"_"<<this->get_clock()->now().seconds();
      msg_2.objective_id = obj_2.str();
      msg_2.selected_mode = fd_2;

      message.list = {msg_1,msg_2};
      obj_pub->publish(message);

      // rclcpp::sleep_for(300ms);
      RCLCPP_INFO_STREAM(this->get_logger(), "Sending objective 1: "<<obj_1.str());
      RCLCPP_INFO_STREAM(this->get_logger(), "Sending FD 1: "<<fd_2);
      // this->client->async_send_goal(goal_msg_1, send_goal_options);

      // auto goal_msg_2 = ControlQos::Goal();
      // goal_msg_2.qos_expected.objective_type = function_2;
      // obj_2<<"_"<<this->get_clock()->now().seconds();
      // goal_msg_2.qos_expected.objective_id = obj_2.str();
      // goal_msg_2.qos_expected.selected_mode = fd_2;

      RCLCPP_INFO_STREAM(this->get_logger(), "Sending objective 2: "<<obj_2.str());
      RCLCPP_INFO_STREAM(this->get_logger(), "Sending FD 2: "<<fd_2);
      // this->client->async_send_goal(goal_msg_2, send_goal_options);

      old_fd_1.replace(old_fd_1.begin(),old_fd_1.end(),fd_1);
      old_fd_2.replace(old_fd_2.begin(),old_fd_2.end(),fd_2);
    }

    if (pipeline_inspected) {
      // if (!this->client->wait_for_action_server()) {
      //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      //   rclcpp::shutdown();
      // }
      message.cancel = true;
      RCLCPP_INFO_STREAM(this->get_logger(), "Canceling objective 1: "<<msg_1.objective_id);
      RCLCPP_INFO_STREAM(this->get_logger(), "Canceling FD 1: "<<msg_1.selected_mode);
      RCLCPP_INFO_STREAM(this->get_logger(), "Canceling objective 2: "<<msg_2.objective_id);
      RCLCPP_INFO_STREAM(this->get_logger(), "Canceling FD 2: "<<msg_2.selected_mode);
      obj_pub->publish(message);

      // this->client->async_cancel_all_goals();
      RCLCPP_INFO(this->get_logger(), "Follow is done");

      rclcpp::sleep_for(1s);

      finish(true, 1.0, "Follow completed");
    }
  }

  void inspected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    pipeline_inspected = msg->data;
    RCLCPP_INFO(this->get_logger(), "Pipeline Inspected Received");
  }

  bool started;
  bool pipeline_inspected;
  std::string old_fd_1;
  std::string old_fd_2;
  mros2_msgs::msg::QoSList message;
  mros2_msgs::msg::QoS msg_1;
  mros2_msgs::msg::QoS msg_2;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<mros2_msgs::msg::QoSList>> obj_pub;
  // rclcpp_action::Client<ControlQos>::SharedPtr client; 
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr inspected_subscriber;
  // rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // void goal_response_callback(GoalHandleControlQos::SharedPtr future)
  // {
  //   auto goal_handle = future.get();
  //   if (!goal_handle) {
  //     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  //   } else {
  //     RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  //   }
  // }

  // void feedback_callback(
  //   GoalHandleControlQos::SharedPtr,
  //   const std::shared_ptr<const ControlQos::Feedback> feedback)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Performing Follow Action");
  // }

  // void result_callback(const GoalHandleControlQos::WrappedResult & result)
  // {
  //   switch (result.code) {
  //     case rclcpp_action::ResultCode::SUCCEEDED:
  //       break;
  //     case rclcpp_action::ResultCode::ABORTED:
  //       RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
  //       return;
  //     case rclcpp_action::ResultCode::CANCELED:
  //       RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
  //       return;
  //     default:
  //       RCLCPP_ERROR(this->get_logger(), "Unknown result code");
  //       return;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "Follow is done");
  //   rclcpp::shutdown();
  // }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "follow_pipeline"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
