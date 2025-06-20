/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_pac/pac_status_panel.hpp>
using namespace std::chrono;
using namespace std::chrono_literals;

namespace rviz_pac {
PACStatusPanel::PACStatusPanel(QWidget* parent) : Panel(parent), qos_(1) {
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_ = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
      qos_profile);

  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("PAC status: 2");
  label_->setStyleSheet("color: red; font-weight: bold;");
  radio_button_ready_ = new QRadioButton("Ready");
  radio_button_pause_ = new QRadioButton("Pause");
  radio_button_stop_ = new QRadioButton("Stop");
  radio_button_stop_->setChecked(true);

  layout->addWidget(label_);
  layout->addWidget(radio_button_ready_);
  layout->addWidget(radio_button_pause_);
  layout->addWidget(radio_button_stop_);

  const auto idf_layout = new QHBoxLayout;
  idf_layout->addWidget(new QLabel("IDF File:"));
  idf_file_input_ = new QLineEdit;
  idf_layout->addWidget(idf_file_input_);
  reset_button_ = new QPushButton("Reset World");
  idf_layout->addWidget(reset_button_);

  layout->addLayout(idf_layout);
  output_text_ = new QTextEdit;
  output_text_->setReadOnly(true);
  output_text_->setStyleSheet("background-color: lightgray;");
  layout->addWidget(output_text_);

  connect(radio_button_ready_, &QRadioButton::toggled, this, [=](bool checked) {
    if (checked) {
      pac_status_ = 0;
      label_->setText("PAC status: 0");
      label_->setStyleSheet("color: green; font-weight: bold;");
    }
  });

  connect(radio_button_pause_, &QRadioButton::toggled, this, [=](bool checked) {
    if (checked) {
      pac_status_ = 1;
      label_->setText("PAC status: 1");
      label_->setStyleSheet("color: blue; font-weight: bold;");
    }
  });

  connect(radio_button_stop_, &QRadioButton::toggled, this, [=](bool checked) {
    if (checked) {
      pac_status_ = 2;
      label_->setText("PAC status: 2");
      label_->setStyleSheet("color: red; font-weight: bold;");
    }
  });

  connect(idf_file_input_, &QLineEdit::editingFinished, this, [=]() {
    idf_file_name_ = idf_file_input_->text();
  });

  connect(reset_button_, &QPushButton::clicked, this, [=]() {
    if (pac_status_ == 0) {
      output_text_->append("<span style='color: red;'>Cannot reset world while PAC is ready.</span>");
      return;
    }
    output_text_->append("Resetting world to: " + idf_file_name_);
    UpdateWorldFile();
  });
}

PACStatusPanel::~PACStatusPanel() = default;

void PACStatusPanel::onInitialize() {
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making
  // subscriptions/publishers (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_ =
      node->create_publisher<std_msgs::msg::Int32>("/pac_gcs/status_pac", qos_);
  timer_ =
      node->create_wall_timer(std::chrono::milliseconds(500), [this]() -> void {
        auto msg = std_msgs::msg::Int32();
        msg.data = pac_status_;
        publisher_->publish(msg);
      });
  GetSystemInfo();
}

void PACStatusPanel::UpdateWorldFile() {
  using UpdateWorldFileAction = async_pac_gnn_interfaces::action::UpdateWorldFile;
  using GoalHandleUpdateWorldFile = rclcpp_action::ClientGoalHandle<UpdateWorldFileAction>;
  
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  
  // Create action client
  update_world_action_client_ = rclcpp_action::create_client<UpdateWorldFileAction>(
      node, "update_world");
      
  // Wait for action server
  if (!update_world_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    output_text_->append("<span style='color: red;'>UpdateWorld action server not available\n Run pac update_world</span>");
    return;
  }
  
  // Create goal
  auto goal_msg = UpdateWorldFileAction::Goal();
  goal_msg.file = idf_file_name_.toStdString();
  
  output_text_->clear();
  output_text_->append("Sending update world request for: " + idf_file_name_);
  
  // Set up goal options with callbacks
  auto send_goal_options = rclcpp_action::Client<UpdateWorldFileAction>::SendGoalOptions();
  
  // Goal response callback
  send_goal_options.goal_response_callback =
    [output = output_text_](const GoalHandleUpdateWorldFile::SharedPtr& goal_handle) {
      if (!goal_handle) {
        output->append("<span style='color: red;'>Goal was rejected by server</span>");
      } else {
        output->append("Goal accepted by server, processing...");
      }
    };
    
  // Feedback callback
  send_goal_options.feedback_callback =
    [output = output_text_](
      GoalHandleUpdateWorldFile::SharedPtr,
      const std::shared_ptr<const UpdateWorldFileAction::Feedback> feedback) {
      QString progress = QString("%1/%2")
                         .arg(feedback->robots_completed)
                         .arg(feedback->robots_total);
      output->append(progress);
      
      if (!feedback->message.empty()) {
        output->append(QString::fromStdString(feedback->message));
      }
    };
    
  // Result callback
  send_goal_options.result_callback =
    [output = output_text_](const GoalHandleUpdateWorldFile::WrappedResult& result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success) {
            output->append("<span style='color: green;'>Complete</span>");
          } else {
            output->append("<span style='color: red;'>Failed</span>");
            if (!result.result->message.empty()) {
              output->append(QString::fromStdString(result.result->message));
            }
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          output->append("<span style='color: red;'>Action was aborted</span>");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          output->append("<span style='color: red;'>Action was canceled</span>");
          break;
        default:
          output->append("<span style='color: red;'>Unknown result code</span>");
          break;
      }
    };
  
  // Send goal
  update_world_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void PACStatusPanel::GetSystemInfo() {
  using async_pac_gnn_interfaces::srv::SystemInfo;
  using ServiceResponseFuture =
      rclcpp::Client<SystemInfo>::SharedFutureWithRequest;
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  system_info_client_ =
      node->create_client<async_pac_gnn_interfaces::srv::SystemInfo>(
          "get_system_info");
  while (!system_info_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      return;
    }
    RCLCPP_WARN(node->get_logger(), "waiting for get_system_info");
  }
  auto get_world_file_cb =
      [this](ServiceResponseFuture future) {
        auto request_response_pair = future.get();
        auto response = request_response_pair.second;
        idf_file_input_->setText(QString::fromStdString(response->idf_file));
        idf_file_name_ = QString::fromStdString(response->idf_file);
      };

  auto request =
      std::make_shared<SystemInfo::Request>();
  request->name = "pac_rviz_panel";
  system_info_client_->async_send_request(
      request, std::move(get_world_file_cb));

}
}  // namespace rviz_pac

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_pac::PACStatusPanel, rviz_common::Panel)
