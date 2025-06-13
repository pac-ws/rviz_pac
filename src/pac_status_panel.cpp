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
  GetWorldFile();
}

void PACStatusPanel::UpdateWorldFile() {
  using async_pac_gnn_interfaces::srv::UpdateWorldFile;
  using ServiceResponseFuture =
          rclcpp::Client<UpdateWorldFile>::SharedFutureWithRequest;
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  update_world_client_ =
      node->create_client<UpdateWorldFile>(
          "/sim/update_world_file");
  while (!update_world_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      return;
    }
  }
  auto update_world_file_cb =
    [output = output_text_](ServiceResponseFuture future) {
      auto request_response_pair = future.get();
      if (request_response_pair.second->success) {
        output->append(
            "World file updated successfully");
      } else {
        output->append("Filed to update world file");
      }
    };
  auto request =
      std::make_shared<UpdateWorldFile::Request>();
  request->file = idf_file_name_.toStdString();
  update_world_client_->async_send_request(
      request, std::move(update_world_file_cb));
}

void PACStatusPanel::GetWorldFile() {
  using async_pac_gnn_interfaces::srv::WorldFile;
  using ServiceResponseFuture =
      rclcpp::Client<WorldFile>::SharedFutureWithRequest;
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  world_file_client_ =
      node->create_client<async_pac_gnn_interfaces::srv::WorldFile>(
          "/sim/get_world_file");
  while (!world_file_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      return;
    }
  }
  auto get_world_file_cb =
      [this](ServiceResponseFuture future) {
        auto request_response_pair = future.get();
        idf_file_input_->setText(QString::fromStdString(request_response_pair.second->file));
        idf_file_name_ = QString::fromStdString(request_response_pair.second->file);
      };

  auto request =
      std::make_shared<WorldFile::Request>();
  request->name = "pac_rviz_panel";
  world_file_client_->async_send_request(
      request, std::move(get_world_file_cb));

}
}  // namespace rviz_pac

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_pac::PACStatusPanel, rviz_common::Panel)
