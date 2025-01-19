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
#include <rviz_pac_panel/main_panel.hpp>

namespace rviz_pac_panel {
MainPanel::MainPanel(QWidget* parent) : Panel(parent) {
  // Create a label and a button, displayed vertically (the V in VBox means
  // vertical)
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
}

MainPanel::~MainPanel() = default;

void MainPanel::onInitialize() {
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making
  // subscriptions/publishers (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_ =
      node->create_publisher<std_msgs::msg::Int32>("/pac_gcs/status_pac", 10);
  timer_ = node->create_wall_timer(
      std::chrono::milliseconds(500), [this]() -> void {
        auto msg = std_msgs::msg::Int32();
        msg.data = pac_status_;
        publisher_->publish(msg);
      });
}

}  // namespace rviz_pac_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_pac_panel::MainPanel, rviz_common::Panel)
