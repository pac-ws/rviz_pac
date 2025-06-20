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

#ifndef RVIZ_PAC__MAIN_PANEL_HPP_
#define RVIZ_PAC__MAIN_PANEL_HPP_

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QTextEdit>
#include <async_pac_gnn_interfaces/action/update_world_file.hpp>
#include <async_pac_gnn_interfaces/srv/system_info.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/int32.hpp>

namespace rviz_pac {
class PACStatusPanel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit PACStatusPanel(QWidget* parent = 0);
  ~PACStatusPanel() override;

  void onInitialize() override;
  void GetSystemInfo();
  void UpdateWorldFile();

 protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>
      node_ptr_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  QLabel* label_;
  QRadioButton* radio_button_ready_;
  QRadioButton* radio_button_pause_;
  QRadioButton* radio_button_stop_;
  // Add a button to reset world, a text field to enter IDF file name, and a
  // read-only text field to display output messages
  QPushButton* reset_button_;
  QLineEdit* idf_file_input_;
  QString idf_file_name_;
  QTextEdit* output_text_;


 private:
  int pac_status_ = 2;
  rclcpp::QoS qos_;
  rclcpp_action::Client<async_pac_gnn_interfaces::action::UpdateWorldFile>::SharedPtr
      update_world_action_client_;
  rclcpp::Client<async_pac_gnn_interfaces::srv::SystemInfo>::SharedPtr
      system_info_client_;
};

}  // namespace rviz_pac

#endif  // RVIZ_PAC__MAIN_PANEL_HPP_
