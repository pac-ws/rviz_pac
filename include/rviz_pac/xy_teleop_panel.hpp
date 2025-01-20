// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RVIZ_PAC__TELEOP_PANEL_HPP_
#define RVIZ_PAC__TELEOP_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#endif

class QLabel;
class QLineEdit;
class QCheckBox;
class QDoubleSpinBox;

namespace rviz_pac {

class XYDriveWidget;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz_common::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz_common::Panel.
//
// XYTeleopPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// XYDriveWidget class, and is described there.
class XYTeleopPanel : public rviz_common::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT

 public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  explicit XYTeleopPanel(QWidget* parent = 0);
  void onInitialize() override;

  // Now we declare overrides of rviz_common::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

  // Next come a couple of public Qt slots.

 public Q_SLOTS:
  // The control area, XYDriveWidget, sends its output to a Qt signal
  // for ease of re-use, so here we declare a Qt slot to receive it.
  void setVel(float vel_x_, float vel_y_);

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void setTopic(const QString& topic);
  void setMaxSpeed(double max_speed);

  // Here we declare some internal slots.

 protected Q_SLOTS:
  // sendvel() publishes the current velocity values to a ROS
  // topic.  Internally this is connected to a timer which calls it 10
  // times per second.
  void sendVel();

  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void updateTopic();
  void updateScale();
  void changeEnable();

  // Then we finish up with protected member variables.

 protected:
  // The control-area widget which turns mouse events into command
  // velocities.
  XYDriveWidget* drive_widget_;

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* output_topic_editor_;

  QLineEdit* display_vel_x_;
  QLineEdit* display_vel_y_;
  QLabel* max_speed_label_;
  QDoubleSpinBox* max_speed_box_;

  // The current name of the output topic.
  QString output_topic_;

  // The ROS node and publisher for the command velocity.
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>
      velocity_node_abstract_;
  rclcpp::Node::SharedPtr velocity_node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // The latest velocity values from the drive widget.
  float vel_x_;
  float vel_y_;
  rclcpp::QoS qos_;
  QCheckBox* enable_checkbox_;
  bool enabled_;
  float max_speed_;
  // END_TUTORIAL
};

}  // end namespace rviz_pac

#endif  // RVIZ_PAC__TELEOP_PANEL_HPP_
