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

#include <stdio.h>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_pac/xy_drive_widget.hpp>
#include <rviz_pac/xy_teleop_panel.hpp>

namespace rviz_pac {

// BEGIN_TUTORIAL
// Here is the implementation of the XYTeleopPanel class.  XYTeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements XYDriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
XYTeleopPanel::XYTeleopPanel(QWidget* parent)
    : rviz_common::Panel(parent),
      drive_widget_(nullptr),
      output_topic_editor_(nullptr),
      velocity_node_abstract_(nullptr),
      velocity_publisher_(nullptr),
      vel_x_(0.),
      vel_y_(0.),
      qos_(1),
      enabled_(false),
      max_speed_(1.0) {
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_ = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
      qos_profile);

  enable_checkbox_ = new QCheckBox("");
  enable_checkbox_->setChecked(false);
  max_speed_box_ = new QDoubleSpinBox();
  max_speed_box_->setRange(0, 10.0);
  max_speed_box_->setSingleStep(0.1);
  max_speed_box_->setValue(max_speed_);

  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  const auto topic_layout = new QHBoxLayout;
  topic_layout->addWidget(enable_checkbox_);
  topic_layout->addWidget(new QLabel("Topic:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_);

  const auto display_vel_layout = new QHBoxLayout;
  display_vel_layout->addWidget(new QLabel("Velocity "));
  display_vel_layout->addWidget(new QLabel("x:"));
  display_vel_x_ = new QLineEdit;
  display_vel_x_->setReadOnly(true);
  display_vel_x_->setText("");
  display_vel_layout->addWidget(display_vel_x_);
  display_vel_layout->addWidget(new QLabel("y:"));
  display_vel_y_ = new QLineEdit;
  display_vel_y_->setReadOnly(true);
  display_vel_y_->setText("");
  display_vel_layout->addWidget(display_vel_y_);

  const auto max_speed_layout = new QHBoxLayout;
  max_speed_label_ = new QLabel("Max Speed:");
  max_speed_layout->addWidget(max_speed_label_);
  max_speed_layout->addWidget(max_speed_box_);
  // Then create the control widget.
  drive_widget_ = new XYDriveWidget;

  // Lay out the topic field above the control widget.
  const auto layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(display_vel_layout);
  layout->addLayout(max_speed_layout);
  layout->addWidget(drive_widget_);
  setLayout(layout);
  // Next we make signal/slot connections.
  connect(drive_widget_, SIGNAL(outputVelocity(float, float)), this,
          SLOT(setVel(float, float)));
  connect(enable_checkbox_, &QCheckBox::toggled, this, [=](bool checked) {
    enabled_ = checked;
    changeEnable();
  });
  connect(output_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateTopic()));

  connect(max_speed_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &XYTeleopPanel::setMaxSpeed);
  /* connect(max_speed_box_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), */
  /*         this, [=](double value) { max_speed_ = static_cast<float>(value); Q_EMIT configChanged(); }); */

  // Make the control widget start disabled, since we don't start with an output
  // topic.
}

void XYTeleopPanel::changeEnable() {
  max_speed_label_->setEnabled(enabled_);
  max_speed_box_->setEnabled(enabled_);
  display_vel_x_->setEnabled(enabled_);
  display_vel_y_->setEnabled(enabled_);
  drive_widget_->setEnabled(enabled_);
}

void XYTeleopPanel::setMaxSpeed(double value) {
  max_speed_ = static_cast<float>(value);
  Q_EMIT configChanged();
}

void XYTeleopPanel::onInitialize() {
  velocity_node_abstract_ = getDisplayContext()->getRosNodeAbstraction().lock();
  velocity_node_ = velocity_node_abstract_->get_raw_node();
  timer_ = velocity_node_->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&XYTeleopPanel::sendVel, this));
  changeEnable();
}

// setVel() is connected to the XYDriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.
void XYTeleopPanel::setVel(float v_x, float v_y) {
  if (enabled_ == false) {
    return;
  }
  vel_x_ = v_x * max_speed_;
  vel_y_ = v_y * max_speed_;
  auto speed = std::sqrt(vel_x_ * vel_x_ + vel_y_ * vel_y_);
  if (speed > max_speed_) {
    vel_x_ = vel_x_ / speed * max_speed_;
    vel_y_ = vel_y_ / speed * max_speed_;
  }
  // Get velocity as string to two decimal places
  QString vel_x_str = QString::number(vel_x_, 'f', 2);
  QString vel_y_str = QString::number(vel_y_, 'f', 2);
  display_vel_x_->setText(vel_x_str);
  display_vel_y_->setText(vel_y_str);
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void XYTeleopPanel::updateTopic() { setTopic(output_topic_editor_->text()); }

// Set the topic name we are publishing to.
void XYTeleopPanel::setTopic(const QString& new_topic) {
  // Only take action if the name has changed.
  if (new_topic != output_topic_) {
    output_topic_ = new_topic;
    // If a publisher currently exists, destroy it.
    if (velocity_publisher_ != NULL) {
      velocity_publisher_.reset();
    }
    // If the topic is the empty string, don't publish anything.
    if (output_topic_ != "") {
      // The call to create_publisher() says we want to publish data on the new
      // topic name.
      velocity_publisher_ =
          velocity_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
              output_topic_.toStdString(), qos_);
    }
    // rviz_common::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz_common::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

  // Gray out the control widget when the output topic is empty.
  drive_widget_->setEnabled(output_topic_ != "" && enabled_);
}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void XYTeleopPanel::sendVel() {
  if (enabled_ == true && rclcpp::ok() && velocity_publisher_ != NULL) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = velocity_node_->now();
    msg.header.frame_id = "map";
    msg.twist.linear.x = vel_x_;
    msg.twist.linear.y = vel_y_;
    msg.twist.linear.z = 0;
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = 0;
    velocity_publisher_->publish(msg);
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void XYTeleopPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);
  config.mapSetValue("MaxSpeed", max_speed_);
}

// Load all configuration data for this panel from the given Config object.
void XYTeleopPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    output_topic_editor_->setText(topic);
    updateTopic();
  }
  if (config.mapGetFloat("MaxSpeed", &max_speed_)) {
    max_speed_box_->setValue(max_speed_);
  }
}

}  // end namespace rviz_pac

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
PLUGINLIB_EXPORT_CLASS(rviz_pac::XYTeleopPanel, rviz_common::Panel)
// END_TUTORIAL
