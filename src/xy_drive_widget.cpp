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

#define _USE_MATH_DEFINES
#include <stdio.h>

#include <QMouseEvent>
#include <QPainter>
#include <cmath>
#include <rviz_pac/xy_drive_widget.hpp>

namespace rviz_pac {

// BEGIN_TUTORIAL
// The XYDriveWidget constructor does the normal Qt thing of
// passing the parent widget to the superclass constructor, then
// initializing the member variables.
XYDriveWidget::XYDriveWidget(QWidget* parent)
    : QWidget(parent), vel_x_(0.0f), vel_y_(0.0f) {}

// This paintEvent() is complex because of the drawing of the two
// arc-arrows representing wheel motion.  It is not particularly
// relevant to learning how to make an RViz plugin, so I will kind of
// skim it.
void XYDriveWidget::paintEvent(QPaintEvent* event) {
  (void)event;

  // The background color and crosshair lines are drawn differently
  // depending on whether this widget is enabled or not.  This gives a
  // nice visual indication of whether the control is "live".
  QColor background;
  QColor crosshair;
  if (isEnabled()) {
    background = Qt::white;
    crosshair = Qt::black;
  } else {
    background = Qt::lightGray;
    crosshair = Qt::darkGray;
  }

  // The main visual is a square, centered in the widget's area.  Here
  // we compute the size of the square and the horizontal and vertical
  // offsets of it.
  int w = width();
  int h = height();
  int size = ((w > h) ? h : w) - 1;
  int hpad = (w - size) / 2;
  int vpad = (h - size) / 2;

  QPainter painter(this);
  painter.setBrush(background);
  painter.setPen(crosshair);

  // Draw the background square.
  painter.drawRect(QRect(hpad, vpad, size, size));

  // Draw a cross-hair inside the square.
  painter.drawLine(hpad, height() / 2, hpad + size, height() / 2);
  painter.drawLine(width() / 2, vpad, width() / 2, vpad + size);

  // Draw arrow from center to mouse position.
  /* vel_x_ = -(1.0 - static_cast<float>(x - hpad) / static_cast<float>(size / 2)); */
  /* vel_y_ = (1.0 - static_cast<float>(y - vpad) / static_cast<float>(size / 2)); */
  /* float pt1_x = width() / 2; */
  /* float pt1_y = height() / 2; */
  /* float pt2_x = vel_x_ * size/2.0 + pt1_x * 2 - size/2.0; */
  /* float pt2_y = -vel_y_ * size/2.0 + pt1_y * 2 - size/2.0; */
  /* painter.drawLine(pt1_x, pt1_y, pt2_x, pt2_y); */

}

// Every mouse move event received here sends a velocity because Qt
// only sends us mouse move events if there was previously a
// mouse-press event while in the widget.
void XYDriveWidget::mouseMoveEvent(QMouseEvent* event) {
  sendVelocitiesFromMouse(event->x(), event->y(), width(), height());
}

// Mouse-press events should send the velocities too, of course.
void XYDriveWidget::mousePressEvent(QMouseEvent* event) {
  sendVelocitiesFromMouse(event->x(), event->y(), width(), height());
}

// When the mouse leaves the widget but the button is still held down,
// we don't get the leaveEvent() because the mouse is "grabbed" (by
// default from Qt).  However, when the mouse drags out of the widget
// and then other buttons are pressed (or possibly other
// window-manager things happen), we will get a leaveEvent() but not a
// mouseReleaseEvent().  Without catching this event you can have a
// robot stuck "on" without the user controlling it.
void XYDriveWidget::leaveEvent(QEvent* event) {
  (void)event;
  stop();
}

// The ordinary way to stop: let go of the mouse button.
void XYDriveWidget::mouseReleaseEvent(QMouseEvent* event) {
  (void)event;
  stop();
}

// Compute and emit linear and angular velocities based on Y and X
// mouse positions relative to the central square.
void XYDriveWidget::sendVelocitiesFromMouse(int x, int y, int width, int height) {
  int size = ((width > height) ? height : width);
  int hpad = (width - size) / 2;
  int vpad = (height - size) / 2;

  vel_x_ = -(1.0 - static_cast<float>(x - hpad) / static_cast<float>(size / 2));
  vel_y_ = (1.0 - static_cast<float>(y - vpad) / static_cast<float>(size / 2));
  vel_x_ = std::min(std::max(vel_x_, -1.0f), 1.0f);
  vel_y_ = std::min(std::max(vel_y_, -1.0f), 1.0f);
  Q_EMIT outputVelocity(vel_x_, vel_y_);

  // update() is a QWidget function which schedules this widget to be
  // repainted the next time through the main event loop.  We need
  // this because the velocities have just changed, so the arrows need
  // to be redrawn to match.
  update();
}

// How to stop: emit velocities of 0!
void XYDriveWidget::stop() {
  vel_x_ = 0.0f;
  vel_y_ = 0.0f;
  Q_EMIT outputVelocity(vel_x_, vel_y_);
  update();
}
// END_TUTORIAL

}  // end namespace rviz_pac
