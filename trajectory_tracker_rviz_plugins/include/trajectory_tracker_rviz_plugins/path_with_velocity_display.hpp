/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// This file is based on rviz_default_plugins' path display.

#ifndef TRAJECTORY_TRACKER_RVIZ_PLUGINS__PATH_WITH_VELOCITY_DISPLAY_HPP_
#define TRAJECTORY_TRACKER_RVIZ_PLUGINS__PATH_WITH_VELOCITY_DISPLAY_HPP_

#include <OgreMaterial.h>
#include <OgreMatrix4.h>

#include <cstddef>
#include <vector>

#include "rviz_common/message_filter_display.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "trajectory_tracker_msgs/msg/path_with_velocity.hpp"

namespace Ogre
{
class ManualObject;
}  // namespace Ogre

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace trajectory_tracker_rviz_plugins
{
/**
 * \class PathWithVelocityDisplay
 * \brief Displays a trajectory_tracker_msgs::msg::PathWithVelocity message
 */
class PathWithVelocityDisplay
: public rviz_common::MessageFilterDisplay<trajectory_tracker_msgs::msg::PathWithVelocity>
{
  Q_OBJECT

public:
  PathWithVelocityDisplay();
  ~PathWithVelocityDisplay() override;

  /** @brief Overridden from Display. */
  void reset() override;

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage(trajectory_tracker_msgs::msg::PathWithVelocity::ConstSharedPtr msg) override;

protected:
  /** @brief Overridden from Display. */
  void onInitialize() override;

private Q_SLOTS:
  void updateBufferLength();
  void updateStyle();
  void updateLineWidth();
  void updateOffset();
  void updatePoseStyle();
  void updatePoseAxisGeometry();
  void updatePoseArrowColor();
  void updatePoseArrowGeometry();

private:
  using PathWithVelocityConstSharedPtr =
    trajectory_tracker_msgs::msg::PathWithVelocity::ConstSharedPtr;

  void destroyObjects();
  void allocateArrowVector(std::vector<rviz_rendering::Arrow *> & arrow_vect, size_t num);
  void allocateAxesVector(std::vector<rviz_rendering::Axes *> & axes_vect, size_t num);
  void destroyPoseAxesChain();
  void destroyPoseArrowChain();
  void updateManualObject(
    Ogre::ManualObject * manual_object, PathWithVelocityConstSharedPtr msg,
    const Ogre::Matrix4 & transform);
  void updateBillBoardLine(
    rviz_rendering::BillboardLine * billboard_line, PathWithVelocityConstSharedPtr msg,
    const Ogre::Matrix4 & transform);
  void updatePoseMarkers(
    size_t buffer_index, PathWithVelocityConstSharedPtr msg, const Ogre::Matrix4 & transform);
  void updateAxesMarkers(
    std::vector<rviz_rendering::Axes *> & axes_vect, PathWithVelocityConstSharedPtr msg,
    const Ogre::Matrix4 & transform);
  void updateArrowMarkers(
    std::vector<rviz_rendering::Arrow *> & arrow_vect, PathWithVelocityConstSharedPtr msg,
    const Ogre::Matrix4 & transform);

  std::vector<Ogre::ManualObject *> manual_objects_;
  std::vector<rviz_rendering::BillboardLine *> billboard_lines_;
  std::vector<std::vector<rviz_rendering::Axes *>> axes_chain_;
  std::vector<std::vector<rviz_rendering::Arrow *>> arrow_chain_;
  Ogre::MaterialPtr lines_material_;

  rviz_common::properties::EnumProperty * style_property_;
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * line_width_property_;
  rviz_common::properties::IntProperty * buffer_length_property_;
  rviz_common::properties::VectorProperty * offset_property_;

  enum LineStyle
  {
    LINES,
    BILLBOARDS
  };

  // pose marker property
  rviz_common::properties::EnumProperty * pose_style_property_;
  rviz_common::properties::FloatProperty * pose_axes_length_property_;
  rviz_common::properties::FloatProperty * pose_axes_radius_property_;
  rviz_common::properties::ColorProperty * pose_arrow_color_property_;
  rviz_common::properties::FloatProperty * pose_arrow_shaft_length_property_;
  rviz_common::properties::FloatProperty * pose_arrow_head_length_property_;
  rviz_common::properties::FloatProperty * pose_arrow_shaft_diameter_property_;
  rviz_common::properties::FloatProperty * pose_arrow_head_diameter_property_;

  enum PoseStyle
  {
    NONE,
    AXES,
    ARROWS,
  };
};

}  // namespace trajectory_tracker_rviz_plugins

#endif  // TRAJECTORY_TRACKER_RVIZ_PLUGINS__PATH_WITH_VELOCITY_DISPLAY_HPP_
