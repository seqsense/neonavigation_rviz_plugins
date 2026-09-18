/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2022, the neonavigation authors
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

// This file is based on rviz_default_plugins' map display.

#ifndef COSTMAP_CSPACE_RVIZ_PLUGINS__C_SPACE_3D_DISPLAY_HPP_
#define COSTMAP_CSPACE_RVIZ_PLUGINS__C_SPACE_3D_DISPLAY_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#ifndef Q_MOC_RUN

#include <OgreSharedPtr.h>
#include <OgreTexture.h>

#endif  // Q_MOC_RUN

#include "costmap_cspace_msgs/msg/c_space3_d.hpp"
#include "costmap_cspace_msgs/msg/c_space3_d_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/qos.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "rviz_default_plugins/displays/map/swatch.hpp"

namespace costmap_cspace_rviz_plugins
{
/**
 * \class CSpace3DDisplay
 * \brief Displays one yaw layer of a costmap_cspace_msgs::msg::CSpace3D along the XY plane.
 */
class CSpace3DDisplay : public rviz_common::RosTopicDisplay<costmap_cspace_msgs::msg::CSpace3D>
{
  Q_OBJECT

public:
  CSpace3DDisplay();
  ~CSpace3DDisplay() override;

  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;

  /** @brief Copy msg into current_map_ and call showMap(). */
  void processMessage(costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr msg) override;

public Q_SLOTS:
  /** @brief Show current_map_ in the scene. */
  void showMap();

Q_SIGNALS:
  /** @brief Emitted when a new map is received */
  void mapUpdated();

protected Q_SLOTS:
  void updateAlpha();
  void updateDrawUnder() const;
  void updatePalette();
  void updateYaw();
  void transformMap();
  void updateMapUpdateTopic();

protected:
  void subscribe() override;
  void unsubscribe() override;
  void onEnable() override;
  void update(float wall_dt, float ros_dt) override;

  void subscribeToUpdateTopic();
  void unsubscribeToUpdateTopic();

  /** @brief Copy update into current_update_ and call showMap(). */
  void incomingUpdate(costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr update);
  bool updateDataOutOfBounds(const costmap_cspace_msgs::msg::CSpace3DUpdate & update) const;

  /** @brief Flatten one yaw layer of current_map_, with current_update_ applied, so that
   * rviz_default_plugins' Swatch can render it. */
  nav_msgs::msg::OccupancyGrid buildYawSlice(size_t yaw) const;
  void applyUpdate(size_t yaw, nav_msgs::msg::OccupancyGrid & grid) const;

  void clear();
  void showValidMap();
  void resetSwatchesIfNecessary(size_t width, size_t height, float resolution);
  void createSwatches();
  void tryCreateSwatches(
    size_t width, size_t height, float resolution, size_t swatch_width, size_t swatch_height,
    int number_swatches);
  void doubleSwatchNumber(
    size_t & swatch_width, size_t & swatch_height, int & number_swatches) const;
  static size_t getEffectiveDimension(
    size_t map_dimension, size_t swatch_dimension, size_t position);
  void updateSwatches() const;

  std::vector<std::shared_ptr<rviz_default_plugins::displays::Swatch>> swatches_;
  std::vector<Ogre::TexturePtr> palette_textures_;
  std::vector<bool> color_scheme_transparency_;
  bool loaded_;

  float resolution_;
  size_t width_;
  size_t height_;
  std::string frame_;
  costmap_cspace_msgs::msg::CSpace3D current_map_;
  costmap_cspace_msgs::msg::CSpace3DUpdate current_update_;

  rclcpp::Subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>::SharedPtr update_subscription_;
  rclcpp::QoS update_profile_;

  rviz_common::properties::RosTopicProperty * update_topic_property_;
  rviz_common::properties::QosProfileProperty * update_profile_property_;
  rviz_common::properties::FloatProperty * resolution_property_;
  rviz_common::properties::FloatProperty * angular_resolution_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::Property * draw_under_property_;
  rviz_common::properties::EnumProperty * color_scheme_property_;
  rviz_common::properties::IntProperty * yaw_property_;
  rviz_common::properties::BoolProperty * transform_timestamp_property_;
};

}  // namespace costmap_cspace_rviz_plugins

#endif  // COSTMAP_CSPACE_RVIZ_PLUGINS__C_SPACE_3D_DISPLAY_HPP_
