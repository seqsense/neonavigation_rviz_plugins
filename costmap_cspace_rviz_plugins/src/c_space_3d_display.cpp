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

#include "costmap_cspace_rviz_plugins/c_space_3d_display.hpp"

#include <OgrePass.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>

#include <QString>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_default_plugins/displays/map/palette_builder.hpp"
#include "rviz_rendering/material_manager.hpp"

namespace costmap_cspace_rviz_plugins
{
using rviz_common::properties::StatusProperty;
using rviz_default_plugins::displays::Swatch;

namespace
{
Ogre::TexturePtr makePaletteTexture(std::vector<unsigned char> palette_bytes)
{
  Ogre::DataStreamPtr palette_stream;
  palette_stream.reset(new Ogre::MemoryDataStream(palette_bytes.data(), 256 * 4));

  static int palette_tex_count = 0;
  const std::string tex_name = "CSpace3DPaletteTexture" + std::to_string(palette_tex_count++);
  // "rviz_rendering" is the group the Swatch materials and textures live in.
  return Ogre::TextureManager::getSingleton().loadRawData(
    tex_name, "rviz_rendering", palette_stream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0);
}

bool validateFloats(const costmap_cspace_msgs::msg::CSpace3D & msg)
{
  return rviz_common::validateFloats(msg.info.linear_resolution) &&
         rviz_common::validateFloats(msg.info.origin);
}
}  // namespace

CSpace3DDisplay::CSpace3DDisplay()
: loaded_(false), resolution_(0.0f), width_(0), height_(0), update_profile_(rclcpp::QoS(5))
{
  connect(this, SIGNAL(mapUpdated()), this, SLOT(showMap()));

  update_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Update Topic", "", rosidl_generator_traits::name<costmap_cspace_msgs::msg::CSpace3DUpdate>(),
    "Topic where updates to this map display are received.", this, SLOT(updateMapUpdateTopic()));

  update_profile_property_ =
    new rviz_common::properties::QosProfileProperty(update_topic_property_, update_profile_);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.7f, "Amount of transparency to apply to the map.", this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  color_scheme_property_ = new rviz_common::properties::EnumProperty(
    "Color Scheme", "costmap", "How to color the occupancy values.", this, SLOT(updatePalette()));
  // Option values here must correspond to indices in palette_textures_ array in onInitialize()
  // below.
  color_scheme_property_->addOption("costmap", 0);
  color_scheme_property_->addOption("raw", 1);

  yaw_property_ = new rviz_common::properties::IntProperty(
    "Yaw", 0, "Yaw layer of the configuration space to draw.", this, SLOT(updateYaw()));
  yaw_property_->setMin(0);

  draw_under_property_ = new rviz_common::properties::Property(
    "Draw Behind", false,
    "Rendering option, controls whether or not the map is always drawn behind everything else.",
    this, SLOT(updateDrawUnder()));

  resolution_property_ = new rviz_common::properties::FloatProperty(
    "Resolution", 0, "Resolution of the map. (not editable)", this);
  resolution_property_->setReadOnly(true);

  angular_resolution_property_ = new rviz_common::properties::FloatProperty(
    "Angle Resolution", 0, "Angle resolution of the map. (not editable)", this);
  angular_resolution_property_->setReadOnly(true);

  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 0, "Width of the map, in meters. (not editable)", this);
  width_property_->setReadOnly(true);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 0, "Height of the map, in meters. (not editable)", this);
  height_property_->setReadOnly(true);

  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", Ogre::Vector3::ZERO,
    "Position of the bottom left corner of the map, in meters. (not editable)", this);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY, "Orientation of the map. (not editable)", this);
  orientation_property_->setReadOnly(true);

  transform_timestamp_property_ = new rviz_common::properties::BoolProperty(
    "Use Timestamp", false, "Use map header timestamp when transforming", this,
    SLOT(transformMap()));
}

CSpace3DDisplay::~CSpace3DDisplay() { unsubscribe(); }

void CSpace3DDisplay::onInitialize()
{
  RTDClass::onInitialize();

  update_topic_property_->initialize(rviz_ros_node_);
  update_profile_property_->initialize([this](rclcpp::QoS profile) {
    this->update_profile_ = profile;
    updateMapUpdateTopic();
  });

  // Order of palette textures here must match option indices for color_scheme_property_ above.
  palette_textures_.push_back(
    makePaletteTexture(rviz_default_plugins::displays::makeCostmapPalette()));
  color_scheme_transparency_.push_back(true);
  palette_textures_.push_back(makePaletteTexture(rviz_default_plugins::displays::makeRawPalette()));
  color_scheme_transparency_.push_back(true);
}

void CSpace3DDisplay::subscribe()
{
  RTDClass::subscribe();
  subscribeToUpdateTopic();
}

void CSpace3DDisplay::unsubscribe()
{
  RTDClass::unsubscribe();
  unsubscribeToUpdateTopic();
}

void CSpace3DDisplay::subscribeToUpdateTopic()
{
  // Editing the update topic or its QoS calls this directly, not through subscribe().
  if (!isEnabled()) {
    return;
  }
  if (update_topic_property_->isEmpty()) {
    setStatus(StatusProperty::Ok, "Update Topic", "Not specified");
    return;
  }

  try {
    update_subscription_ =
      rviz_ros_node_.lock()
        ->get_raw_node()
        ->create_subscription<costmap_cspace_msgs::msg::CSpace3DUpdate>(
          update_topic_property_->getTopicStd(), update_profile_,
          [this](const costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr message) {
            incomingUpdate(message);
          });
    setStatus(StatusProperty::Ok, "Update Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(StatusProperty::Error, "Update Topic", QString("Error subscribing: ") + e.what());
  }
}

void CSpace3DDisplay::unsubscribeToUpdateTopic() { update_subscription_.reset(); }

void CSpace3DDisplay::updateMapUpdateTopic()
{
  unsubscribeToUpdateTopic();
  reset();
  subscribeToUpdateTopic();
  context_->queueRender();
}

void CSpace3DDisplay::onEnable()
{
  RTDClass::onEnable();
  setStatus(StatusProperty::Warn, "Message", "No map received");
}

void CSpace3DDisplay::processMessage(costmap_cspace_msgs::msg::CSpace3D::ConstSharedPtr msg)
{
  current_map_ = *msg;
  loaded_ = true;
  // updated via signal in case the ros spinner is in a different thread
  Q_EMIT mapUpdated();
}

bool CSpace3DDisplay::updateDataOutOfBounds(
  const costmap_cspace_msgs::msg::CSpace3DUpdate & update) const
{
  // Widened so that an offset near UINT32_MAX cannot wrap past the check.
  return uint64_t{update.x} + update.width > current_map_.info.width ||
         uint64_t{update.y} + update.height > current_map_.info.height ||
         uint64_t{update.yaw} + update.angle > current_map_.info.angle ||
         update.data.size() != static_cast<size_t>(update.width) * update.height * update.angle;
}

void CSpace3DDisplay::incomingUpdate(
  costmap_cspace_msgs::msg::CSpace3DUpdate::ConstSharedPtr update)
{
  // Only update the map if we have gotten a full one first.
  if (!loaded_) {
    return;
  }

  if (updateDataOutOfBounds(*update)) {
    setStatus(StatusProperty::Error, "Update", "Update area outside of original map area.");
    return;
  }
  setStatus(StatusProperty::Ok, "Update", "Update OK");

  current_update_ = *update;
  // updated via signal in case the ros spinner is in a different thread
  Q_EMIT mapUpdated();
}

nav_msgs::msg::OccupancyGrid CSpace3DDisplay::buildYawSlice(size_t yaw) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = current_map_.header;
  grid.info.resolution = current_map_.info.linear_resolution;
  grid.info.width = current_map_.info.width;
  grid.info.height = current_map_.info.height;
  grid.info.origin = current_map_.info.origin;

  const size_t layer_size = static_cast<size_t>(grid.info.width) * grid.info.height;
  const size_t offset = layer_size * yaw;
  if (current_map_.data.size() < offset + layer_size) {
    grid.data.assign(layer_size, -1);
    return grid;
  }
  grid.data.assign(
    current_map_.data.begin() + offset, current_map_.data.begin() + offset + layer_size);

  applyUpdate(yaw, grid);
  return grid;
}

void CSpace3DDisplay::applyUpdate(size_t yaw, nav_msgs::msg::OccupancyGrid & grid) const
{
  const costmap_cspace_msgs::msg::CSpace3DUpdate & update = current_update_;
  if (update.width == 0 || update.height == 0) {
    return;
  }
  // An update carries the layers [yaw, yaw + angle) of the map, in that order; layers outside
  // that range keep the values of the last full map.
  if (yaw < update.yaw || yaw >= static_cast<size_t>(update.yaw) + update.angle) {
    return;
  }
  if (
    uint64_t{update.x} + update.width > grid.info.width ||
    uint64_t{update.y} + update.height > grid.info.height) {
    return;
  }

  const size_t layer_size = static_cast<size_t>(update.width) * update.height;
  const size_t offset = layer_size * (yaw - update.yaw);
  if (update.data.size() < offset + layer_size) {
    return;
  }

  for (size_t row = 0; row < update.height; ++row) {
    const auto from = update.data.begin() + offset + row * update.width;
    const auto to = grid.data.begin() + (update.y + row) * grid.info.width + update.x;
    std::transform(from, from + update.width, to, to, [](int8_t updated, int8_t current) {
      return std::max(updated, current);
    });
  }
}

void CSpace3DDisplay::updateAlpha()
{
  const float alpha = alpha_property_->getFloat();
  Ogre::SceneBlendType scene_blending;
  bool depth_write;

  if (
    alpha < rviz_rendering::unit_alpha_threshold ||
    color_scheme_transparency_[color_scheme_property_->getOptionInt()]) {
    scene_blending = Ogre::SBT_TRANSPARENT_ALPHA;
    depth_write = false;
  } else {
    scene_blending = Ogre::SBT_REPLACE;
    depth_write = !draw_under_property_->getValue().toBool();
  }

  for (const auto & swatch : swatches_) {
    swatch->updateAlpha(scene_blending, depth_write, alpha);
  }
}

void CSpace3DDisplay::updateDrawUnder() const
{
  const bool draw_under = draw_under_property_->getValue().toBool();

  if (alpha_property_->getFloat() >= rviz_rendering::unit_alpha_threshold) {
    for (const auto & swatch : swatches_) {
      swatch->setDepthWriteEnabled(!draw_under);
    }
  }

  const uint8_t group = draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;
  for (const auto & swatch : swatches_) {
    swatch->setRenderQueueGroup(group);
  }
}

void CSpace3DDisplay::updateYaw()
{
  if (!loaded_) {
    return;
  }
  updateSwatches();
  context_->queueRender();
}

void CSpace3DDisplay::clear()
{
  if (isEnabled()) {
    setStatus(StatusProperty::Warn, "Message", "No map received");
  }

  if (!loaded_) {
    return;
  }

  swatches_.clear();
  width_ = 0;
  height_ = 0;
  resolution_ = 0.0f;

  loaded_ = false;
}

void CSpace3DDisplay::showMap()
{
  if (current_map_.data.empty()) {
    return;
  }

  if (!validateFloats(current_map_)) {
    setStatus(
      StatusProperty::Error, "Map",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  const size_t width = current_map_.info.width;
  const size_t height = current_map_.info.height;
  const size_t angle = current_map_.info.angle;

  if (width * height * angle == 0) {
    setStatus(
      StatusProperty::Error, "Map",
      QString("Map is zero-sized (%1x%2x%3)").arg(width).arg(height).arg(angle));
    return;
  }

  if (width * height * angle != current_map_.data.size()) {
    setStatus(
      StatusProperty::Error, "Map",
      QString("Data size doesn't match width*height*angle: width = %1, height = %2, angle = %3, "
              "data size = %4")
        .arg(width)
        .arg(height)
        .arg(angle)
        .arg(current_map_.data.size()));
    return;
  }

  setStatus(StatusProperty::Ok, "Message", "Map received");

  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "Received a " << width << " X " << height << " X " << angle << " map @ "
                  << current_map_.info.linear_resolution << "m/pix\n");

  showValidMap();
}

void CSpace3DDisplay::showValidMap()
{
  const size_t width = current_map_.info.width;
  const size_t height = current_map_.info.height;
  const float resolution = current_map_.info.linear_resolution;

  // setMax() clamps the current value, so it has to run before the yaw is read back.
  yaw_property_->setMax(static_cast<int>(current_map_.info.angle) - 1);

  resetSwatchesIfNecessary(width, height, resolution);

  frame_ = current_map_.header.frame_id;
  if (frame_.empty()) {
    frame_ = "map";
  }

  updateSwatches();

  setStatus(StatusProperty::Ok, "Map", "Map OK");
  updatePalette();

  resolution_property_->setValue(resolution);
  angular_resolution_property_->setValue(current_map_.info.angular_resolution);
  width_property_->setValue(static_cast<unsigned int>(width));
  height_property_->setValue(static_cast<unsigned int>(height));

  position_property_->setVector(rviz_common::pointMsgToOgre(current_map_.info.origin.position));
  orientation_property_->setQuaternion(
    rviz_common::quaternionMsgToOgre(current_map_.info.origin.orientation));

  transformMap();

  updateDrawUnder();

  context_->queueRender();
}

void CSpace3DDisplay::resetSwatchesIfNecessary(size_t width, size_t height, float resolution)
{
  if (width != width_ || height != height_ || resolution_ != resolution) {
    createSwatches();
    width_ = width;
    height_ = height;
    resolution_ = resolution;
  }
}

void CSpace3DDisplay::createSwatches()
{
  const size_t width = current_map_.info.width;
  const size_t height = current_map_.info.height;
  const float resolution = current_map_.info.linear_resolution;

  size_t swatch_width = width;
  size_t swatch_height = height;
  int number_swatches = 1;
  // One swatch can have up to 2^16 * 2^16 pixel (8 bit texture, i.e. 4GB of data)
  // Since the width and height are separately limited by 2^16 it might be necessary to have several
  // pieces, however more than 8 swatches is probably unnecessary due to memory limitations
  const size_t maximum_number_swatch_splittings = 4;

  for (size_t i = 0; i < maximum_number_swatch_splittings; ++i) {
    swatches_.clear();
    try {
      // OpenGL size limit is 16384x16384
      if (swatch_width > 16384 || swatch_height > 16384) {
        doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
        continue;
      }
      tryCreateSwatches(width, height, resolution, swatch_width, swatch_height, number_swatches);
      updateDrawUnder();
      return;
    } catch (Ogre::InvalidParametersException &) {
      doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
    } catch (Ogre::RenderingAPIException &) {
      // This exception seems no longer thrown on some systems. May still be relevant for others.
      doubleSwatchNumber(swatch_width, swatch_height, number_swatches);
    }
  }
  RVIZ_COMMON_LOG_ERROR_STREAM(
    "Creating " << number_swatches
                << " swatches failed. "
                   "This map is too large to be displayed by RViz.");
  swatches_.clear();
}

void CSpace3DDisplay::doubleSwatchNumber(
  size_t & swatch_width, size_t & swatch_height, int & number_swatches) const
{
  RVIZ_COMMON_LOG_ERROR_STREAM(
    "Failed to create map using " << number_swatches
                                  << " swatches. "
                                     "At least one swatch seems to need too much memory");
  if (swatch_width > swatch_height) {
    swatch_width /= 2;
  } else {
    swatch_height /= 2;
  }
  number_swatches *= 2;
}

void CSpace3DDisplay::tryCreateSwatches(
  size_t width, size_t height, float resolution, size_t swatch_width, size_t swatch_height,
  int number_swatches)
{
  const nav_msgs::msg::OccupancyGrid slice =
    buildYawSlice(static_cast<size_t>(yaw_property_->getInt()));
  const bool draw_under = draw_under_property_->getValue().toBool();

  size_t x = 0;
  size_t y = 0;
  for (int i = 0; i < number_swatches; i++) {
    const size_t effective_width = getEffectiveDimension(width, swatch_width, x);
    const size_t effective_height = getEffectiveDimension(height, swatch_height, y);

    swatches_.push_back(std::make_shared<Swatch>(
      scene_manager_, scene_node_, x, y, effective_width, effective_height, resolution,
      draw_under));
    swatches_[i]->updateData(slice);

    x += effective_width;
    if (x >= width) {
      x = 0;
      y += effective_height;
    }
  }
  updateAlpha();
}

size_t CSpace3DDisplay::getEffectiveDimension(
  size_t map_dimension, size_t swatch_dimension, size_t position)
{
  // Last swatch is bigger than swatch_dimension for odd numbers.
  // subtracting the swatch_dimension in the LHS handles this case.
  return map_dimension - position - swatch_dimension >= swatch_dimension ? swatch_dimension
                                                                         : map_dimension - position;
}

void CSpace3DDisplay::updateSwatches() const
{
  const nav_msgs::msg::OccupancyGrid slice =
    buildYawSlice(static_cast<size_t>(yaw_property_->getInt()));

  for (const auto & swatch : swatches_) {
    swatch->updateData(slice);

    Ogre::Pass * pass = swatch->getTechniquePass();
    Ogre::TextureUnitState * tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 0) {
      tex_unit = pass->getTextureUnitState(0);
    } else {
      tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setTextureName(swatch->getTextureName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);
    swatch->setVisible(true);
    swatch->resetOldTexture();
  }
}

void CSpace3DDisplay::updatePalette()
{
  const int palette_index = color_scheme_property_->getOptionInt();

  for (const auto & swatch : swatches_) {
    Ogre::Pass * pass = swatch->getTechniquePass();
    Ogre::TextureUnitState * palette_tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 1) {
      palette_tex_unit = pass->getTextureUnitState(1);
    } else {
      palette_tex_unit = pass->createTextureUnitState();
    }
    palette_tex_unit->setTexture(palette_textures_[palette_index]);
    palette_tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  }

  updateAlpha();
  updateDrawUnder();
}

void CSpace3DDisplay::transformMap()
{
  if (!loaded_) {
    return;
  }

  rclcpp::Time transform_time = context_->getClock()->now();

  if (transform_timestamp_property_->getBool()) {
    transform_time = rclcpp::Time(current_map_.header.stamp, RCL_ROS_TIME);
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (
    !context_->getFrameManager()->transform(
      frame_, transform_time, current_map_.info.origin, position, orientation) &&
    !context_->getFrameManager()->transform(
      frame_, rclcpp::Time(0, 0, context_->getClock()->get_clock_type()), current_map_.info.origin,
      position, orientation)) {
    setMissingTransformToFixedFrame(frame_);
    scene_node_->setVisible(false);
  } else {
    setTransformOk();

    scene_node_->setVisible(true);
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
  }
}

void CSpace3DDisplay::fixedFrameChanged() { transformMap(); }

void CSpace3DDisplay::reset()
{
  RTDClass::reset();
  clear();
  current_map_ = costmap_cspace_msgs::msg::CSpace3D();
  current_update_ = costmap_cspace_msgs::msg::CSpace3DUpdate();
}

void CSpace3DDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  transformMap();
}

}  // namespace costmap_cspace_rviz_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(costmap_cspace_rviz_plugins::CSpace3DDisplay, rviz_common::Display)
