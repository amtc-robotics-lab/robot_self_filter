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

#ifndef FILTERS_SELF_SEE_H_
#define FILTERS_SELF_SEE_H_

#include <robot_self_filter/self_mask.h>
#include <cstring>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robot_self_filter
{

/** \brief A filter to remove parts of the robot seen in a pointcloud
 *
 * This is a plain helper class (not a pluginlib filter or a node) that a
 * node owns and drives: it wraps a SelfMask<PointT> and turns its per-point
 * mask into filtered output clouds.
 */
template <typename PointT>
class SelfFilter
{
public:
  typedef pcl::PointCloud<PointT> PointCloud;

  /** \brief Construct the filter.
   * \param tf_buffer   tf2 buffer used to look up link transforms; owned by the caller.
   * \param urdf_model  parsed robot description used to build collision bodies for `links`.
   * \param links       links to check for self-see, with per-link padding/scale.
   * \param invert      if true, keep only the points that are on the robot instead of removing them.
   * \param keep_organized  if true, removed points are replaced with NaNs instead of being dropped,
   *                        preserving the width/height of an organized cloud.
   * \param min_sensor_dist minimum distance from the sensor origin at which a point can be trusted.
   */
  SelfFilter(tf2_ros::Buffer &tf_buffer, const urdf::Model &urdf_model, const std::vector<LinkInfo> &links,
             bool invert = false, bool keep_organized = false, double min_sensor_dist = 0.01,
             const rclcpp::Logger &logger = rclcpp::get_logger("robot_self_filter"))
    : sm_(tf_buffer, urdf_model, links, logger),
      invert_(invert),
      min_sensor_dist_(min_sensor_dist),
      keep_organized_(keep_organized),
      logger_(logger)
  {
    if (invert_)
      RCLCPP_INFO(logger_, "Inverting filter output");
  }

  bool updateWithSensorFrame(const PointCloud& data_in, PointCloud& data_out, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out);
  }

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  bool update(const PointCloud& data_in, PointCloud& data_out)
  {
    std::vector<int> keep(data_in.points.size());
    computeMask(data_in, keep, sensor_frame_);
    fillResult(data_in, keep, data_out);
    return true;
  }

  /** \brief Compute the per-point mask only (INSIDE/OUTSIDE/SHADOW), without building an
   *  output cloud. Useful when the caller wants to apply the mask itself, e.g. directly on
   *  the raw bytes of a sensor_msgs::msg::PointCloud2 to preserve its original point type.
   *  \param sensor_frame if non-empty, shadow points are computed via ray intersection from
   *         this frame; otherwise a plain containment test is performed.
   */
  void computeMask(const PointCloud& data_in, std::vector<int> &mask, const std::string& sensor_frame = "")
  {
    mask.resize(data_in.points.size());
    if (sensor_frame.empty()) {
      sm_.maskContainment(data_in, mask);
    } else {
      sm_.maskIntersection(data_in, sensor_frame, min_sensor_dist_, mask);
    }
  }

  bool updateWithSensorFrame(const PointCloud& data_in, PointCloud& data_out, PointCloud& data_diff,
                              const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out, data_diff);
  }

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  bool update(const PointCloud& data_in, PointCloud& data_out, PointCloud& data_diff)
  {
    std::vector<int> keep(data_in.points.size());
    if(sensor_frame_.empty()) {
      sm_.maskContainment(data_in, keep);
    } else {
      sm_.maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    }
    fillResult(data_in, keep, data_out);
    fillDiff(data_in, keep, data_diff);
    return true;
  }

  void fillDiff(const PointCloud& data_in, const std::vector<int> &keep, PointCloud& data_out)
  {
    const unsigned int np = data_in.points.size();

    // fill in output data
    data_out.header = data_in.header;

    data_out.points.resize(0);
    data_out.points.reserve(np);

    for (unsigned int i = 0 ; i < np ; ++i)
    {
      if ((keep[i] && invert_) || (!keep[i] && !invert_))
      {
        data_out.points.push_back(data_in.points[i]);
      }
    }
  }

  void fillResult(const PointCloud& data_in, const std::vector<int> &keep, PointCloud& data_out)
  {
    const unsigned int np = data_in.points.size();

    // fill in output data with points that are NOT on the robot
    data_out.header = data_in.header;

    data_out.points.resize(0);
    data_out.points.reserve(np);
    PointT nan_point;
    nan_point.x = std::numeric_limits<float>::quiet_NaN();
    nan_point.y = std::numeric_limits<float>::quiet_NaN();
    nan_point.z = std::numeric_limits<float>::quiet_NaN();
    for (unsigned int i = 0 ; i < np ; ++i)
    {
      const bool onRobot = (keep[i] == robot_self_filter::INSIDE);
      const bool keepPoint = invert_ ? onRobot : !onRobot;
      if (keepPoint)
      {
        data_out.points.push_back(data_in.points[i]);
      }
      else if (keep_organized_)
      {
        data_out.points.push_back(nan_point);
      }
    }
    if (keep_organized_) {
      data_out.width = data_in.width;
      data_out.height = data_in.height;
    }
    else
    {
      data_out.width = data_out.points.size();
      data_out.height = 1;
    }
  }

  bool updateWithSensorFrame(const std::vector<PointCloud> & data_in, std::vector<PointCloud>& data_out,
                              const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out);
  }

  bool update(const std::vector<PointCloud> & data_in, std::vector<PointCloud>& data_out)
  {
    bool result = true;
    data_out.resize(data_in.size());
    for (unsigned int i = 0 ; i < data_in.size() ; ++i)
      if (!update(data_in[i], data_out[i]))
        result = false;
    return result;
  }

  robot_self_filter::SelfMask<PointT>* getSelfMask() {
    return &sm_;
  }

  void setSensorFrame(const std::string& frame) {
    sensor_frame_ = frame;
  }

protected:

  robot_self_filter::SelfMask<PointT> sm_;
  bool invert_;
  std::string sensor_frame_;
  double min_sensor_dist_;
  bool keep_organized_;
  rclcpp::Logger logger_;

};

/** \brief Build a filtered PointCloud2 that preserves the wire format of the input cloud
 *  (its fields, point_step, byte layout) instead of reconstructing points through a fixed
 *  PCL point type. Only the x/y/z bytes of a removed point are ever touched (set to NaN,
 *  when `keep_organized` is true); every other byte -- intensity, ring, rgb, custom fields,
 *  whatever the driver put there -- is copied through untouched. This is what lets the
 *  self filter run on clouds whose point type it doesn't know about at compile time.
 *
 *  \param input           original cloud, as received
 *  \param mask            per-point mask computed by SelfFilter::computeMask() /
 *                         SelfMask::maskContainment() / SelfMask::maskIntersection()
 *  \param invert          if true, keep only the points that are on the robot
 *  \param keep_organized  if true, removed points are NaN'd in place instead of dropped,
 *                         preserving width/height/row_step
 */
inline sensor_msgs::msg::PointCloud2::UniquePtr filterKeepingPointType(
    const sensor_msgs::msg::PointCloud2 & input,
    const std::vector<int> & mask,
    bool invert,
    bool keep_organized)
{
  auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
  output->header = input.header;
  output->fields = input.fields;
  output->is_bigendian = input.is_bigendian;
  output->point_step = input.point_step;

  int x_offset = -1, y_offset = -1, z_offset = -1;
  for (const auto & field : input.fields)
  {
    if (field.name == "x") { x_offset = static_cast<int>(field.offset); }
    else if (field.name == "y") { y_offset = static_cast<int>(field.offset); }
    else if (field.name == "z") { z_offset = static_cast<int>(field.offset); }
  }

  const uint32_t width = input.width;
  const uint32_t height = input.height;
  const uint32_t point_step = input.point_step;
  const uint32_t row_step = input.row_step;
  constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

  const auto keepPoint = [invert](int m) {
    const bool onRobot = (m == robot_self_filter::INSIDE);
    return invert ? onRobot : !onRobot;
  };

  if (keep_organized)
  {
    output->data = input.data;
    for (uint32_t row = 0; row < height; ++row)
    {
      for (uint32_t col = 0; col < width; ++col)
      {
        const size_t i = static_cast<size_t>(row) * width + col;
        if (i < mask.size() && !keepPoint(mask[i]))
        {
          uint8_t * p = &output->data[row * row_step + col * point_step];
          if (x_offset >= 0) { std::memcpy(p + x_offset, &kNan, sizeof(float)); }
          if (y_offset >= 0) { std::memcpy(p + y_offset, &kNan, sizeof(float)); }
          if (z_offset >= 0) { std::memcpy(p + z_offset, &kNan, sizeof(float)); }
        }
      }
    }
    output->width = width;
    output->height = height;
    output->row_step = row_step;
    output->is_dense = false;
  }
  else
  {
    output->data.reserve(input.data.size());
    uint32_t kept = 0;
    for (uint32_t row = 0; row < height; ++row)
    {
      for (uint32_t col = 0; col < width; ++col)
      {
        const size_t i = static_cast<size_t>(row) * width + col;
        if (i < mask.size() && keepPoint(mask[i]))
        {
          const uint8_t * src = &input.data[row * row_step + col * point_step];
          output->data.insert(output->data.end(), src, src + point_step);
          ++kept;
        }
      }
    }
    output->width = kept;
    output->height = 1;
    output->row_step = point_step * kept;
    output->is_dense = input.is_dense;
  }

  return output;
}

}

#endif //#ifndef FILTERS_SELF_SEE_H_
