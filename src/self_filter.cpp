/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/** \author Ioan Sucan */

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

#include "robot_self_filter/self_see_filter.h"

namespace robot_self_filter
{

/** \brief Removes the robot's own body from a PointCloud2 stream.
 *
 * Registered as an rclcpp component so it can be loaded into a composable-node
 * container: when co-located in the same process as its upstream publisher and/or
 * downstream subscriber, messages are handed over via intra-process transport instead
 * of being serialized. The node publishes with unique_ptr ownership everywhere it can,
 * which is what lets the middleware move (rather than copy) the message when there is
 * a single intra-process subscriber.
 */
class SelfFilterNode : public rclcpp::Node
{
public:
  explicit SelfFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("self_filter", options)
  {
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "");
    use_rgb_ = declare_parameter<bool>("use_rgb", false);
    keep_original_point_type_ = declare_parameter<bool>("keep_original_point_type", false);
    max_queue_size_ = declare_parameter<int>("max_queue_size", 10);
    min_sensor_dist_ = declare_parameter<double>("min_sensor_dist", 0.01);
    default_padding_ = declare_parameter<double>("self_see_default_padding", 0.01);
    default_scale_ = declare_parameter<double>("self_see_default_scale", 1.0);
    keep_organized_ = declare_parameter<bool>("keep_organized", false);
    invert_ = declare_parameter<bool>("invert", false);
    link_names_param_ = declare_parameter<std::vector<std::string>>(
      "self_see_links", std::vector<std::string>());

    if (link_names_param_.empty())
      RCLCPP_WARN(get_logger(), "No links specified for self filtering (parameter 'self_see_links').");

    if (keep_original_point_type_ && use_rgb_)
      RCLCPP_WARN(get_logger(),
                   "'keep_original_point_type' is enabled; 'use_rgb' is ignored since the "
                   "geometry test only needs x/y/z and every other field of the input cloud "
                   "(including rgb) is passed through untouched.");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    pointCloudPublisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_out", rclcpp::SensorDataQoS().keep_last(max_queue_size_));

    // robot_description is expected to be published latched (transient local) by
    // robot_state_publisher; we only need it once to build the collision bodies.
    rclcpp::QoS description_qos(1);
    description_qos.transient_local().reliable();
    robotDescriptionSub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", description_qos,
      std::bind(&SelfFilterNode::robotDescriptionCallback, this, std::placeholders::_1));
  }

private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (self_filter_ || self_filter_rgb_)
      return;

    urdf::Model model;
    if (!model.initString(msg->data))
    {
      RCLCPP_ERROR(get_logger(), "Unable to parse URDF description!");
      return;
    }

    std::vector<robot_self_filter::LinkInfo> links;
    links.reserve(link_names_param_.size());
    for (const auto & name : link_names_param_)
    {
      robot_self_filter::LinkInfo li;
      li.name = name;
      li.padding = declare_parameter<double>("self_see_links." + name + ".padding", default_padding_);
      li.scale = declare_parameter<double>("self_see_links." + name + ".scale", default_scale_);
      links.push_back(li);
    }

    if (keep_original_point_type_)
    {
      // Geometry test only ever needs x/y/z, regardless of what point type the input
      // cloud actually carries -- so a plain PointXYZ filter is enough here.
      self_filter_ = std::make_unique<robot_self_filter::SelfFilter<pcl::PointXYZ>>(
        *tf_buffer_, model, links, invert_, keep_organized_, min_sensor_dist_, get_logger());
      self_filter_->getSelfMask()->getLinkNames(frames_);
    }
    else if (use_rgb_)
    {
      self_filter_rgb_ = std::make_unique<robot_self_filter::SelfFilter<pcl::PointXYZRGB>>(
        *tf_buffer_, model, links, invert_, keep_organized_, min_sensor_dist_, get_logger());
      self_filter_rgb_->getSelfMask()->getLinkNames(frames_);
    }
    else
    {
      self_filter_ = std::make_unique<robot_self_filter::SelfFilter<pcl::PointXYZ>>(
        *tf_buffer_, model, links, invert_, keep_organized_, min_sensor_dist_, get_logger());
      self_filter_->getSelfMask()->getLinkNames(frames_);
    }

    if (!sensor_frame_.empty())
      RCLCPP_INFO(get_logger(),
                  "Self filter is removing shadow points for sensor in frame '%s'. Minimum distance to sensor is %f.",
                  sensor_frame_.c_str(), min_sensor_dist_);

    subscribeToCloud();
  }

  void subscribeToCloud()
  {
    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    qos.depth = static_cast<size_t>(max_queue_size_);

    if (frames_.empty())
    {
      RCLCPP_DEBUG(get_logger(),
                   "No valid frames have been passed into the self filter. Using a callback that will just forward scans on.");
      noFilterSub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_in", rclcpp::SensorDataQoS().keep_last(max_queue_size_),
        std::bind(&SelfFilterNode::noFilterCallback, this, std::placeholders::_1));
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "Valid frames were passed in. We'll filter them.");
      cloudSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
      cloudSub_->subscribe(shared_from_this(), "cloud_in", qos);
      tfFilter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *cloudSub_, *tf_buffer_, "", max_queue_size_, shared_from_this());
      tfFilter_->setTargetFrames(frames_);
      tfFilter_->registerCallback(std::bind(&SelfFilterNode::cloudCallback, this, std::placeholders::_1));
    }
  }

  void noFilterCallback(sensor_msgs::msg::PointCloud2::UniquePtr cloud)
  {
    RCLCPP_DEBUG(get_logger(), "Self filter publishing unfiltered frame");
    // No self_see_links configured: forward the message as-is. Taking/publishing it by
    // UniquePtr lets intra-process transport move it instead of copying it.
    pointCloudPublisher_->publish(std::move(cloud));
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud2)
  {
    // tf2_ros::MessageFilter hands us a ConstSharedPtr -- it needs shared ownership to
    // buffer messages while waiting on tf, so that part can't be UniquePtr. The output
    // we build ourselves, though, so it is published by UniquePtr below.
    const rclcpp::Time start = now();

    sensor_msgs::msg::PointCloud2::UniquePtr out2;
    size_t input_size = cloud2->height*cloud2->width, output_size = 0;
    if (input_size == 0){
      RCLCPP_WARN(get_logger(), "empty cloud, not filtering");
      pointCloudPublisher_->publish(*cloud2);
      return;
    }
    if (keep_original_point_type_)
    {
      // Only x/y/z are needed for the geometry test; every other byte of the original
      // message (whatever point type it is) is carried through untouched.
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*cloud2, cloud);
      std::vector<int> mask;
      self_filter_->computeMask(cloud, mask, sensor_frame_);
      out2 = robot_self_filter::filterKeepingPointType(*cloud2, mask, invert_, keep_organized_);
      input_size = cloud.points.size();
      output_size = static_cast<size_t>(out2->width) * out2->height;
    }
    else if (use_rgb_)
    {
      pcl::PointCloud<pcl::PointXYZRGB> cloud, out;
      pcl::fromROSMsg(*cloud2, cloud);
      self_filter_rgb_->updateWithSensorFrame(cloud, out, sensor_frame_);
      out2 = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(out, *out2);
      input_size = cloud.points.size();
      output_size = out.points.size();
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZ> cloud, out;
      pcl::fromROSMsg(*cloud2, cloud);
      self_filter_->updateWithSensorFrame(cloud, out, sensor_frame_);
      out2 = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(out, *out2);
      input_size = cloud.points.size();
      output_size = out.points.size();
    }
    out2->header = cloud2->header;

    const double sec = (now() - start).seconds();
    RCLCPP_DEBUG(get_logger(), "Self filter: reduced %zu points to %zu points in %f seconds",
                 input_size, output_size, sec);
    pointCloudPublisher_->publish(std::move(out2));
  }

  // parameters
  std::string sensor_frame_;
  bool use_rgb_ = false;
  bool keep_original_point_type_ = false;
  int max_queue_size_ = 10;
  double min_sensor_dist_ = 0.01;
  double default_padding_ = 0.01;
  double default_scale_ = 1.0;
  bool keep_organized_ = false;
  bool invert_ = false;
  std::vector<std::string> link_names_param_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<robot_self_filter::SelfFilter<pcl::PointXYZ>> self_filter_;
  std::unique_ptr<robot_self_filter::SelfFilter<pcl::PointXYZRGB>> self_filter_rgb_;
  std::vector<std::string> frames_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr noFilterSub_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloudSub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tfFilter_;
};

}  // namespace robot_self_filter

RCLCPP_COMPONENTS_REGISTER_NODE(robot_self_filter::SelfFilterNode)
