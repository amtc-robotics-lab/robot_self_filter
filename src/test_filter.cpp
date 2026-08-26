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

// Manual test / benchmark utility: builds a SelfMask for a single link
// ("base_link"), throws random points at it and publishes markers for the
// points classified as inside the robot (and for shadow-ray intersections),
// so the result can be checked visually in rviz.

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>

#include "robot_self_filter/self_mask.h"

using namespace std::chrono_literals;

class TestSelfFilter : public rclcpp::Node
{
public:
  TestSelfFilter()
  : rclcpp::Node("test_self_filter"), id_(1)
  {
    vmPub_ = create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10240);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS description_qos(1);
    description_qos.transient_local().reliable();
    robotDescriptionSub_ = create_subscription<std_msgs::msg::String>(
      "robot_description", description_qos,
      std::bind(&TestSelfFilter::robotDescriptionCallback, this, std::placeholders::_1));
  }

private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (sf_)
      return;

    urdf::Model model;
    if (!model.initString(msg->data))
    {
      RCLCPP_ERROR(get_logger(), "Unable to parse URDF description!");
      return;
    }

    robot_self_filter::LinkInfo li;
    li.name = "base_link";
    li.padding = .05;
    li.scale = 1.0;
    std::vector<robot_self_filter::LinkInfo> links{li};

    sf_ = std::make_unique<robot_self_filter::SelfMask<pcl::PointXYZ>>(*tf_buffer_, model, links, get_logger());

    timer_ = create_wall_timer(1s, std::bind(&TestSelfFilter::run, this));
  }

  void sendPoint(double x, double y, double z)
  {
    visualization_msgs::msg::Marker mk;

    mk.header.stamp = now();
    mk.header.frame_id = "base_link";

    mk.ns = "test_self_filter";
    mk.id = id_++;
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.position.x = x;
    mk.pose.position.y = y;
    mk.pose.position.z = z;
    mk.pose.orientation.w = 1.0;

    mk.scale.x = mk.scale.y = mk.scale.z = 0.01;

    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.04;
    mk.color.b = 0.04;

    mk.lifetime = rclcpp::Duration(10s);

    vmPub_->publish(mk);
  }

  static double uniform(double magnitude)
  {
    return (2.0 * (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX)) - 1.0) * magnitude;
  }

  void run()
  {
    // only run once
    timer_->cancel();

    pcl::PointCloud<pcl::PointXYZ> in;

    in.header.frame_id = "base_link";

    const unsigned int N = 500000;
    in.points.resize(N);
    for (unsigned int i = 0 ; i < N ; ++i)
    {
      in.points[i].x = uniform(1.5);
      in.points[i].y = uniform(1.5);
      in.points[i].z = uniform(1.5);
    }

    const auto tm = std::chrono::steady_clock::now();
    std::vector<int> mask;
    sf_->maskIntersection(in, "laser_tilt_mount_link", 0.01, mask,
                           [this](const tf2::Vector3 &pt) { sendPoint(pt.x(), pt.y(), pt.z()); });
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - tm).count();
    RCLCPP_INFO(get_logger(), "%f points per second", static_cast<double>(N) / elapsed);

    int k = 0;
    for (unsigned int i = 0 ; i < mask.size() ; ++i)
    {
      if (mask[i] != robot_self_filter::INSIDE) continue;
      k++;
    }
    RCLCPP_INFO(get_logger(), "%d points inside the robot", k);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<robot_self_filter::SelfMask<pcl::PointXYZ>> sf_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vmPub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robotDescriptionSub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSelfFilter>());
  rclcpp::shutdown();
  return 0;
}
