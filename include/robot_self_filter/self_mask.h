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

#ifndef ROBOT_SELF_FILTER_SELF_MASK_
#define ROBOT_SELF_FILTER_SELF_MASK_

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <robot_self_filter/bodies.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>
#include <filesystem>

#include <urdf/model.h>
#include <resource_retriever/retriever.hpp>

namespace robot_self_filter
{

/** \brief The possible values of a mask computed for a point */
enum
{
  INSIDE = 0,
  OUTSIDE = 1,
  SHADOW = 2,
};

struct LinkInfo
{
  std::string name;
  double padding;
  double scale;
};

static inline tf2::Transform urdfPose2TFTransform(const urdf::Pose &pose)
{
  return tf2::Transform(tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
                         tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

static inline tf2::Transform transformMsgToTF2(const geometry_msgs::msg::Transform &t)
{
  return tf2::Transform(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                         tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
}

static inline shapes::Shape* constructShape(const urdf::Geometry *geom,
                                             const rclcpp::Logger &logger = rclcpp::get_logger("robot_self_filter"))
{
  rcpputils::assert_true(geom != NULL, "Geometry pointer is NULL");

  shapes::Shape *result = NULL;
  switch (geom->type)
  {
  case urdf::Geometry::SPHERE:
    result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
    break;
  case urdf::Geometry::BOX:
    {
      urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
      result = new shapes::Box(dim.x, dim.y, dim.z);
    }
    break;
  case urdf::Geometry::CYLINDER:
    result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
                                   dynamic_cast<const urdf::Cylinder*>(geom)->length);
    break;
  case urdf::Geometry::MESH:
    {
      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
      if (!mesh->filename.empty())
      {
        resource_retriever::Retriever retriever;
        resource_retriever::MemoryResource res;
        bool ok = true;

        try
        {
          res = retriever.get(mesh->filename);
        }
        catch (resource_retriever::Exception& e)
        {
          RCLCPP_ERROR(logger, "%s", e.what());
          ok = false;
        }

        if (ok)
        {
          if (res.size == 0)
            RCLCPP_WARN(logger, "Retrieved empty mesh for resource '%s'", mesh->filename.c_str());
          else
          {
            std::filesystem::path model_path(mesh->filename);
            std::string ext = model_path.extension().string();
            if (ext == ".dae" || ext == ".DAE") {
              result = shapes::createMeshFromBinaryDAE(mesh->filename.c_str(), logger);
            }
            else {
              result = shapes::createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size);
            }
            if (result == NULL)
              RCLCPP_ERROR(logger, "Failed to load mesh '%s'", mesh->filename.c_str());
          }
        }
      }
      else
        RCLCPP_WARN(logger, "Empty mesh filename");
    }

    break;
  default:
    RCLCPP_ERROR(logger, "Unknown geometry type: %d", (int)geom->type);
    break;
  }

  return result;
}

/** \brief Computing a mask for a pointcloud that states which points are inside the robot
 *
 */
template <typename PointT>
class SelfMask
{
protected:

  struct SeeLink
  {
    SeeLink(void)
    {
      body = unscaledBody = NULL;
    }

    std::string   name;
    bodies::Body *body;
    bodies::Body *unscaledBody;
    tf2::Transform   constTransf;
    double        volume;
  };

  struct SortBodies
  {
    bool operator()(const SeeLink &b1, const SeeLink &b2)
    {
      return b1.volume > b2.volume;
    }
  };

public:
  typedef pcl::PointCloud<PointT> PointCloud;

  /** \brief Construct the filter */
  SelfMask(tf2_ros::Buffer &tf_buffer, const urdf::Model &urdf_model, const std::vector<LinkInfo> &links,
           const rclcpp::Logger &logger = rclcpp::get_logger("robot_self_filter"))
    : tf_(tf_buffer), logger_(logger)
  {
    configure(urdf_model, links);
  }

  /** \brief Destructor to clean up
   */
  ~SelfMask(void)
  {
    freeMemory();
  }

  /** \brief Compute the containment mask (INSIDE or OUTSIDE) for a given pointcloud. If a mask element is INSIDE, the point
      is inside the robot. The point is outside if the mask element is OUTSIDE.
   */
  void maskContainment(const PointCloud& data_in, std::vector<int> &mask)
  {
    mask.resize(data_in.points.size());
    if (bodies_.empty())
      std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    else
    {
      std_msgs::msg::Header header = pcl_conversions::fromPCL(data_in.header);
      assumeFrame(header);
      maskAuxContainment(data_in, mask);
    }
  }

  /** \brief Compute the intersection mask for a given
      pointcloud. If a mask element can have one of the values
      INSIDE, OUTSIDE or SHADOW. If the value is SHADOW, the
      point is on a ray behind the robot and should not have
      been seen. If the mask element is INSIDE, the point is
      inside the robot. The sensor frame is specified to obtain
      the origin of the sensor. A callback can be registered for
      the first intersection point on each body.
   */
  void maskIntersection(const PointCloud& data_in, const std::string &sensor_frame, const double min_sensor_dist,
                         std::vector<int> &mask,
                         const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr)
  {
    mask.resize(data_in.points.size());
    if (bodies_.empty()) {
      std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    }
    else
    {
      std_msgs::msg::Header header = pcl_conversions::fromPCL(data_in.header);
      assumeFrame(header, sensor_frame, min_sensor_dist);
      if (sensor_frame.empty())
        maskAuxContainment(data_in, mask);
      else
        maskAuxIntersection(data_in, mask, intersectionCallback);
    }
  }


  /** \brief Compute the intersection mask for a given pointcloud. If a mask
      element can have one of the values INSIDE, OUTSIDE or SHADOW. If the value is SHADOW,
      the point is on a ray behind the robot and should not have
      been seen. If the mask element is INSIDE, the point is inside
      the robot. The origin of the sensor is specified as well.
   */
  void maskIntersection(const PointCloud& data_in, const tf2::Vector3 &sensor_pos, const double min_sensor_dist,
                         std::vector<int> &mask,
                         const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr)
  {
    mask.resize(data_in.points.size());
    if (bodies_.empty())
      std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
    else
    {
      std_msgs::msg::Header header = pcl_conversions::fromPCL(data_in.header);
      assumeFrame(header, sensor_pos, min_sensor_dist);
      maskAuxIntersection(data_in, mask, intersectionCallback);
    }
  }

  /** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
   *   The frame in which the sensor is located is optional */
  void assumeFrame(const std_msgs::msg::Header& header)
  {
    const unsigned int bs = bodies_.size();
    const tf2::TimePoint time = tf2_ros::fromMsg(header.stamp);
    const tf2::Duration timeout = tf2::durationFromSec(0.1);

    // place the links in the assumed frame
    for (unsigned int i = 0 ; i < bs ; ++i)
    {
      geometry_msgs::msg::TransformStamped transf;
      try
      {
        transf = tf_.lookupTransform(header.frame_id, bodies_[i].name, time, timeout);
      }
      catch(tf2::TransformException& ex)
      {
        RCLCPP_ERROR(logger_, "Unable to lookup transform from %s to %s. Exception: %s",
                     bodies_[i].name.c_str(), header.frame_id.c_str(), ex.what());
        continue;
      }

      // set it for each body; we also include the offset specified in URDF
      tf2::Transform tf2Transf = transformMsgToTF2(transf.transform);
      bodies_[i].body->setPose(tf2Transf * bodies_[i].constTransf);
      bodies_[i].unscaledBody->setPose(tf2Transf * bodies_[i].constTransf);
    }

    computeBoundingSpheres();
  }


  /** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
   *  Also specify which possition to assume for the sensor (frame is not needed) */
  void assumeFrame(const std_msgs::msg::Header& header, const tf2::Vector3 &sensor_pos, const double min_sensor_dist)
  {
    assumeFrame(header);
    sensor_pos_ = sensor_pos;
    min_sensor_dist_ = min_sensor_dist;
  }

  /** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
   *   The frame in which the sensor is located is optional */
  void assumeFrame(const std_msgs::msg::Header& header, const std::string &sensor_frame, const double min_sensor_dist)
  {
    assumeFrame(header);

    // compute the origin of the sensor in the frame of the cloud
    try
    {
      geometry_msgs::msg::TransformStamped transf = tf_.lookupTransform(
        header.frame_id, sensor_frame, tf2_ros::fromMsg(header.stamp), tf2::durationFromSec(0.1));
      sensor_pos_ = tf2::Vector3(transf.transform.translation.x,
                                  transf.transform.translation.y,
                                  transf.transform.translation.z);
    }
    catch(tf2::TransformException& ex)
    {
      sensor_pos_.setValue(0, 0, 0);
      RCLCPP_ERROR(logger_, "Unable to lookup transform from %s to %s.  Exception: %s",
                   sensor_frame.c_str(), header.frame_id.c_str(), ex.what());
    }

    min_sensor_dist_ = min_sensor_dist;
  }

  /** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point. No
      setup is performed, assumeFrame() should be called before use */
  int  getMaskContainment(const tf2::Vector3 &pt) const
  {
    const unsigned int bs = bodies_.size();
    int out = OUTSIDE;
    for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
      if (bodies_[j].body->containsPoint(pt))
        out = INSIDE;
    return out;
  }

  /** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point. No
      setup is performed, assumeFrame() should be called before use */
  int  getMaskContainment(double x, double y, double z) const
  {
    return getMaskContainment(tf2::Vector3(x, y, z));
  }

  /** \brief Get the intersection mask (INSIDE, OUTSIDE or
      SHADOW) value for an individual point. No setup is
      performed, assumeFrame() should be called before use */
  int  getMaskIntersection(double x, double y, double z,
                            const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr) const
  {
    return getMaskIntersection(tf2::Vector3(x, y, z), intersectionCallback);
  }

  /** \brief Get the intersection mask (INSIDE, OUTSIDE or
      SHADOW) value for an individual point. No setup is
      performed, assumeFrame() should be called before use */
  int  getMaskIntersection(const tf2::Vector3 &pt,
                            const std::function<void(const tf2::Vector3&)> &intersectionCallback = nullptr) const
  {
    const unsigned int bs = bodies_.size();

    // we first check is the point is in the unscaled body.
    // if it is, the point is definitely inside
    int out = OUTSIDE;
    for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
      if (bodies_[j].unscaledBody->containsPoint(pt))
        out = INSIDE;

    if (out == OUTSIDE)
    {
      // we check if the point is a shadow point
      tf2::Vector3 dir(sensor_pos_ - pt);
      tf2Scalar  lng = dir.length();
      if (lng < min_sensor_dist_)
        out = INSIDE;
      else
      {
        dir /= lng;

        std::vector<tf2::Vector3> intersections;
        for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
        {
          // get the 1st intersection of ray pt->sensor
          intersections.clear(); // intersectsRay doesn't clear the vector...
          if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
          {
            // is the intersection between point and sensor?
            if (dir.dot(sensor_pos_ - intersections[0]) >= 0.0)
            {
              if (intersectionCallback)
                intersectionCallback(intersections[0]);
              out = SHADOW;
            }
          }
        }

        // if it is not a shadow point, we check if it is inside the scaled body
        for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
          if (bodies_[j].body->containsPoint(pt))
            out = INSIDE;
      }
    }
    return out;
  }

  /** \brief Get the set of link names that have been instantiated for self filtering */
  void getLinkNames(std::vector<std::string> &frames) const
  {
    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
      frames.push_back(bodies_[i].name);
  }

protected:

  /** \brief Free memory. */
  void freeMemory(void)
  {
    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
    {
      if (bodies_[i].body)
        delete bodies_[i].body;
      if (bodies_[i].unscaledBody)
        delete bodies_[i].unscaledBody;
    }

    bodies_.clear();
  }


  /** \brief Configure the filter. */
  bool configure(const urdf::Model &urdfModel, const std::vector<LinkInfo> &links)
  {
    // in case configure was called before, we free the memory
    freeMemory();
    sensor_pos_.setValue(0, 0, 0);

    std::stringstream missing;

    // from the geometric model, find the shape of each link of interest
    // and create a body from it, one that knows about poses and can
    // check for point inclusion
    for (unsigned int i = 0 ; i < links.size() ; ++i)
    {
      const urdf::Link *link = urdfModel.getLink(links[i].name).get();
      if (!link)
      {
        missing << " " << links[i].name;
        continue;
      }

      // a link can have several <collision> elements; every one of them is filtered
      std::vector<urdf::CollisionSharedPtr> collisions = link->collision_array;
      if (collisions.empty() && link->collision)
        collisions.push_back(link->collision);

      bool has_geometry = false;
      for (const auto &collision : collisions)
      {
        if (!collision || !collision->geometry)
          continue;
        has_geometry = true;

        shapes::Shape *shape = constructShape(collision->geometry.get(), logger_);

        if (!shape)
        {
          RCLCPP_ERROR(logger_, "Unable to construct collision shape for link '%s'", links[i].name.c_str());
          continue;
        }

        SeeLink sl;
        sl.body = bodies::createBodyFromShape(shape);

        if (sl.body)
        {
          sl.name = links[i].name;

          // collision models may have an offset, in addition to what TF gives
          // so we keep it around
          sl.constTransf = urdfPose2TFTransform(collision->origin);

          sl.body->setScale(links[i].scale);
          sl.body->setPadding(links[i].padding);
          RCLCPP_INFO(logger_, "Self see link name %s padding %f", links[i].name.c_str(), links[i].padding);
          sl.volume = sl.body->computeVolume();
          sl.unscaledBody = bodies::createBodyFromShape(shape);
          bodies_.push_back(sl);
        }
        else
          RCLCPP_WARN(logger_, "Unable to create point inclusion body for link '%s'", links[i].name.c_str());

        delete shape;
      }

      if (!has_geometry)
        RCLCPP_WARN(logger_, "No collision geometry specified for link '%s'", links[i].name.c_str());
    }

    if (missing.str().size() > 0)
      RCLCPP_WARN(logger_, "Some links were included for self mask but they do not exist in the model:%s", missing.str().c_str());

    if (bodies_.empty())
      RCLCPP_WARN(logger_, "No robot links will be checked for self mask");

    // put larger volume bodies first -- higher chances of containing a point
    std::sort(bodies_.begin(), bodies_.end(), SortBodies());

    bspheres_.resize(bodies_.size());
    bspheresRadius2_.resize(bodies_.size());

    for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
      RCLCPP_DEBUG(logger_, "Self mask includes link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].volume);

    return true;
  }

  /** \brief Compute bounding spheres for the checked robot links. */
  void computeBoundingSpheres(void)
  {
    const unsigned int bs = bodies_.size();
    for (unsigned int i = 0 ; i < bs ; ++i)
    {
      bodies_[i].body->computeBoundingSphere(bspheres_[i]);
      bspheresRadius2_[i] = bspheres_[i].radius * bspheres_[i].radius;
    }
  }


  /** \brief Perform the actual mask computation. */
  void maskAuxContainment(const PointCloud& data_in, std::vector<int> &mask)
  {
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.points.size();

    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    tf2Scalar radiusSquared = bound.radius * bound.radius;

    // we now decide which points we keep
    for (int i = 0 ; i < (int)np ; ++i)
    {
      tf2::Vector3 pt = tf2::Vector3(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
      int out = OUTSIDE;
      if (bound.center.distance2(pt) < radiusSquared)
        for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
          if (bodies_[j].body->containsPoint(pt))
            out = INSIDE;

      mask[i] = out;
    }
  }

  /** \brief Perform the actual mask computation. */
  void maskAuxIntersection(const PointCloud& data_in, std::vector<int> &mask,
                            const std::function<void(const tf2::Vector3&)> &callback)
  {
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.points.size();

    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    tf2Scalar radiusSquared = bound.radius * bound.radius;

    // we now decide which points we keep
    for (int i = 0 ; i < (int)np ; ++i)
    {
      tf2::Vector3 pt = tf2::Vector3(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
      int out = OUTSIDE;

      // we first check is the point is in the unscaled body.
      // if it is, the point is definitely inside
      if (bound.center.distance2(pt) < radiusSquared)
        for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
          if (bodies_[j].unscaledBody->containsPoint(pt))
            out = INSIDE;

      // if the point is not inside the unscaled body,
      if (out == OUTSIDE)
      {
        // we check if the point is a shadow point
        tf2::Vector3 dir(sensor_pos_ - pt);
        tf2Scalar  lng = dir.length();
        if (lng < min_sensor_dist_) {
          out = INSIDE;
        }
        else
        {
          dir /= lng;

          std::vector<tf2::Vector3> intersections;
          for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j) {
            // get the 1st intersection of ray pt->sensor
            intersections.clear(); // intersectsRay doesn't clear the vector...
            if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
            {
              // is the intersection between point and sensor?
              if (dir.dot(sensor_pos_ - intersections[0]) >= 0.0)
              {
                if (callback)
                  callback(intersections[0]);
                out = SHADOW;
              }
            }
          }
          // if it is not a shadow point, we check if it is inside the scaled body
          if (out == OUTSIDE && bound.center.distance2(pt) < radiusSquared)
            for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
              if (bodies_[j].body->containsPoint(pt)) {
                out = INSIDE;
              }
        }
      }
      mask[i] = out;
    }
  }

  tf2_ros::Buffer                     &tf_;
  rclcpp::Logger                       logger_;

  tf2::Vector3                        sensor_pos_;
  double                              min_sensor_dist_;

  std::vector<SeeLink>                bodies_;
  std::vector<double>                 bspheresRadius2_;
  std::vector<bodies::BoundingSphere> bspheres_;

};

}

#endif
