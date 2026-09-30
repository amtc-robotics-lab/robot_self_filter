// Checks that padding/scale set on a body built from a URDF <box> collision
// geometry (same path SelfMask::configure uses) is applied evenly on all faces.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>
#include <urdf/model.h>

#include "robot_self_filter/self_mask.h"

namespace bodies = robot_self_filter::bodies;
namespace shapes = robot_self_filter::shapes;
namespace
{
const char * kUrdf = R"(
<robot name="box_robot">
  <link name="box_link">
    <collision>
      <geometry><box size="1.0 0.6 0.4"/></geometry>
    </collision>
  </link>
</robot>)";

std::unique_ptr<bodies::Body> makeBox(double padding, double scale)
{
  urdf::Model model;
  EXPECT_TRUE(model.initString(kUrdf));
  const urdf::Link * link = model.getLink("box_link").get();
  std::unique_ptr<shapes::Shape> shape(robot_self_filter::constructShape(link->collision->geometry.get()));
  std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shape.get()));
  body->setScale(scale);
  body->setPadding(padding);
  return body;
}

// Probe just inside / just outside each face along +-axis at half-extent h.
void expectFaces(const bodies::Body & body, const tf2::Transform & pose, const double h[3])
{
  const double eps = 1e-3;
  for (int axis = 0; axis < 3; ++axis) {
    for (int sign = -1; sign <= 1; sign += 2) {
      tf2::Vector3 in(0, 0, 0), out(0, 0, 0);
      in[axis] = sign * (h[axis] - eps);
      out[axis] = sign * (h[axis] + eps);
      EXPECT_TRUE(body.containsPoint(pose * in)) << "axis " << axis << " sign " << sign;
      EXPECT_FALSE(body.containsPoint(pose * out)) << "axis " << axis << " sign " << sign;
    }
  }
}
}  // namespace

TEST(BoxPadding, NoPadding)
{
  auto body = makeBox(0.0, 1.0);
  tf2::Transform id;
  id.setIdentity();
  body->setPose(id);
  const double h[3] = {0.5, 0.3, 0.2};
  expectFaces(*body, id, h);
}

TEST(BoxPadding, PaddingAddedToEveryFace)
{
  auto body = makeBox(0.1, 1.0);
  tf2::Transform id;
  id.setIdentity();
  body->setPose(id);
  const double h[3] = {0.6, 0.4, 0.3};
  expectFaces(*body, id, h);
}

TEST(BoxPadding, PaddingWithScale)
{
  auto body = makeBox(0.1, 2.0);
  tf2::Transform id;
  id.setIdentity();
  body->setPose(id);
  const double h[3] = {1.1, 0.7, 0.5};  // 2 * half-extent + padding
  expectFaces(*body, id, h);
}

TEST(BoxPadding, PaddingWithRotatedTranslatedPose)
{
  auto body = makeBox(0.1, 1.0);
  tf2::Quaternion q;
  q.setRPY(0.3, -0.5, 1.2);
  tf2::Transform pose(q, tf2::Vector3(1.0, -2.0, 0.5));
  body->setPose(pose);
  const double h[3] = {0.6, 0.4, 0.3};
  expectFaces(*body, pose, h);
}

TEST(BoxPadding, RayHitsPaddedFace)
{
  auto body = makeBox(0.1, 1.0);
  tf2::Transform id;
  id.setIdentity();
  body->setPose(id);
  std::vector<tf2::Vector3> hits;
  ASSERT_TRUE(body->intersectsRay(tf2::Vector3(3, 0, 0), tf2::Vector3(-1, 0, 0), &hits, 1));
  ASSERT_FALSE(hits.empty());
  EXPECT_NEAR(hits[0].x(), 0.6, 1e-6);
}

TEST(BoxPadding, ParallelRayOutsideSlabMisses)
{
  auto body = makeBox(0.1, 1.0);
  tf2::Transform id;
  id.setIdentity();
  body->setPose(id);
  std::vector<tf2::Vector3> hits;
  // padded y half-extent is 0.4, z is 0.3; rays along x just outside / inside
  EXPECT_FALSE(body->intersectsRay(tf2::Vector3(3, 0.41, 0), tf2::Vector3(-1, 0, 0), &hits, 1));
  EXPECT_FALSE(body->intersectsRay(tf2::Vector3(3, 0, 0.31), tf2::Vector3(-1, 0, 0), &hits, 1));
  EXPECT_TRUE(body->intersectsRay(tf2::Vector3(3, 0.39, 0.29), tf2::Vector3(-1, 0, 0), &hits, 1));
  // starting inside: only the exit is reported
  hits.clear();
  ASSERT_TRUE(body->intersectsRay(tf2::Vector3(0, 0, 0), tf2::Vector3(1, 0, 0), &hits, 1));
  ASSERT_EQ(hits.size(), 1u);
  EXPECT_NEAR(hits[0].x(), 0.6, 1e-6);
}

TEST(BoxPadding, VolumeIncludesPadding)
{
  auto body = makeBox(0.1, 1.0);
  EXPECT_NEAR(body->computeVolume(), 1.2 * 0.8 * 0.6, 1e-9);
}

// End-to-end through SelfMask: a link with two collision elements (a box and a
// sphere, both offset) must mask points near BOTH, padded by 0.2 m.
TEST(SelfMaskPadding, AllCollisionElementsOfALinkArePaddedAndMasked)
{
  const char * urdf = R"(
  <robot name="two_geom">
    <link name="world"/>
    <link name="multi">
      <collision>
        <origin xyz="1 0 0"/>
        <geometry><sphere radius="0.1"/></geometry>
      </collision>
      <collision>
        <origin xyz="-1 0 0"/>
        <geometry><box size="0.4 0.4 0.4"/></geometry>
      </collision>
    </link>
    <joint name="j" type="fixed"><parent link="world"/><child link="multi"/></joint>
  </robot>)";
  urdf::Model model;
  ASSERT_TRUE(model.initString(urdf));

  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = "multi";
  t.transform.rotation.w = 1.0;
  buffer.setTransform(t, "test", true);

  robot_self_filter::LinkInfo li;
  li.name = "multi";
  li.padding = 0.2;
  li.scale = 1.0;
  robot_self_filter::SelfMask<pcl::PointXYZ> mask(buffer, model, {li});

  std::vector<std::string> names;
  mask.getLinkNames(names);
  ASSERT_EQ(names.size(), 2u);

  std_msgs::msg::Header header;
  header.frame_id = "world";
  mask.assumeFrame(header);
  using robot_self_filter::INSIDE;
  using robot_self_filter::OUTSIDE;

  // box at x=-1, half extent 0.2 + 0.2 padding = 0.4
  EXPECT_EQ(mask.getMaskContainment(-1.39, 0, 0), INSIDE);
  EXPECT_EQ(mask.getMaskContainment(-1.41, 0, 0), OUTSIDE);
  EXPECT_EQ(mask.getMaskContainment(-1, 0.39, 0.39), INSIDE);
  EXPECT_EQ(mask.getMaskContainment(-1, 0.41, 0), OUTSIDE);
  // sphere at x=+1, radius 0.1 + 0.2 padding = 0.3
  EXPECT_EQ(mask.getMaskContainment(1.29, 0, 0), INSIDE);
  EXPECT_EQ(mask.getMaskContainment(1.31, 0, 0), OUTSIDE);
}

// With a sensor position the shadow test runs too: points inside the padding zone must
// be INSIDE (not SHADOW, which a ray leaving the padded body would otherwise produce),
// and points behind the robot as seen from the sensor must be SHADOW.
TEST(SelfMaskPadding, PaddingZoneIsInsideNotShadowWhenSensorIsSet)
{
  const char * urdf = R"(
  <robot name="b"><link name="world"/>
    <link name="box_link"><collision><geometry><box size="1.0 0.6 0.4"/></geometry></collision></link>
    <joint name="j" type="fixed"><parent link="world"/><child link="box_link"/></joint></robot>)";
  urdf::Model model;
  ASSERT_TRUE(model.initString(urdf));
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = "box_link";
  t.transform.rotation.w = 1.0;
  buffer.setTransform(t, "test", true);

  robot_self_filter::LinkInfo li;
  li.name = "box_link";
  li.padding = 0.2;
  li.scale = 1.0;
  robot_self_filter::SelfMask<pcl::PointXYZ> mask(buffer, model, {li});
  std_msgs::msg::Header header;
  header.frame_id = "world";
  mask.assumeFrame(header, tf2::Vector3(5, 0, 0), 0.01);

  using robot_self_filter::INSIDE;
  using robot_self_filter::OUTSIDE;
  using robot_self_filter::SHADOW;
  EXPECT_EQ(mask.getMaskIntersection(0.6, 0, 0), INSIDE);    // padding zone, sensor side
  EXPECT_EQ(mask.getMaskIntersection(-0.69, 0, 0), INSIDE);  // padding zone, far side
  EXPECT_EQ(mask.getMaskIntersection(0, 0.49, 0), INSIDE);
  EXPECT_EQ(mask.getMaskIntersection(0.3, 0, 0), INSIDE);    // inside the geometry
  EXPECT_EQ(mask.getMaskIntersection(0.71, 0, 0), OUTSIDE);
  EXPECT_EQ(mask.getMaskIntersection(2, 1.0, 0), OUTSIDE);   // its ray to the sensor misses the box
  EXPECT_EQ(mask.getMaskIntersection(-0.71, 0, 0), SHADOW);  // behind the box seen from the sensor
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
