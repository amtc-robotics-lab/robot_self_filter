// Padding on meshes (ConvexMesh) and on the merged bounding sphere used as the
// SelfMask pre-filter.

#include <gtest/gtest.h>

#include <memory>
#include <random>

#include "robot_self_filter/bodies.h"
#include "robot_self_filter/shapes.h"

namespace bodies = robot_self_filter::bodies;
namespace shapes = robot_self_filter::shapes;

namespace
{
// axis-aligned box mesh centred on the origin with half extents hx,hy,hz
std::unique_ptr<shapes::Mesh> boxMesh(double hx, double hy, double hz)
{
  auto m = std::make_unique<shapes::Mesh>(8, 12);
  for (int i = 0; i < 8; ++i) {
    m->vertices[3 * i] = (i & 1 ? hx : -hx);
    m->vertices[3 * i + 1] = (i & 2 ? hy : -hy);
    m->vertices[3 * i + 2] = (i & 4 ? hz : -hz);
  }
  const unsigned int t[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                                 {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  for (int i = 0; i < 12; ++i)
    for (int j = 0; j < 3; ++j) m->triangles[3 * i + j] = t[i][j];
  return m;
}

std::unique_ptr<bodies::Body> makeMeshBody(const shapes::Shape * s, double padding)
{
  std::unique_ptr<bodies::Body> b(bodies::createBodyFromShape(s));
  b->setPadding(padding);
  tf2::Transform id;
  id.setIdentity();
  b->setPose(id);
  return b;
}
}  // namespace

class MeshPadding : public ::testing::TestWithParam<double> {};

// Every face of a box-shaped mesh must be pushed out by exactly `padding`.
TEST_P(MeshPadding, EveryFaceIsPaddedEvenly)
{
  const double pad = GetParam();
  const double h[3] = {0.5, 0.3, 0.2};
  auto mesh = boxMesh(h[0], h[1], h[2]);
  auto body = makeMeshBody(mesh.get(), pad);
  for (int axis = 0; axis < 3; ++axis)
    for (int sign = -1; sign <= 1; sign += 2) {
      tf2::Vector3 in(0, 0, 0), out(0, 0, 0);
      in[axis] = sign * (h[axis] + pad - 1e-2);
      out[axis] = sign * (h[axis] + pad + 1e-2);
      EXPECT_TRUE(body->containsPoint(in)) << "axis " << axis << " sign " << sign;
      EXPECT_FALSE(body->containsPoint(out)) << "axis " << axis << " sign " << sign;
    }
}

// The shadow ray test must see the same padded surface as containsPoint.
TEST_P(MeshPadding, RayHitsPaddedFace)
{
  const double pad = GetParam();
  auto mesh = boxMesh(0.5, 0.3, 0.2);
  auto body = makeMeshBody(mesh.get(), pad);
  std::vector<tf2::Vector3> hits;
  ASSERT_TRUE(body->intersectsRay(tf2::Vector3(10, 0, 0), tf2::Vector3(-1, 0, 0), &hits, 1));
  ASSERT_FALSE(hits.empty());
  EXPECT_NEAR(hits[0].x(), 0.5 + pad, 1e-3);
}

// Scale grows the mesh about its center, then padding is added on top (mesh
// translated away from the origin).
TEST(MeshScale, ScaleThenPaddingTranslatedMesh)
{
  auto mesh = boxMesh(0.5, 0.3, 0.2);
  for (unsigned int i = 0; i < mesh->vertexCount; ++i) mesh->vertices[3 * i] += 2.0;  // centered at x=2
  std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(mesh.get()));
  body->setScale(2.0);
  body->setPadding(0.1);
  tf2::Transform id;
  id.setIdentity();
  body->setPose(id);
  // symmetric box: center x=2, half extent 0.5*2+0.1 = 1.1
  EXPECT_TRUE(body->containsPoint(tf2::Vector3(2.0 + 1.09, 0, 0)));
  EXPECT_FALSE(body->containsPoint(tf2::Vector3(2.0 + 1.11, 0, 0)));
  EXPECT_TRUE(body->containsPoint(tf2::Vector3(2.0 - 1.09, 0, 0)));
  EXPECT_TRUE(body->containsPoint(tf2::Vector3(2.0, 0.6 * 1.0 - 0.01 + 0.1, 0)));   // 0.3*2+0.1 = 0.7
  EXPECT_FALSE(body->containsPoint(tf2::Vector3(2.0, 0.71, 0)));
  std::vector<tf2::Vector3> hits;
  ASSERT_TRUE(body->intersectsRay(tf2::Vector3(10, 0, 0), tf2::Vector3(-1, 0, 0), &hits, 1));
  EXPECT_NEAR(hits[0].x(), 3.1, 1e-3);
}

// A ray starting inside the body exits through the padded surface.
TEST(MeshScale, RayFromInsideExitsAtPaddedSurface)
{
  auto mesh = boxMesh(0.5, 0.3, 0.2);
  auto body = makeMeshBody(mesh.get(), 0.2);
  std::vector<tf2::Vector3> hits;
  ASSERT_TRUE(body->intersectsRay(tf2::Vector3(0, 0, 0), tf2::Vector3(0, 1, 0), &hits, 1));
  EXPECT_NEAR(hits[0].y(), 0.5, 1e-3);
  EXPECT_FALSE(body->intersectsRay(tf2::Vector3(0, 2, 0), tf2::Vector3(0, 1, 0), &hits, 1));
}

INSTANTIATE_TEST_SUITE_P(Pads, MeshPadding, ::testing::Values(0.0, 0.2, 1.2));

// The marker geometry must be a closed, gap-free surface: for a box mesh it has the exact
// area of the padded box, and every vertex lies on the padded box surface.
TEST(MeshPaddedGeometry, ClosedSurfaceWithoutGaps)
{
  auto mesh = boxMesh(0.5, 0.3, 0.2);
  auto body = makeMeshBody(mesh.get(), 0.2);
  bodies::PaddedGeometry g;
  body->getPaddedGeometry(g);
  ASSERT_EQ(g.type, shapes::MESH);
  ASSERT_EQ(g.triangles.size() % 3, 0u);
  double area = 0.0;
  for (size_t i = 0; i < g.triangles.size(); i += 3)
    area += 0.5 * (g.triangles[i + 1] - g.triangles[i]).cross(g.triangles[i + 2] - g.triangles[i]).length();
  EXPECT_NEAR(area, 2 * (1.4 * 1.0 + 1.4 * 0.8 + 1.0 * 0.8), 1e-6);
  for (const auto & v : g.triangles) {
    EXPECT_LE(fabs(v.x()), 0.7 + 1e-6);
    EXPECT_LE(fabs(v.y()), 0.5 + 1e-6);
    EXPECT_LE(fabs(v.z()), 0.4 + 1e-6);
    const bool on_surface = fabs(fabs(v.x()) - 0.7) < 1e-6 || fabs(fabs(v.y()) - 0.5) < 1e-6 || fabs(fabs(v.z()) - 0.4) < 1e-6;
    EXPECT_TRUE(on_surface);
  }
}

// Same check on a non-box hull (octahedron): the drawn surface is the boundary of the region
// containsPoint() accepts.
TEST(MeshPaddedGeometry, MatchesContainsPointOnOctahedron)
{
  auto m = std::make_unique<shapes::Mesh>(6, 8);
  const double verts[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 3; ++j) m->vertices[3 * i + j] = verts[i][j];
  const unsigned int t[8][3] = {{0, 2, 4}, {2, 1, 4}, {1, 3, 4}, {3, 0, 4}, {2, 0, 5}, {1, 2, 5}, {3, 1, 5}, {0, 3, 5}};
  for (int i = 0; i < 8; ++i)
    for (int j = 0; j < 3; ++j) m->triangles[3 * i + j] = t[i][j];
  auto body = makeMeshBody(m.get(), 0.3);
  bodies::PaddedGeometry g;
  body->getPaddedGeometry(g);
  ASSERT_FALSE(g.triangles.empty());
  for (size_t i = 0; i < g.triangles.size(); i += 3) {
    for (int j = 0; j < 3; ++j)
      EXPECT_TRUE(body->containsPoint(g.triangles[i + j] * (1.0 - 1e-4)));   // on the boundary, just inside
    const tf2::Vector3 n = (g.triangles[i + 1] - g.triangles[i]).cross(g.triangles[i + 2] - g.triangles[i]).normalized();
    const tf2::Vector3 centroid = (g.triangles[i] + g.triangles[i + 1] + g.triangles[i + 2]) / 3.0;
    // one of the two sides must be outside: nudging the triangle along its normal leaves the body
    EXPECT_TRUE(!body->containsPoint(centroid + n * 1e-3) || !body->containsPoint(centroid - n * 1e-3));
    EXPECT_TRUE(body->containsPoint(centroid - n * 1e-3) || body->containsPoint(centroid + n * 1e-3));
  }
}

// The pre-filter sphere must contain every body's own bounding sphere.
TEST(BoundingSphereMerge, ContainsAllInputs)
{
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> pos(-2, 2), rad(0.05, 1.5);
  for (int trial = 0; trial < 2000; ++trial) {
    std::vector<bodies::BoundingSphere> spheres(2 + trial % 6);
    for (auto & s : spheres) {
      s.center = tf2::Vector3(pos(rng), pos(rng), pos(rng));
      s.radius = rad(rng);
    }
    bodies::BoundingSphere merged;
    bodies::mergeBoundingSpheres(spheres, merged);
    for (const auto & s : spheres)
      ASSERT_LE(s.center.distance(merged.center) + s.radius, merged.radius + 1e-9) << "trial " << trial;
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
