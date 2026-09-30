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

#include "robot_self_filter/bodies.h"
#include <bullet/LinearMath/btConvexHull.h>
// #include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace robot_self_filter
{
bodies::Body* bodies::createBodyFromShape(const shapes::Shape *shape)
{
    Body *body = NULL;
    
    if (shape)
	switch (shape->type)
	{
	case shapes::BOX:
	    body = new bodies::Box(shape);
	    break;
	case shapes::SPHERE:
	    body = new bodies::Sphere(shape);
	    break;
	case shapes::CYLINDER:
	    body = new bodies::Cylinder(shape);
	    break;
	case shapes::MESH:
	    body = new bodies::ConvexMesh(shape);
	    break;
	default:
	    std::cerr << "Creating body from shape: Unknown shape type" << shape->type << std::endl;
	    break;
	}
    
    return body;
}

void bodies::mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere)
{
    if (spheres.empty())
    {
	mergedSphere.center.setValue(tf2Scalar(0), tf2Scalar(0), tf2Scalar(0));
	mergedSphere.radius = 0.0;
    }
    else
    {
	mergedSphere = spheres[0];
	for (unsigned int i = 1 ; i < spheres.size() ; ++i)
	{
	    if (spheres[i].radius <= 0.0)
		continue;
	    double d = spheres[i].center.distance(mergedSphere.center);
	    if (d + mergedSphere.radius <= spheres[i].radius)
	    {
		mergedSphere.center = spheres[i].center;
		mergedSphere.radius = spheres[i].radius;
	    }
	    else
		if (d + spheres[i].radius > mergedSphere.radius)
		{
		    tf2::Vector3 delta = mergedSphere.center - spheres[i].center;
		    mergedSphere.radius = (delta.length() + spheres[i].radius + mergedSphere.radius)/2.0;
		    mergedSphere.center = delta.normalized() * (mergedSphere.radius - spheres[i].radius) + spheres[i].center;
		}
	}
    }
}

namespace bodies
{
    static const double ZERO = 1e-9;
    
    /** \brief Compute the square of the distance between a ray and a point 
	Note: this requires 'dir' to be normalized */
    static inline double distanceSQR(const tf2::Vector3& p, const tf2::Vector3& origin, const tf2::Vector3& dir)
    {
	tf2::Vector3 a = p - origin;
	double d = dir.dot(a);
	return a.length2() - d * d;
    }
    
    namespace detail
    {
	
	// temp structure for intersection points (used for ordering them)
	struct intersc
	{
	    intersc(const tf2::Vector3 &_pt, const double _tm) : pt(_pt), time(_tm) {}
	    
	    tf2::Vector3 pt;
	    double    time;
	};
	
	// define order on intersection points
	struct interscOrder
	{
	    bool operator()(const intersc &a, const intersc &b) const
	    {
		return a.time < b.time;
	    }
	};
    }
}

bool bodies::Sphere::containsPoint(const tf2::Vector3 &p, bool verbose) const
{
    return (m_center - p).length2() < m_radius2;
}

void bodies::Sphere::useDimensions(const shapes::Shape *shape) // radius
{
    m_radius = static_cast<const shapes::Sphere*>(shape)->radius;
}

void bodies::Sphere::updateInternalData(void)
{
    m_radiusU = m_radius * m_scale + m_padding;
    m_radius2 = m_radiusU * m_radiusU;
    m_center = m_pose.getOrigin();
}

double bodies::Sphere::computeVolume(void) const
{
    return 4.0 * M_PI * m_radiusU * m_radiusU * m_radiusU / 3.0;
}

void bodies::Sphere::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusU;
}

void bodies::Sphere::getPaddedGeometry(PaddedGeometry &geometry) const
{
    geometry.type = shapes::SPHERE;
    const double d = 2.0 * m_radiusU;
    geometry.size = tf2::Vector3(d, d, d);
    geometry.triangles.clear();
}

bool bodies::Sphere::intersectsRay(const tf2::Vector3& origin, const tf2::Vector3& dir, std::vector<tf2::Vector3> *intersections, unsigned int count) const
{
    if (distanceSQR(m_center, origin, dir) > m_radius2) return false;
    
    bool result = false;
    
    tf2::Vector3 cp = origin - m_center;
    double dpcpv = cp.dot(dir);
    
    tf2::Vector3 w = cp - dpcpv * dir;
    tf2::Vector3 Q = m_center + w;
    double x = m_radius2 - w.length2();
    
    if (fabs(x) < ZERO)
    { 
	w = Q - origin;
	double dpQv = w.dot(dir);
	if (dpQv > ZERO)
	{
	    if (intersections)
		intersections->push_back(Q);
	    result = true;
	}
    } else 
	if (x > 0.0)
	{    
	    x = sqrt(x);
	    w = dir * x;
	    tf2::Vector3 A = Q - w;
	    tf2::Vector3 B = Q + w;
	    w = A - origin;
	    double dpAv = w.dot(dir);
	    w = B - origin;
	    double dpBv = w.dot(dir);
	    
	    if (dpAv > ZERO)
	    {	
		result = true;
		if (intersections)
		{
		    intersections->push_back(A);
		    if (count == 1)
			return result;
		}
	    }
	    
	    if (dpBv > ZERO)
	    {
		result = true;
		if (intersections)
		    intersections->push_back(B);
	    }
	}
    return result;
}

bool bodies::Cylinder::containsPoint(const tf2::Vector3 &p, bool verbose) const 
{
    tf2::Vector3 v = p - m_center;		
    double pH = v.dot(m_normalH);
    
    if (fabs(pH) > m_length2)
	return false;
    
    double pB1 = v.dot(m_normalB1);
    double remaining = m_radius2 - pB1 * pB1;
    
    if (remaining < 0.0)
	return false;
    else
    {
	double pB2 = v.dot(m_normalB2);
	return pB2 * pB2 < remaining;
    }		
}

void bodies::Cylinder::useDimensions(const shapes::Shape *shape) // (length, radius)
{
    m_length = static_cast<const shapes::Cylinder*>(shape)->length;
    m_radius = static_cast<const shapes::Cylinder*>(shape)->radius;
}

void bodies::Cylinder::updateInternalData(void)
{
    m_radiusU = m_radius * m_scale + m_padding;
    m_radius2 = m_radiusU * m_radiusU;
    m_length2 = m_scale * m_length / 2.0 + m_padding;
    m_center = m_pose.getOrigin();
    m_radiusBSqr = m_length2 * m_length2 + m_radius2;
    m_radiusB = sqrt(m_radiusBSqr);
    
    const tf2::Matrix3x3& basis = m_pose.getBasis();
    m_normalB1 = basis.getColumn(0);
    m_normalB2 = basis.getColumn(1);
    m_normalH  = basis.getColumn(2);
    
    double tmp = -m_normalH.dot(m_center);
    m_d1 = tmp + m_length2;
    m_d2 = tmp - m_length2;
}

double bodies::Cylinder::computeVolume(void) const
{
    return 2.0 * M_PI * m_radius2 * m_length2;
}

void bodies::Cylinder::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

void bodies::Cylinder::getPaddedGeometry(PaddedGeometry &geometry) const
{
    geometry.type = shapes::CYLINDER;
    geometry.size = tf2::Vector3(2.0 * m_radiusU, 2.0 * m_radiusU, 2.0 * m_length2);
    geometry.triangles.clear();
}

bool bodies::Cylinder::intersectsRay(const tf2::Vector3& origin, const tf2::Vector3& dir, std::vector<tf2::Vector3> *intersections, unsigned int count) const
{
    if (distanceSQR(m_center, origin, dir) > m_radiusBSqr) return false;

    std::vector<detail::intersc> ipts;
    
    // intersect bases
    double tmp = m_normalH.dot(dir);
    if (fabs(tmp) > ZERO)
    {
	double tmp2 = -m_normalH.dot(origin);
	double t1 = (tmp2 - m_d1) / tmp;
	
	if (t1 > 0.0)
	{
	    tf2::Vector3 p1(origin + dir * t1);
	    tf2::Vector3 v1(p1 - m_center);
	    v1 = v1 - m_normalH.dot(v1) * m_normalH;
	    if (v1.length2() < m_radius2 + ZERO)
	    {
		if (intersections == NULL)
		    return true;
		
		detail::intersc ip(p1, t1);
		ipts.push_back(ip);
	    }
	}
	
	double t2 = (tmp2 - m_d2) / tmp;
	if (t2 > 0.0)
	{
	    tf2::Vector3 p2(origin + dir * t2);
	    tf2::Vector3 v2(p2 - m_center);
	    v2 = v2 - m_normalH.dot(v2) * m_normalH;
	    if (v2.length2() < m_radius2 + ZERO)
	    {
		if (intersections == NULL)
		    return true;
		
		detail::intersc ip(p2, t2);
		ipts.push_back(ip);
	    }
	}
    }
    
    if (ipts.size() < 2)
    {
	// intersect with infinite cylinder
	tf2::Vector3 VD(m_normalH.cross(dir));
	tf2::Vector3 ROD(m_normalH.cross(origin - m_center));
	double a = VD.length2();
	double b = 2.0 * ROD.dot(VD);
	double c = ROD.length2() - m_radius2;
	double d = b * b - 4.0 * a * c;
	if (d > 0.0 && fabs(a) > ZERO)
	{
	    d = sqrt(d);
	    double e = -a * 2.0;
	    double t1 = (b + d) / e;
	    double t2 = (b - d) / e;
	    
	    if (t1 > 0.0)
	    {
		tf2::Vector3 p1(origin + dir * t1);
		tf2::Vector3 v1(m_center - p1);
		
		if (fabs(m_normalH.dot(v1)) < m_length2 + ZERO)
		{
		    if (intersections == NULL)
			return true;
		    
		    detail::intersc ip(p1, t1);
		    ipts.push_back(ip);
		}
	    }
	    
	    if (t2 > 0.0)
	    {
		tf2::Vector3 p2(origin + dir * t2);    
		tf2::Vector3 v2(m_center - p2);
		
		if (fabs(m_normalH.dot(v2)) < m_length2 + ZERO)
		{
		    if (intersections == NULL)
			return true;
		    detail::intersc ip(p2, t2);
		    ipts.push_back(ip);
		}
	    }
	}
    }
    
    if (ipts.empty())
	return false;

    std::sort(ipts.begin(), ipts.end(), detail::interscOrder());
    const unsigned int n = count > 0 ? std::min<unsigned int>(count, ipts.size()) : ipts.size();
    for (unsigned int i = 0 ; i < n ; ++i)
	intersections->push_back(ipts[i].pt);
    
    return true;
}

bool bodies::Box::containsPoint(const tf2::Vector3 &p, bool verbose) const 
{
  /*  if(verbose)
      fprintf(stderr,"Actual: %f,%f,%f \nDesired: %f,%f,%f \nTolerance:%f,%f,%f\n",p.x(),p.y(),p.z(),m_center.x(),m_center.y(),m_center.z(),m_length2,m_width2,m_height2);*/
    tf2::Vector3 v = p - m_center;
    double pL = v.dot(m_normalL);
    
    if (fabs(pL) > m_length2)
	return false;
    
    double pW = v.dot(m_normalW);
    
    if (fabs(pW) > m_width2)
	return false;
    
    double pH = v.dot(m_normalH);
    
    if (fabs(pH) > m_height2)
	return false;
    
    return true;
}

void bodies::Box::useDimensions(const shapes::Shape *shape) // (x, y, z) = (length, width, height)
{
    const double *size = static_cast<const shapes::Box*>(shape)->size;
    m_length = size[0];
    m_width  = size[1];
    m_height = size[2];
}

void bodies::Box::updateInternalData(void) 
{
    double s2 = m_scale / 2.0;
    m_length2 = m_length * s2 + m_padding;
    m_width2  = m_width * s2 + m_padding;
    m_height2 = m_height * s2 + m_padding;
    
    m_center  = m_pose.getOrigin();
    
    m_radius2 = m_length2 * m_length2 + m_width2 * m_width2 + m_height2 * m_height2;
    m_radiusB = sqrt(m_radius2);

    const tf2::Matrix3x3& basis = m_pose.getBasis();
    m_normalL = basis.getColumn(0);
    m_normalW = basis.getColumn(1);
    m_normalH = basis.getColumn(2);

    const tf2::Vector3 tmp(m_normalL * m_length2 + m_normalW * m_width2 + m_normalH * m_height2);
    m_corner1 = m_center - tmp;
    m_corner2 = m_center + tmp;
}

double bodies::Box::computeVolume(void) const
{
    return 8.0 * m_length2 * m_width2 * m_height2;
}

void bodies::Box::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

void bodies::Box::getPaddedGeometry(PaddedGeometry &geometry) const
{
    geometry.type = shapes::BOX;
    geometry.size = tf2::Vector3(2.0 * m_length2, 2.0 * m_width2, 2.0 * m_height2);
    geometry.triangles.clear();
}

bool bodies::Box::intersectsRay(const tf2::Vector3& origin, const tf2::Vector3& dir, std::vector<tf2::Vector3> *intersections, unsigned int count) const
{  
    if (distanceSQR(m_center, origin, dir) > m_radius2) return false;

    double t_near = -INFINITY;
    double t_far  = INFINITY;
    
    for (int i = 0; i < 3; i++)
    {
	const tf2::Vector3 &vN = i == 0 ? m_normalL : (i == 1 ? m_normalW : m_normalH);
	double dp = vN.dot(dir);
	
	if (fabs(dp) > ZERO)
	{
	    double t1 = vN.dot(m_corner1 - origin) / dp;
	    double t2 = vN.dot(m_corner2 - origin) / dp;
	    
	    if (t1 > t2)
		std::swap(t1, t2);
	    
	    if (t1 > t_near)
		t_near = t1;
	    
	    if (t2 < t_far)
		t_far = t2;
	    
	    if (t_near > t_far)
		return false;
	    
	    if (t_far < 0.0)
		return false;
	}
	else
	{
	    // ray parallel to this pair of faces: it misses the box if it lies outside the slab
	    const double half = i == 0 ? m_length2 : (i == 1 ? m_width2 : m_height2);
	    if (fabs(vN.dot(origin - m_center)) > half)
		return false;
	}
    }
    
    if (t_far < 0.0)
	return false;
    
    if (intersections)
    {
	// intersections behind the origin are not reported: a ray starting inside the box only
	// leaves it through t_far
	if (t_near > ZERO && t_far - t_near > ZERO)
	{
	    intersections->push_back(t_near * dir + origin);
	    if (count == 0 || count > 1)
		intersections->push_back(t_far  * dir + origin);
	}
	else
	    intersections->push_back(t_far * dir + origin);
    }
    
    return true;
}
/*
bool bodies::Mesh::containsPoint(const tf2::Vector3 &p) const
{
    // compute all intersections
    std::vector<tf2::Vector3> pts;
    intersectsRay(p, tf2::Vector3(0, 0, 1), &pts);
    
    // if we have an odd number of intersections, we are inside
    return pts.size() % 2 == 1;
}

namespace bodies
{
    namespace detail
    {
	// callback for bullet 
	class myTriangleCallback : public btTriangleCallback
	{
	public:
	    
	    myTriangleCallback(bool keep_triangles) : keep_triangles_(keep_triangles)
	    {
		found_intersections = false;
	    }
	    
	    virtual void processTriangle(tf2::Vector3 *triangle, int partId, int triangleIndex)
	    {
		found_intersections = true;
		if (keep_triangles_)
		    triangles.push_back(btTriangleShape(triangle[0], triangle[1], triangle[2]));
		/// \todo figure out if scaling & margins are used for this triangle
	    }
	    
	    bool                         found_intersections;
	    std::vector<btTriangleShape> triangles;
	    
	private:
	    
	    bool                         keep_triangles_;
	};

    }
}

bool bodies::Mesh::intersectsRay(const tf2::Vector3& origin, const tf2::Vector3& dir, std::vector<tf2::Vector3> *intersections, unsigned int count) const
{
    if (m_btMeshShape)
    {
	
	if (distanceSQR(m_center, origin, dir) > m_radiusBSqr) return false;
	
	// transform the ray into the coordinate frame of the mesh
	tf2::Vector3 orig(m_iPose * origin);
	tf2::Vector3 dr(m_iPose.getBasis() * dir);
	
	// find intersection with AABB
	tf2Scalar maxT = 0.0;
	
	if (fabs(dr.x()) > ZERO)
	{
	    tf2Scalar t = (m_aabbMax.x() - orig.x()) / dr.x();
	    if (t < 0.0)
		t = (m_aabbMin.x() - orig.x()) / dr.x();
	    if (t > maxT)
		maxT = t;
	}
	if (fabs(dr.y()) > ZERO)
	{
	    tf2Scalar t = (m_aabbMax.y() - orig.y()) / dr.y();
	    if (t < 0.0)
		t = (m_aabbMin.y() - orig.y()) / dr.y();
	    if (t > maxT)
		maxT = t;
	}
	if (fabs(dr.z()) > ZERO)
	{
	    tf2Scalar t = (m_aabbMax.z() - orig.z()) / dr.z();
	    if (t < 0.0)
		t = (m_aabbMin.z() - orig.z()) / dr.z();
	    if (t > maxT)
		maxT = t;
	}
	
	// the farthest point where we could have an intersection id orig + dr * maxT
	
	detail::myTriangleCallback cb(intersections != NULL);
	m_btMeshShape->performRaycast(&cb, orig, orig + dr * maxT);
	if (cb.found_intersections)
	{
	    if (intersections)
	    {
		std::vector<detail::intersc> intpt;
		for (unsigned int i = 0 ; i < cb.triangles.size() ; ++i)
		{
		    tf2::Vector3 normal;
		    cb.triangles[i].calcNormal(normal);
		    tf2Scalar dv = normal.dot(dr);
		    if (fabs(dv) > 1e-3)
		    {
			double t = (normal.dot(cb.triangles[i].getVertexPtr(0)) - normal.dot(orig)) / dv;			
			// here we use the input origin & direction, since the transform is linear
			detail::intersc ip(origin + dir * t, t);
			intpt.push_back(ip);
		    }
		}
		std::sort(intpt.begin(), intpt.end(), detail::interscOrder());
		const unsigned int n = count > 0 ? std::min<unsigned int>(count, intpt.size()) : intpt.size();
		for (unsigned int i = 0 ; i < n ; ++i)
		    intersections->push_back(intpt[i].pt);
	    }
	    return true;	    
	}
	else
	    return false;
    }
    else
	return false;
}   

void bodies::Mesh::useDimensions(const shapes::Shape *shape)
{  
    const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
    if (m_btMeshShape)
	delete m_btMeshShape;
    if (m_btMesh)
	delete m_btMesh;
    
    m_btMesh = new btTriangleMesh();
    const unsigned int nt = mesh->triangleCount / 3;
    for (unsigned int i = 0 ; i < nt ; ++i)
    {
	const unsigned int i3 = i *3;
	const unsigned int v1 = 3 * mesh->triangles[i3];
	const unsigned int v2 = 3 * mesh->triangles[i3 + 1];
	const unsigned int v3 = 3 * mesh->triangles[i3 + 2];
	m_btMesh->addTriangle(tf2::Vector3(mesh->vertices[v1], mesh->vertices[v1 + 1], mesh->vertices[v1 + 2]),
			      tf2::Vector3(mesh->vertices[v2], mesh->vertices[v2 + 1], mesh->vertices[v2 + 2]),
			      tf2::Vector3(mesh->vertices[v3], mesh->vertices[v3 + 1], mesh->vertices[v3 + 2]), true);
    }
    
    m_btMeshShape = new btBvhTriangleMeshShape(m_btMesh, true);
}

void bodies::Mesh::updateInternalData(void) 
{
    if (m_btMeshShape)
    {
	m_btMeshShape->setLocalScaling(tf2::Vector3(m_scale, m_scale, m_scale));
	m_btMeshShape->setMargin(m_padding);
    }
    m_iPose = m_pose.inverse();
    
    tf::Transform id;
    id.setIdentity();
    m_btMeshShape->getAabb(id, m_aabbMin, m_aabbMax);

    tf2::Vector3 d = (m_aabbMax - m_aabbMin) / 2.0;

    m_center = m_pose.getOrigin() + d;
    m_radiusBSqr = d.length2();
    m_radiusB = sqrt(m_radiusBSqr);
    
    /// \todo check if the AABB computation includes padding & scaling; if not, include it
}

double bodies::Mesh::computeVolume(void) const
{
    if (m_btMeshShape)
    {
	// approximation; volume of AABB at identity transform
	tf2::Vector3 d = m_aabbMax - m_aabbMin;
	return d.x() * d.y() * d.z();
    }
    else
	return 0.0;
}

void bodies::Mesh::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

*/

bool bodies::ConvexMesh::containsPoint(const tf2::Vector3 &p, bool verbose) const
{
    if (m_boundingBox.containsPoint(p))
    {
	return isPointInsidePlanes(m_iPose * p);
    }
    else
	return false;
}

void bodies::ConvexMesh::useDimensions(const shapes::Shape *shape)
{  
    const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);

    double maxX = -INFINITY, maxY = -INFINITY, maxZ = -INFINITY;
    double minX =  INFINITY, minY =  INFINITY, minZ  = INFINITY;
    
    for(unsigned int i = 0; i < mesh->vertexCount ; ++i)
    {
	double vx = mesh->vertices[3 * i    ];
	double vy = mesh->vertices[3 * i + 1];
	double vz = mesh->vertices[3 * i + 2];
	
	if (maxX < vx) maxX = vx;
	if (maxY < vy) maxY = vy;
	if (maxZ < vz) maxZ = vz;
	
	if (minX > vx) minX = vx;
	if (minY > vy) minY = vy;
	if (minZ > vz) minZ = vz;
    }
    
    if (maxX < minX) maxX = minX = 0.0;
    if (maxY < minY) maxY = minY = 0.0;
    if (maxZ < minZ) maxZ = minZ = 0.0;
    
    shapes::Box *box_shape = new shapes::Box(maxX - minX, maxY - minY, maxZ - minZ);
    m_boundingBox.setDimensions(box_shape);
    delete box_shape;
    
    m_boxOffset.setValue((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);
    
    m_planes.clear();
    m_triangles.clear();
    m_vertices.clear();
    m_meshRadiusB = 0.0;
    m_meshCenter.setValue(tf2Scalar(0), tf2Scalar(0), tf2Scalar(0));

    btVector3 *vertices = new btVector3[mesh->vertexCount];
    for(unsigned int i = 0; i < mesh->vertexCount ; ++i)
    {
	vertices[i].setX(mesh->vertices[3 * i    ]);
	vertices[i].setY(mesh->vertices[3 * i + 1]);
	vertices[i].setZ(mesh->vertices[3 * i + 2]);
    }
    
    HullDesc hd(QF_TRIANGLES, mesh->vertexCount, vertices);
    HullResult hr;
    HullLibrary hl;
    if (hl.CreateConvexHull(hd, hr) == QE_OK)
    {
	//	std::cout << "Convex hull has " << hr.m_OutputVertices.size() << " vertices (down from " << mesh->vertexCount << "), " << hr.mNumFaces << " faces" << std::endl;

	m_vertices.reserve(hr.m_OutputVertices.size());
	tf2::Vector3 sum(0, 0, 0);	
	
	for (int j = 0 ; j < hr.m_OutputVertices.size() ; ++j)
	{
            tf2::Vector3 vector3_tmp(tf2::Vector3(hr.m_OutputVertices[j][0],hr.m_OutputVertices[j][1],hr.m_OutputVertices[j][2]));
	    m_vertices.push_back(vector3_tmp);
	    sum = sum + vector3_tmp;
	}
	
	m_meshCenter = sum / (double)(hr.m_OutputVertices.size());
	for (unsigned int j = 0 ; j < m_vertices.size() ; ++j)
	{
	    double dist = m_vertices[j].distance2(m_meshCenter);
	    if (dist > m_meshRadiusB)
		m_meshRadiusB = dist;
	}
	m_meshRadiusB = sqrt(m_meshRadiusB);
	
	m_triangles.reserve(hr.m_Indices.size());
	for (unsigned int j = 0 ; j < hr.mNumFaces ; ++j)
	{
	    const tf2::Vector3 p1(hr.m_OutputVertices[hr.m_Indices[j * 3    ]][0],hr.m_OutputVertices[hr.m_Indices[j * 3    ]][1],hr.m_OutputVertices[hr.m_Indices[j * 3    ]][2]);
	    const tf2::Vector3 p2(hr.m_OutputVertices[hr.m_Indices[j * 3 + 1]][0],hr.m_OutputVertices[hr.m_Indices[j * 3 + 1]][1],hr.m_OutputVertices[hr.m_Indices[j * 3 + 1]][2]);
	    const tf2::Vector3 p3(hr.m_OutputVertices[hr.m_Indices[j * 3 + 2]][0],hr.m_OutputVertices[hr.m_Indices[j * 3 + 2]][1],hr.m_OutputVertices[hr.m_Indices[j * 3 + 2]][2]);
	    
	    tf2::Vector3 edge1 = (p2 - p1);
	    tf2::Vector3 edge2 = (p3 - p1);
	    
	    edge1.normalize();
	    edge2.normalize();

	    tf2::Vector3 planeNormal = edge1.cross(edge2);
	    
	    if (planeNormal.length2() > tf2Scalar(1e-6))
	    {
		planeNormal.normalize();
		tf2::tf2Vector4 planeEquation(planeNormal.getX(), planeNormal.getY(), planeNormal.getZ(), -planeNormal.dot(p1));

		unsigned int behindPlane = countVerticesBehindPlane(planeEquation);
		if (behindPlane > 0)
		{
		    tf2::tf2Vector4 planeEquation2(-planeEquation.getX(), -planeEquation.getY(), -planeEquation.getZ(), -planeEquation.getW());
		    unsigned int behindPlane2 = countVerticesBehindPlane(planeEquation2);
		    if (behindPlane2 < behindPlane)
		    {
			planeEquation.setValue(planeEquation2.getX(), planeEquation2.getY(), planeEquation2.getZ(), planeEquation2.getW());
			behindPlane = behindPlane2;
		    }
		}
		
		//		if (behindPlane > 0)
		//		    std::cerr << "Approximate plane: " << behindPlane << " of " << m_vertices.size() << " points are behind the plane" << std::endl;
		
		m_planes.push_back(planeEquation);

		m_triangles.push_back(hr.m_Indices[j * 3 + 0]);
		m_triangles.push_back(hr.m_Indices[j * 3 + 1]);
		m_triangles.push_back(hr.m_Indices[j * 3 + 2]);
	    }
	}
    }
    else
    	std::cerr << "Unable to compute convex hull.";
    
    hl.ReleaseResult(hr);    
    delete[] vertices;
    
}

void bodies::ConvexMesh::updateInternalData(void) 
{
    tf2::Transform pose = m_pose;
    pose.setOrigin(m_pose * m_boxOffset);
    m_boundingBox.setPose(pose);
    m_boundingBox.setPadding(m_padding);
    m_boundingBox.setScale(m_scale);
    
    m_iPose = m_pose.inverse();
    m_center = m_pose * m_meshCenter;
    m_radiusB = m_meshRadiusB * m_scale + m_padding;
    m_radiusBSqr = m_radiusB * m_radiusB;

}

void bodies::ConvexMesh::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

/* The padded body is the region containsPoint() accepts: the intersection of the padded face
   half-spaces and of the padded bounding box (which cuts off the long spikes the padded faces
   would make at sharp corners). Each face is drawn as its plane clipped by all the other
   planes, so neighbouring faces meet without gaps (edges and corners are mitred, not
   rounded, like in the point test). */
void bodies::ConvexMesh::getPaddedGeometry(PaddedGeometry &geometry) const
{
    geometry.type = shapes::MESH;
    geometry.size = tf2::Vector3(1, 1, 1);
    geometry.triangles.clear();
    if (m_vertices.empty())
	return;

    std::vector<tf2::Vector3> normals;
    std::vector<double> offsets;
    for (unsigned int i = 0 ; i < m_planes.size() ; ++i)
    {
	normals.push_back(tf2::Vector3(m_planes[i].getX(), m_planes[i].getY(), m_planes[i].getZ()));
	offsets.push_back(faceSurfaceOffset(i));
    }

    // the padded bounding box used as early-out in containsPoint(), in the mesh frame
    tf2::Vector3 lo = m_vertices[0], hi = m_vertices[0];
    for (const tf2::Vector3 &v : m_vertices)
    {
	lo.setValue(std::min(lo.x(), v.x()), std::min(lo.y(), v.y()), std::min(lo.z(), v.z()));
	hi.setValue(std::max(hi.x(), v.x()), std::max(hi.y(), v.y()), std::max(hi.z(), v.z()));
    }
    const tf2::Vector3 boxCenter = (lo + hi) / 2.0;
    const tf2::Vector3 boxHalf = (hi - lo) * (m_scale / 2.0) + tf2::Vector3(m_padding, m_padding, m_padding);
    for (int axis = 0 ; axis < 3 ; ++axis)
	for (int sign = -1 ; sign <= 1 ; sign += 2)
	{
	    tf2::Vector3 n(0, 0, 0);
	    n[axis] = sign;
	    normals.push_back(n);
	    offsets.push_back(n.dot(boxCenter) + boxHalf[axis]);
	}

    std::vector<unsigned int> faces;   // distinct planes (a flat face is made of several triangles)
    for (unsigned int i = 0 ; i < normals.size() ; ++i)
    {
	bool duplicate = false;
	for (unsigned int k : faces)
	    if (normals[i].dot(normals[k]) > 1.0 - 1e-6 && fabs(offsets[i] - offsets[k]) < 1e-6)
	    {
		duplicate = true;
		break;
	    }
	if (!duplicate)
	    faces.push_back(i);
    }

    const double big = 4.0 * (m_meshRadiusB * m_scale + m_padding) + 1.0;
    for (unsigned int i : faces)
    {
	const tf2::Vector3 &n = normals[i];
	// a big square on the plane of face i
	tf2::Vector3 u = fabs(n.x()) < 0.9 ? tf2::Vector3(1, 0, 0) : tf2::Vector3(0, 1, 0);
	u = (u - n * n.dot(u)).normalized();
	const tf2::Vector3 v = n.cross(u);
	const tf2::Vector3 c = m_meshCenter + n * (offsets[i] - n.dot(m_meshCenter));
	std::vector<tf2::Vector3> poly = {c + (u + v) * big, c + (v - u) * big, c - (u + v) * big, c + (u - v) * big};

	// clip it by the half-space of every other face
	for (unsigned int j : faces)
	{
	    if (j == i || poly.empty())
		continue;
	    std::vector<tf2::Vector3> clipped;
	    for (size_t k = 0 ; k < poly.size() ; ++k)
	    {
		const tf2::Vector3 &p0 = poly[k];
		const tf2::Vector3 &p1 = poly[(k + 1) % poly.size()];
		const double d0 = normals[j].dot(p0) - offsets[j];   // > 0 is outside
		const double d1 = normals[j].dot(p1) - offsets[j];
		if (d0 <= 0.0)
		    clipped.push_back(p0);
		if ((d0 < 0.0 && d1 > 0.0) || (d0 > 0.0 && d1 < 0.0))
		    clipped.push_back(p0 + (p1 - p0) * (d0 / (d0 - d1)));
	    }
	    poly.swap(clipped);
	}

	for (size_t k = 1 ; k + 1 < poly.size() ; ++k)
	{
	    geometry.triangles.push_back(poly[0]);
	    geometry.triangles.push_back(poly[k]);
	    geometry.triangles.push_back(poly[k + 1]);
	}
    }
}

/* Offset of the padded + scaled surface of face i along the face normal: the surface is
   the face plane scaled by m_scale about the mesh center and then moved out by m_padding
   along the face normal, so every face is padded by the same amount. */
double bodies::ConvexMesh::faceSurfaceOffset(unsigned int i) const
{
    const tf2::tf2Vector4& plane = m_planes[i];
    const tf2::Vector3 n(plane.getX(), plane.getY(), plane.getZ());
    const double nc = n.dot(m_meshCenter);
    const double faceOffset = -plane.getW();            // n . x = faceOffset on the face
    return nc + m_scale * (faceOffset - nc) + m_padding;
}

/* Signed distance of a point (mesh frame) above the padded + scaled surface of face i:
   positive when outside. */
double bodies::ConvexMesh::signedDistanceToFace(unsigned int i, const tf2::Vector3& point) const
{
    const tf2::tf2Vector4& plane = m_planes[i];
    return tf2::Vector3(plane.getX(), plane.getY(), plane.getZ()).dot(point) - faceSurfaceOffset(i);
}

bool bodies::ConvexMesh::isPointInsidePlanes(const tf2::Vector3& point) const
{
    const unsigned int numplanes = m_planes.size();
    for (unsigned int i = 0 ; i < numplanes ; ++i)
	if (signedDistanceToFace(i, point) > tf2Scalar(1e-6))
	    return false;
    return true;
}

unsigned int bodies::ConvexMesh::countVerticesBehindPlane(const tf2::tf2Vector4& planeNormal) const
{
    unsigned int numvertices = m_vertices.size();
    unsigned int result = 0;
    for (unsigned int i = 0 ; i < numvertices ; ++i)
    {
	tf2Scalar dist = planeNormal.dot(m_vertices[i]) + planeNormal.getW() - tf2Scalar(1e-6);
	if (dist > tf2Scalar(0))
	    result++;
    }
    return result;
}

double bodies::ConvexMesh::computeVolume(void) const
{
    double volume = 0.0;
    for (unsigned int i = 0 ; i < m_triangles.size() / 3 ; ++i)
    {
	const tf2::Vector3 &v1 = m_vertices[m_triangles[3*i + 0]];
        const tf2::Vector3 &v2 = m_vertices[m_triangles[3*i + 1]];
	const tf2::Vector3 &v3 = m_vertices[m_triangles[3*i + 2]];
	volume += v1.x()*v2.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() - v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() - v3.x()*v2.y()*v1.z();
    }
    return fabs(volume)/6.0;
}

bool bodies::ConvexMesh::intersectsRay(const tf2::Vector3& origin, const tf2::Vector3& dir, std::vector<tf2::Vector3> *intersections, unsigned int count) const
{
    if (distanceSQR(m_center, origin, dir) > m_radiusBSqr) return false;
    if (!m_boundingBox.intersectsRay(origin, dir)) return false;
    
    // transform the ray into the coordinate frame of the mesh
    const tf2::Vector3 orig(m_iPose * origin);
    const tf2::Vector3 dr(m_iPose.getBasis() * dir);
    
    // clip the ray against the padded half-spaces of every face, which is exactly
    // the region containsPoint() accepts
    double tNear = -INFINITY;
    double tFar  =  INFINITY;
    const unsigned int numplanes = m_planes.size();
    for (unsigned int i = 0 ; i < numplanes ; ++i)
    {
	const tf2::tf2Vector4& plane = m_planes[i];
	const double denom = tf2::Vector3(plane.getX(), plane.getY(), plane.getZ()).dot(dr);
	const double dist = signedDistanceToFace(i, orig);   // > 0 when the origin is outside this face
	if (fabs(denom) < ZERO)
	{
	    if (dist > 0.0)
		return false;
	    continue;
	}
	const double t = -dist / denom;
	if (denom < 0.0)
	    tNear = std::max(tNear, t);   // entering the half-space
	else
	    tFar = std::min(tFar, t);     // leaving the half-space
	if (tNear > tFar)
	    return false;
    }
    if (numplanes == 0 || tFar <= 0.0)
	return false;
    
    if (intersections)
    {
	std::vector<double> ts;
	if (tNear > 0.0)
	    ts.push_back(tNear);
	if (std::isfinite(tFar))
	    ts.push_back(tFar);
	const unsigned int n = count > 0 ? std::min<unsigned int>(count, ts.size()) : ts.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    intersections->push_back(origin + dir * ts[i]);
    }
    return true;
}
  
}
