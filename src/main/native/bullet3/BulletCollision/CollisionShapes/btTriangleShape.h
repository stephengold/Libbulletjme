/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_OBB_TRIANGLE_MINKOWSKI_H
#define BT_OBB_TRIANGLE_MINKOWSKI_H

#include "btConvexShape.h"
#include "btBoxShape.h"

ATTRIBUTE_ALIGNED16(class)
btTriangleShape : public btPolyhedralConvexShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btVector3 m_vertices1[3];

	virtual int getNumVertices() const
	{
		return 3;
	}

	btVector3& getVertexPtr(int index)
	{
		return m_vertices1[index];
	}

	const btVector3& getVertexPtr(int index) const
	{
		return m_vertices1[index];
	}
	virtual void getVertex(int index, btVector3& vert) const
	{
		vert = m_vertices1[index];
	}

	virtual int getNumEdges() const
	{
		return 3;
	}

	virtual void getEdge(int i, btVector3& pa, btVector3& pb) const
	{
		getVertex(i, pa);
		getVertex((i + 1) % 3, pb);
	}

	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		//		btAssert(0);
		getAabbSlow(t, aabbMin, aabbMax);
	}

	btVector3 localGetSupportingVertexWithoutMargin(const btVector3& dir) const
	{
		btVector3 dots = dir.dot3(m_vertices1[0], m_vertices1[1], m_vertices1[2]);
		return m_vertices1[dots.maxAxis()];
	}

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
	{
		for (int i = 0; i < numVectors; i++)
		{
			const btVector3& dir = vectors[i];
			btVector3 dots = dir.dot3(m_vertices1[0], m_vertices1[1], m_vertices1[2]);
			supportVerticesOut[i] = m_vertices1[dots.maxAxis()];
		}
	}

	btTriangleShape() : btPolyhedralConvexShape()
	{
		m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
	}

	btTriangleShape(const btVector3& p0, const btVector3& p1, const btVector3& p2) : btPolyhedralConvexShape()
	{
		m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
		m_vertices1[0] = p0;
		m_vertices1[1] = p1;
		m_vertices1[2] = p2;
	}

	virtual void getPlane(btVector3 & planeNormal, btVector3 & planeSupport, int i) const
	{
		getPlaneEquation(i, planeNormal, planeSupport);
	}

	virtual int getNumPlanes() const
	{
		return 1;
	}

	void calcNormal(btVector3 & normal) const
	{
		normal = (m_vertices1[1] - m_vertices1[0]).cross(m_vertices1[2] - m_vertices1[0]);
		normal.safeNormalize();// stephengold changed 2021-11-26
	}

	virtual void getPlaneEquation(int i, btVector3& planeNormal, btVector3& planeSupport) const
	{
		(void)i;
		calcNormal(planeNormal);
		planeSupport = m_vertices1[0];
	}

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		(void)mass;
		btAssert(0);
		inertia.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
	}

	virtual bool isInside(const btVector3& pt, btScalar tolerance) const
	{
		btVector3 p0 = m_vertices1[0];// stephengold changed 2021-11-27
		btVector3 p1 = m_vertices1[1];// stephengold changed 2021-11-27
		btVector3 p2 = m_vertices1[2];// stephengold changed 2021-11-27
		btVector3 normal = (p1 - p0).cross(p2 - p0);// stephengold changed 2021-11-27
		btScalar l2 = normal.length2();// stephengold changed 2021-11-27
		bool haveNormal = (l2 >= SIMD_EPSILON * SIMD_EPSILON);// stephengold changed 2021-11-27
		if (haveNormal) {// stephengold changed 2021-11-27
			normal /= btSqrt(l2);// stephengold changed 2021-11-27
			btScalar dist = (pt - p0).dot(normal);// stephengold changed 2021-11-27
			if (dist < -tolerance || dist > tolerance) return false;// stephengold changed 2021-11-27
		}// stephengold changed 2021-11-27
		btScalar tol2 = tolerance * tolerance;// stephengold changed 2021-11-27
		if (pt.distance2(p0) <= tol2) return true;// stephengold changed 2021-11-27
		if (pt.distance2(p1) <= tol2) return true;// stephengold changed 2021-11-27
		if (pt.distance2(p2) <= tol2) return true;// stephengold changed 2021-11-27
		for (int i = 0; i < 3; ++i)// stephengold changed 2021-11-27
		{
			btVector3 pa, pb;// stephengold changed 2021-11-27
			getEdge(i, pa, pb);// stephengold changed 2021-11-27
			btVector3 edge = pb - pa;// stephengold changed 2021-11-27
			btVector3 offset = pt - pa;// stephengold changed 2021-11-27
			if (haveNormal) {// stephengold changed 2021-11-27
				btVector3 edgeNormal = edge.cross(normal);// stephengold changed 2021-11-27
				l2 = edgeNormal.length2();// stephengold changed 2021-11-27
				if (l2 >= SIMD_EPSILON * SIMD_EPSILON) {// stephengold changed 2021-11-27
					edgeNormal /= btSqrt(l2);// stephengold changed 2021-11-27
					btScalar dist = offset.dot(edgeNormal);// stephengold changed 2021-11-27
					if (dist > tolerance) return false;// stephengold changed 2021-11-27
					if (dist <= btScalar(0.)) continue;// stephengold changed 2021-11-27
				}// stephengold changed 2021-11-27
			}
			l2 = edge.length2();// stephengold changed 2021-11-27
			if (l2 >= SIMD_EPSILON * SIMD_EPSILON) {// stephengold changed 2021-11-27
				btScalar dot = offset.dot(edge);// stephengold changed 2021-11-27
				if (dot < btScalar(0.) || dot > l2) return false;// stephengold changed 2021-11-27
				btVector3 projection = (dot / l2) * edge;// stephengold changed 2021-11-27
				if (offset.distance2(projection) > tol2) return false;// stephengold changed 2021-11-27
			}// stephengold changed 2021-11-27
		}
		return true;// stephengold changed 2021-11-27
	}
	//debugging
	virtual const char* getName() const
	{
		return "Triangle";
	}

	virtual int getNumPreferredPenetrationDirections() const
	{
		return 2;
	}

	virtual void getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const
	{
		calcNormal(penetrationVector);
		if (index)
			penetrationVector *= btScalar(-1.);
	}
};

#endif  //BT_OBB_TRIANGLE_MINKOWSKI_H
