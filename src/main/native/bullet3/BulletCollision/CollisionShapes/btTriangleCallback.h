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

#ifndef BT_TRIANGLE_CALLBACK_H
#define BT_TRIANGLE_CALLBACK_H

#include "BulletCollision/CollisionShapes/btTriangleShape.h"// stephengold changed 2021-11-04

///The btTriangleCallback provides a callback for each overlapping triangle when calling processAllTriangles.
///This callback is called by processAllTriangles for all btConcaveShape derived class, such as  btBvhTriangleMeshShape, btStaticPlaneShape and btHeightfieldTerrainShape.
class btTriangleCallback
{
public:
	virtual ~btTriangleCallback();
	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex) = 0;
};

class btInternalTriangleIndexCallback
{
public:
	virtual ~btInternalTriangleIndexCallback();
	virtual void internalProcessTriangleIndex(btVector3* triangle, int partId, int triangleIndex) = 0;
};

class FilteredInteriorCountCallback : public btTriangleCallback// stephengold added 2021-11-04
{// stephengold added 2021-11-04
	const btVector3& m_local;// stephengold added 2021-11-04
	const int m_partId, m_triangleIndex;// stephengold added 2021-11-04
	const btScalar m_margin;// stephengold added 2021-11-04
public:// stephengold added 2021-11-04
	int m_interiorCount;// stephengold added 2021-11-04
	FilteredInteriorCountCallback(const btVector3& local, int partId, int triangleIndex, btScalar margin)// stephengold added 2021-11-04
	: m_local(local), m_partId(partId), m_triangleIndex(triangleIndex),// stephengold added 2021-11-04
	  m_margin(margin), m_interiorCount(0)// stephengold added 2021-11-04
	{// stephengold added 2021-11-04
	}// stephengold added 2021-11-04
	virtual void processTriangle(btVector3* pTriangle, int partId, int triangleIndex)// stephengold added 2021-11-04
	{// stephengold added 2021-11-04
 		if (partId == m_partId && triangleIndex == m_triangleIndex) return;// stephengold added 2021-11-04
		btTriangleShape triangleShape(pTriangle[0], pTriangle[1], pTriangle[2]);// stephengold added 2021-11-04
		bool isInside = triangleShape.isInside(m_local, m_margin);// stephengold added 2021-11-04
		if (isInside) ++m_interiorCount;// stephengold added 2021-11-04
	}// stephengold added 2021-11-04
};// stephengold added 2021-11-04
#endif  //BT_TRIANGLE_CALLBACK_H
