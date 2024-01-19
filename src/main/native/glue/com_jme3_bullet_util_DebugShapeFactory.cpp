/*
 * Copyright (c) 2009-2012 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Normen Hansen, CJ Hare
 */
#include "com_jme3_bullet_util_DebugShapeFactory.h"
#include "jmeBulletUtil.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

class DebugCallback : public btTriangleCallback, public btInternalTriangleIndexCallback {
    JNIEnv *m_pEnv;
    jobject m_callback;
public:
    /*
     * constructor:
     */
    DebugCallback(JNIEnv *pEnv, jobject object) {
        m_pEnv = pEnv;
        m_callback = object;
    }

    virtual void internalProcessTriangleIndex(btVector3* triangle,
            int partId, int triangleIndex) {
        processTriangle(triangle, partId, triangleIndex);
    }

    virtual void processTriangle(btVector3* triangle,
            int partId, int triangleIndex) {
        btVector3 vertexA, vertexB, vertexC;
        vertexA = triangle[0];
        vertexB = triangle[1];
        vertexC = triangle[2];
        m_pEnv->CallVoidMethod(m_callback, jmeClasses::DebugMeshCallback_addVector,
                vertexA.getX(), vertexA.getY(), vertexA.getZ(),
                partId, triangleIndex);
        EXCEPTION_CHK(m_pEnv,);
        m_pEnv->CallVoidMethod(m_callback, jmeClasses::DebugMeshCallback_addVector,
                vertexB.getX(), vertexB.getY(), vertexB.getZ(),
                partId, triangleIndex);
        EXCEPTION_CHK(m_pEnv,);
        m_pEnv->CallVoidMethod(m_callback, jmeClasses::DebugMeshCallback_addVector,
                vertexC.getX(), vertexC.getY(), vertexC.getZ(),
                partId, triangleIndex);
        // no check for exceptions!
    }
};

/*
 * Class:     com_jme3_bullet_util_DebugShapeFactory
 * Method:    getTriangles
 * Signature: (JILcom/jme3/bullet/util/DebugMeshCallback;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_DebugShapeFactory_getTriangles
(JNIEnv *pEnv, jclass, jlong shapeId, jint resolution, jobject callback) {
    const btCollisionShape * const
            pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", JNI_FALSE);

    if (pShape->isConcave()) {
        const btConcaveShape * const pConcave = (btConcaveShape *) pShape;

        DebugCallback * const
                pCallback = new DebugCallback(pEnv, callback); //dance028
        btVector3 min = btVector3(-1e30, -1e30, -1e30);
        btVector3 max = btVector3(1e30, 1e30, 1e30);
        pConcave->processAllTriangles(pCallback, min, max);
        delete pCallback; //dance028

    } else if (pShape->isConvex()) {
        const btConvexShape * const pConvex = (btConvexShape *) pShape;

        // Create a hull approximation.
        btShapeHull * const pHull = new btShapeHull(pConvex); //dance027
        float margin = pConvex->getMargin();
        bool success = pHull->buildHull(margin, resolution);
        if (!success) {
            delete pHull; //dance027
            return JNI_FALSE;
        }

        int numberOfTriangles = pHull->numTriangles();
        const unsigned int * const pHullIndices = pHull->getIndexPointer();
        const btVector3 *pHullVertices = pHull->getVertexPointer();
        btVector3 vertexA, vertexB, vertexC;
        int index = 0;

        for (int i = 0; i < numberOfTriangles; i++) {
            // Copy the triangle's vertices from the hull.
            vertexA = pHullVertices[pHullIndices[index++]];
            vertexB = pHullVertices[pHullIndices[index++]];
            vertexC = pHullVertices[pHullIndices[index++]];

            // Add the vertices to the callback object.
            pEnv->CallVoidMethod(callback,
                    jmeClasses::DebugMeshCallback_addVector, vertexA.getX(),
                    vertexA.getY(), vertexA.getZ());
            EXCEPTION_CHK(pEnv, JNI_FALSE);

            pEnv->CallVoidMethod(callback,
                    jmeClasses::DebugMeshCallback_addVector, vertexB.getX(),
                    vertexB.getY(), vertexB.getZ());
            EXCEPTION_CHK(pEnv, JNI_FALSE);

            pEnv->CallVoidMethod(callback,
                    jmeClasses::DebugMeshCallback_addVector, vertexC.getX(),
                    vertexC.getY(), vertexC.getZ());
            EXCEPTION_CHK(pEnv, JNI_FALSE);
        }
        delete pHull; //dance027
    }

    return JNI_TRUE; // success!
}

/*
 * Class:     com_jme3_bullet_util_DebugShapeFactory
 * Method:    getVertices
 * Signature: (JILcom/jme3/bullet/util/DebugMeshCallback;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_DebugShapeFactory_getVertices
(JNIEnv *pEnv, jclass, jlong shapeId, jint resolution, jobject callback) {
    const btCollisionShape * const
            pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", JNI_FALSE);

    if (pShape->isConcave()) {
        const btConcaveShape * const pConcave = (btConcaveShape *) pShape;

        DebugCallback * const
                pCallback = new DebugCallback(pEnv, callback); //dance025
        btVector3 min = btVector3(-1e30, -1e30, -1e30);
        btVector3 max = btVector3(1e30, 1e30, 1e30);
        pConcave->processAllTriangles(pCallback, min, max);
        delete pCallback; //dance025

    } else if (pShape->isConvex()) {
        const btConvexShape * const pConvex = (btConvexShape *) pShape;

        // Create a hull approximation.
        btShapeHull * const pHull = new btShapeHull(pConvex); //dance026
        float margin = pConvex->getMargin();
        bool success = pHull->buildHull(margin, resolution);
        if (!success) {
            delete pHull; //dance026
            return JNI_FALSE;
        }

        int numberOfVertices = pHull->numVertices();
        const btVector3 * const pHullVertices = pHull->getVertexPointer();
        for (int i = 0; i < numberOfVertices; i++) {
            // Copy the vertex from the hull.
            btVector3 vertex = pHullVertices[i];

            // Add the vertex to the callback object.
            pEnv->CallVoidMethod(callback,
                    jmeClasses::DebugMeshCallback_addVector, vertex.getX(),
                    vertex.getY(), vertex.getZ());
            EXCEPTION_CHK(pEnv, JNI_FALSE);
        }
        delete pHull; //dance026
    }

    return JNI_TRUE; // success!
}
