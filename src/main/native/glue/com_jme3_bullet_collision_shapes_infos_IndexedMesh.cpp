/*
 * Copyright (c) 2019-2024 jMonkeyEngine
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
 * Author: Stephen Gold
 */
#include "com_jme3_bullet_collision_shapes_infos_IndexedMesh.h"
#include "jmeBulletUtil.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    countTriangles
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_countTriangles
(JNIEnv *pEnv, jclass, jlong meshId) {
    btIndexedMesh * const pMesh = reinterpret_cast<btIndexedMesh *> (meshId);
    NULL_CHK(pEnv, pMesh, "The btIndexedMesh does not exist.", 0);

    jint result = pMesh->m_numTriangles;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    countVertices
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_countVertices
(JNIEnv *pEnv, jclass, jlong meshId) {
    btIndexedMesh * const pMesh = reinterpret_cast<btIndexedMesh *> (meshId);
    NULL_CHK(pEnv, pMesh, "The btIndexedMesh does not exist.", 0);

    jint result = pMesh->m_numVertices;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    createByte
 * Signature: (Ljava/nio/ByteBuffer;Ljava/nio/FloatBuffer;IIII)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_createByte
(JNIEnv *pEnv, jclass, jobject byteBuffer, jobject floatBuffer,
        jint numTriangles, jint numVertices, jint vertexStride,
        jint indexStride) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, byteBuffer, "The index buffer does not exist.", 0);
    const unsigned char * const pIndices
            = (unsigned char *) pEnv->GetDirectBufferAddress(byteBuffer);
    NULL_CHK(pEnv, pIndices, "The index buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, floatBuffer, "The position buffer does not exist.", 0);
    const unsigned char * const pVertices
            = (unsigned char *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pVertices, "The position buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    btIndexedMesh * const pMesh = new btIndexedMesh(); //dance020
    pMesh->m_indexType = PHY_UCHAR;
    pMesh->m_vertexType = PHY_FLOAT;
    pMesh->m_triangleIndexBase = pIndices;
    pMesh->m_vertexBase = pVertices;
    pMesh->m_numTriangles = numTriangles;
    pMesh->m_numVertices = numVertices;
    pMesh->m_vertexStride = vertexStride;
    pMesh->m_triangleIndexStride = indexStride;

    return reinterpret_cast<jlong> (pMesh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    createInt
 * Signature: (Ljava/nio/IntBuffer;Ljava/nio/FloatBuffer;IIII)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_createInt
(JNIEnv *pEnv, jclass, jobject intBuffer, jobject floatBuffer,
        jint numTriangles, jint numVertices, jint vertexStride,
        jint indexStride) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, intBuffer, "The index buffer does not exist.", 0);
    const unsigned char * const pIndices
            = (unsigned char *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pIndices, "The index buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, floatBuffer, "The position buffer does not exist.", 0);
    const unsigned char * const pVertices
            = (unsigned char *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pVertices, "The position buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    btIndexedMesh * const pMesh = new btIndexedMesh(); //dance020
    pMesh->m_indexType = PHY_INTEGER;
    pMesh->m_vertexType = PHY_FLOAT;
    pMesh->m_triangleIndexBase = pIndices;
    pMesh->m_vertexBase = pVertices;
    pMesh->m_numTriangles = numTriangles;
    pMesh->m_numVertices = numVertices;
    pMesh->m_vertexStride = vertexStride;
    pMesh->m_triangleIndexStride = indexStride;

    return reinterpret_cast<jlong> (pMesh);
}

// A callback to count the triangles of a concave shape:

class countingCallback : public btTriangleCallback {
public:
    jint m_count; // number of triangles processed
    /*
     * constructor:
     */
    countingCallback() {
        m_count = 0;
    }

    void
    processTriangle(btVector3* pTriangle, int partId, int triangleIndex) {
        ++m_count;
    }
};

// A callback to copy the triangles of a concave shape:

class copyingCallback : public btTriangleCallback {
    float *m_pDest;   // the destination array
    jint m_count;     // number of triangles in m_pDest
    jint m_i;         // number of triangles copied
public:
    /*
     * constructor:
     */
    copyingCallback(jint count, float *pDest) {
        m_pDest = pDest;
        m_count = count;
        m_i = 0;
    }

    void
    processTriangle(btVector3 *pTriangle, int partId, int triangleIndex) {
        btAssert(m_i < m_count);
        for (int i = 0; i < 3; ++i) {
            btVector3 v = pTriangle[i];
            m_pDest[9 * m_i + 3 * i] = v.x();
            m_pDest[9 * m_i + 3 * i + 1] = v.y();
            m_pDest[9 * m_i + 3 * i + 2] = v.z();
        }
        ++m_i;
    }
};

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    createIntDebug
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_createIntDebug
(JNIEnv *pEnv, jclass, jlong shapeId, jint meshResolution) {
    const btCollisionShape * const pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", 0);

    btAssert(meshResolution == 0 || meshResolution == 1);

    btIndexedMesh * const pMesh = new btIndexedMesh(); //dance020
    assert(sizeof(jint) == 4);
    pMesh->m_vertexType = PHY_FLOAT;
    assert(sizeof(float) == 4);
    pMesh->m_vertexStride = 12;
    pMesh->m_triangleIndexStride = 12;
    pMesh->m_ownArrays = true;

    if (pShape->isConcave()) { // meshResolution is ignored.
        const btConcaveShape * const pConcave = (btConcaveShape *) pShape;

        const btVector3 min = btVector3(-1e30, -1e30, -1e30);
        const btVector3 max = btVector3(1e30, 1e30, 1e30);

        // Count the triangles:
        countingCallback counting;
        pConcave->processAllTriangles(&counting, min, max);
        pMesh->m_numTriangles = counting.m_count;

        // Generate default indices:
        pMesh->m_numVertices = 3 * pMesh->m_numTriangles;
        const unsigned int numIndices = pMesh->m_numVertices;
        jint * const pIndices = new jint[numIndices]; //dance042
        for (unsigned int i = 0; i < numIndices; ++i) {
            pIndices[i] = i;
        }
        pMesh->m_triangleIndexBase = (unsigned char *) pIndices;

        // Copy the triangle vertex locations:
        const unsigned int numFloats = 3 * pMesh->m_numVertices;
        float * const pVertices = new float[numFloats]; //dance044
        copyingCallback copying(pMesh->m_numTriangles, pVertices);
        pConcave->processAllTriangles(&copying, min, max);
        pMesh->m_vertexBase = (unsigned char *) pVertices;

    } else if (pShape->isConvex()) {
        const btConvexShape * const pConvex = (btConvexShape *) pShape;

        // Create a hull approximation:
        btShapeHull * const pHull = new btShapeHull(pConvex); //dance043
        const btScalar margin = pConvex->getMargin();
        bool success = pHull->buildHull(margin, meshResolution);
        if (!success) {
            delete pHull; //dance043
            delete pMesh; //dance020
            return 0;
        }

        pMesh->m_numTriangles = pHull->numTriangles();
        pMesh->m_numVertices = pHull->numVertices();

        // Copy the triangle indices:
        const unsigned int numIndices = 3 * pMesh->m_numTriangles;
        const unsigned int *pHullIndices = pHull->getIndexPointer();
        jint * const pIndices = new jint[numIndices]; //dance042
        for (unsigned int i = 0; i < numIndices; ++i) {
            pIndices[i] = pHullIndices[i];
        }
        pMesh->m_triangleIndexBase = (unsigned char *) pIndices;

        // Copy the triangle vertex locations:
        const unsigned int numFloats = 3 * pMesh->m_numVertices;
        const btVector3 *pHullVertices = pHull->getVertexPointer();
        float * const pVertices = new float[numFloats]; //dance044
        unsigned int floatIndex = 0;
        for (jint i = 0; i < pMesh->m_numVertices; ++i) {
            btVector3 vertexLocation = pHullVertices[i];
            pVertices[floatIndex++] = vertexLocation.x();
            pVertices[floatIndex++] = vertexLocation.y();
            pVertices[floatIndex++] = vertexLocation.z();
        }
        btAssert(floatIndex == numFloats);
        pMesh->m_vertexBase = (unsigned char *) pVertices;

        delete pHull; //dance043

    } else { // probably a btCompoundShape
        delete pMesh; //dance020
        return 0;
    }

    return reinterpret_cast<jlong> (pMesh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    createShort
 * Signature: (Ljava/nio/ShortBuffer;Ljava/nio/FloatBuffer;IIII)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_createShort
(JNIEnv *pEnv, jclass, jobject shortBuffer, jobject floatBuffer,
        jint numTriangles, jint numVertices, jint vertexStride,
        jint indexStride) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, shortBuffer, "The index buffer does not exist.", 0);
    const unsigned char * const pIndices
            = (unsigned char *) pEnv->GetDirectBufferAddress(shortBuffer);
    NULL_CHK(pEnv, pIndices, "The index buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, floatBuffer, "The position buffer does not exist.", 0);
    const unsigned char * const pVertices
            = (unsigned char *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pVertices, "The position buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    btIndexedMesh * const pMesh = new btIndexedMesh(); //dance020
    pMesh->m_indexType = PHY_SHORT;
    pMesh->m_vertexType = PHY_FLOAT;
    pMesh->m_triangleIndexBase = pIndices;
    pMesh->m_vertexBase = pVertices;
    pMesh->m_numTriangles = numTriangles;
    pMesh->m_numVertices = numVertices;
    pMesh->m_vertexStride = vertexStride;
    pMesh->m_triangleIndexStride = indexStride;

    return reinterpret_cast<jlong> (pMesh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    fillBuffersInt
 * Signature: (JLjava/nio/FloatBuffer;Ljava/nio/IntBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_fillBuffersInt
(JNIEnv *pEnv, jclass, jlong meshId, jobject floatBuffer, jobject intBuffer) {
    const btIndexedMesh * const pMesh
            = reinterpret_cast<btIndexedMesh *> (meshId);
    NULL_CHK(pEnv, pMesh, "The btIndexedMesh does not exist.",);

    NULL_CHK(pEnv, floatBuffer, "The position buffer does not exist.",);
    float * const pVertices
            = (float *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pVertices, "The position buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);
    const jlong capacityFloats = pEnv->GetDirectBufferCapacity(floatBuffer);
    EXCEPTION_CHK(pEnv,);
    btAssert(capacityFloats == 3 * pMesh->m_numVertices);

    NULL_CHK(pEnv, intBuffer, "The index buffer does not exist.",);
    jint * const pIndices = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pIndices, "The index buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);
    const jlong capacityInts = pEnv->GetDirectBufferCapacity(intBuffer);
    EXCEPTION_CHK(pEnv,);
    btAssert(capacityInts == 3 * pMesh->m_numTriangles);

    jint * const pMeshIndices = (jint *) pMesh->m_triangleIndexBase;
    for (jint i = 0; i < capacityInts; ++i) {
        pIndices[i] = pMeshIndices[i];
    }

    float * const pMeshVertices = (float *) pMesh->m_vertexBase;
    for (unsigned int i = 0; i < capacityFloats; ++i) {
        pVertices[i] = pMeshVertices[i];
    }
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_finalizeNative
(JNIEnv *pEnv, jclass, jlong meshId) {
    btIndexedMesh * const pMesh = reinterpret_cast<btIndexedMesh *> (meshId);
    NULL_CHK(pEnv, pMesh, "The btIndexedMesh does not exist.",);

    if (pMesh->m_ownArrays) {
        delete[] pMesh->m_triangleIndexBase; //dance042
        pMesh->m_triangleIndexBase = nullptr;

        delete[] pMesh->m_vertexBase; //dance044
        pMesh->m_vertexBase = nullptr;
    }
    delete pMesh; //dance020
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    triangleIndexStride
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_triangleIndexStride
(JNIEnv *pEnv, jclass, jlong meshId) {
    btIndexedMesh * const pMesh = reinterpret_cast<btIndexedMesh *> (meshId);
    NULL_CHK(pEnv, pMesh, "The btIndexedMesh does not exist.", 0);

    jint result = pMesh->m_triangleIndexStride;
    return result;
}
