/*
 * Copyright (c) 2019 jMonkeyEngine
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
#include "com_jme3_bullet_collision_shapes_infos_StridingMesh.h"
#include "jmeBulletUtil.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_collision_shapes_infos_StridingMesh
     * Method:    createTiva
     * Signature: (Ljava/nio/IntBuffer;Ljava/nio/FloatBuffer;IIII)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_StridingMesh_createTiva
    (JNIEnv *env, jobject object, jobject intBuffer, jobject floatBuffer,
            jint numTriangles, jint numVertices, jint vertexStride,
            jint indexStride) {
        jmeClasses::initJavaClasses(env);

        NULL_CHECK(intBuffer, "The index buffer does not exist.", 0);
        int* pIndices = (int *) env->GetDirectBufferAddress(intBuffer);

        NULL_CHECK(floatBuffer, "The position buffer does not exist.", 0);
        jfloat* pVertices = (jfloat*) env->GetDirectBufferAddress(floatBuffer);

        btTriangleIndexVertexArray* pMesh;
#ifdef BT_USE_DOUBLE_PRECISION
        int numCoordinates = 3 * numVertices;
        btScalar *pDpVertices = new btScalar[numCoordinates]; // never freed
        for (int i = 0; i < numCoordinates; ++i) {
            pDpVertices[i] = pVertices[i];
        }
        int doubleStride = 2 * vertexStride;
        pMesh = new btTriangleIndexVertexArray(numTriangles, pIndices,
                indexStride, numVertices, pDpVertices, doubleStride);
#else
        pMesh = new btTriangleIndexVertexArray(numTriangles, pIndices,
                indexStride, numVertices, pVertices, vertexStride);
#endif

        return reinterpret_cast<jlong> (pMesh);
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_infos_StridingMesh
     * Method:    finalizeNative
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_StridingMesh_finalizeNative
    (JNIEnv *env, jobject object, jlong meshId) {
        btStridingMeshInterface* pMesh
                = reinterpret_cast<btStridingMeshInterface*> (meshId);
        NULL_CHECK(pMesh, "The btStridingMeshInterface does not exist.",);

#ifdef BT_USE_DOUBLE_PRECISION
        // TODO free pMesh->m_vertexBase
#endif
        delete pMesh;
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_infos_StridingMesh
     * Method:    getScaling
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_StridingMesh_getScaling
    (JNIEnv *env, jobject object, jlong meshId, jobject storeVector) {
        btStridingMeshInterface* pMesh
                = reinterpret_cast<btStridingMeshInterface*> (meshId);
        NULL_CHECK(pMesh, "The btStridingMeshInterface does not exist.",);
        NULL_CHECK(storeVector, "The store vector does not exist.",);

        const btVector3 *pScale = &pMesh->getScaling();
        jmeBulletUtil::convert(env, pScale, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_infos_StridingMesh
     * Method:    setScaling
     * Signature: (JFFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_StridingMesh_setScaling
    (JNIEnv *env, jobject object, jlong meshId, jfloat xScale, jfloat yScale, jfloat zScale) {
        btStridingMeshInterface* pMesh
                = reinterpret_cast<btStridingMeshInterface*> (meshId);
        NULL_CHECK(pMesh, "The btStridingMeshInterface does not exist.",);

        btVector3 scale;
        scale.setX(xScale);
        scale.setY(yScale);
        scale.setZ(zScale);
        pMesh->setScaling(scale);
    }

#ifdef __cplusplus
}
#endif
