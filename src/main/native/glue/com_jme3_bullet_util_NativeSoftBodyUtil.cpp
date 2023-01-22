/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
 * Author: Dokthar
 */
#include "com_jme3_bullet_util_NativeSoftBodyUtil.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"

static inline btVector3 getBoundingCenter(const btSoftBody *pBody) {
    return (pBody->m_bounds[0] + pBody->m_bounds[1]) / 2;
}

/*
 * Class:     com_jme3_bullet_util_NativeSoftBodyUtil
 * Method:    updateClusterMesh
 * Signature: (JLjava/nio/FloatBuffer;Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeSoftBodyUtil_updateClusterMesh
(JNIEnv *pEnv, jclass, jlong bodyId, jobject positionsBuffer,
        jboolean meshInLocalSpace) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    NULL_CHK(pEnv, positionsBuffer, "The positions buffer does not exist.",);
    jfloat * const pBuffer
            = (jfloat *) pEnv->GetDirectBufferAddress(positionsBuffer);
    NULL_CHK(pEnv, pBuffer, "The positions buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);

    const btVector3 offset =
            meshInLocalSpace ? getBoundingCenter(pBody) : btVector3(0, 0, 0);
    const int numClusters = pBody->m_clusters.size();

    for (int i = 0; i < numClusters; ++i) {
        const btSoftBody::Cluster *pCluster = pBody->m_clusters[i];
        const btVector3& clusterCom = pCluster->m_com;
        pBuffer[i * 3] = clusterCom.getX() - offset.getX();
        pBuffer[i * 3 + 1] = clusterCom.getY() - offset.getY();
        pBuffer[i * 3 + 2] = clusterCom.getZ() - offset.getZ();
    }
}

/*
 * Class:     com_jme3_bullet_util_NativeSoftBodyUtil
 * Method:    updateMesh
 * Signature: (JLjava/nio/IntBuffer;Ljava/nio/FloatBuffer;Ljava/nio/FloatBuffer;ZZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeSoftBodyUtil_updateMesh__JLjava_nio_IntBuffer_2Ljava_nio_FloatBuffer_2Ljava_nio_FloatBuffer_2ZZ
(JNIEnv *pEnv, jclass, jlong bodyId, jobject indexMap,
        jobject positionsBuffer, jobject normalsBuffer,
        jboolean meshInLocalSpace, jboolean doNormalUpdate) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    NULL_CHK(pEnv, indexMap, "The index map does not exist.",);
    const jint * const pJme2bulletMap
            = (jint *) pEnv->GetDirectBufferAddress(indexMap);
    NULL_CHK(pEnv, pJme2bulletMap, "The index map is not direct.",);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, positionsBuffer, "The positions buffer does not exist.",);
    jfloat *pPositions
            = (jfloat *) pEnv->GetDirectBufferAddress(positionsBuffer);
    NULL_CHK(pEnv, pPositions, "The positions buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);

    const jlong mapCapacity = pEnv->GetDirectBufferCapacity(indexMap);
    EXCEPTION_CHK(pEnv,);
    const btVector3 offset
            = (meshInLocalSpace ? getBoundingCenter(pBody) : btVector3(0, 0, 0));

    if (doNormalUpdate) {
        NULL_CHK(pEnv, normalsBuffer, "The normals buffer does not exist.",);
        jfloat * const pNormals
                = (jfloat *) pEnv->GetDirectBufferAddress(normalsBuffer);
        NULL_CHK(pEnv, pNormals, "The normals buffer is not direct.",);
        EXCEPTION_CHK(pEnv,);

        for (int i = 0; i < mapCapacity; ++i) {
            const btSoftBody::Node& n = pBody->m_nodes[pJme2bulletMap[i]];
            pPositions[i * 3] = n.m_x.getX() - offset.getX();
            pPositions[i * 3 + 1] = n.m_x.getY() - offset.getY();
            pPositions[i * 3 + 2] = n.m_x.getZ() - offset.getZ();
            //--- normals
            pNormals[i * 3] = n.m_n.getX();
            pNormals[i * 3 + 1] = n.m_n.getY();
            pNormals[i * 3 + 2] = n.m_n.getZ();
        }
    } else {
        for (int i = 0; i < mapCapacity; ++i) {
            const btSoftBody::Node& n = pBody->m_nodes[pJme2bulletMap[i]];
            pPositions[i * 3] = n.m_x.getX() - offset.getX();
            pPositions[i * 3 + 1] = n.m_x.getY() - offset.getY();
            pPositions[i * 3 + 2] = n.m_x.getZ() - offset.getZ();
        }
    }
}

/*
 * Class:     com_jme3_bullet_util_NativeSoftBodyUtil
 * Method:    updateMesh
 * Signature: (JLjava/nio/FloatBuffer;Ljava/nio/FloatBuffer;ZZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeSoftBodyUtil_updateMesh__JLjava_nio_FloatBuffer_2Ljava_nio_FloatBuffer_2ZZ
(JNIEnv *pEnv, jclass, jlong bodyId, jobject positionsBuffer,
        jobject normalsBuffer, jboolean meshInLocalSpace,
        jboolean doNormalUpdate) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    NULL_CHK(pEnv, positionsBuffer, "The positions buffer does not exist.",);
    jfloat * const pPositions
            = (jfloat *) pEnv->GetDirectBufferAddress(positionsBuffer);
    NULL_CHK(pEnv, pPositions, "The positions buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);

    const btVector3 offset
            = (meshInLocalSpace ? getBoundingCenter(pBody) : btVector3(0, 0, 0));
    const int numNodes = pBody->m_nodes.size();

    if (doNormalUpdate) {
        NULL_CHK(pEnv, normalsBuffer, "The normals buffer does not exist.",);
        jfloat *pNormals
                = (jfloat *) pEnv->GetDirectBufferAddress(normalsBuffer);
        NULL_CHK(pEnv, pNormals, "The normals buffer is not direct.",);
        EXCEPTION_CHK(pEnv,);

        for (int i = 0; i < numNodes; ++i) {
            const btSoftBody::Node& n = pBody->m_nodes[i];
            pPositions[i * 3] = n.m_x.getX() - offset.getX();
            pPositions[i * 3 + 1] = n.m_x.getY() - offset.getY();
            pPositions[i * 3 + 2] = n.m_x.getZ() - offset.getZ();
            //--- normals
            pNormals[i * 3] = n.m_n.getX();
            pNormals[i * 3 + 1] = n.m_n.getY();
            pNormals[i * 3 + 2] = n.m_n.getZ();
        }
    } else {
        for (int i = 0; i < numNodes; ++i) {
            const btSoftBody::Node& n = pBody->m_nodes[i];
            pPositions[i * 3] = n.m_x.getX() - offset.getX();
            pPositions[i * 3 + 1] = n.m_x.getY() - offset.getY();
            pPositions[i * 3 + 2] = n.m_x.getZ() - offset.getZ();
        }
    }
}

/*
 * Class:     com_jme3_bullet_util_NativeSoftBodyUtil
 * Method:    updatePinMesh
 * Signature: (JLjava/nio/FloatBuffer;Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeSoftBodyUtil_updatePinMesh
(JNIEnv *pEnv, jclass, jlong bodyId, jobject positionsBuffer,
        jboolean meshInLocalSpace) {
    const btSoftBody * const
            pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    NULL_CHK(pEnv, positionsBuffer, "The positions buffer does not exist.",);
    jfloat *pBuffer = (jfloat *) pEnv->GetDirectBufferAddress(positionsBuffer);
    NULL_CHK(pEnv, pBuffer, "The positions buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);

    const btVector3 offset
            = meshInLocalSpace ? getBoundingCenter(pBody) : btVector3(0, 0, 0);
    const int numNodes = pBody->m_nodes.size();

    for (int i = 0; i < numNodes; ++i) {
        if (pBody->getMass(i) == 0) {
            const btSoftBody::Node& n = pBody->m_nodes[i];
            pBuffer[0] = n.m_x.getX() - offset.getX();
            pBuffer[1] = n.m_x.getY() - offset.getY();
            pBuffer[2] = n.m_x.getZ() - offset.getZ();
            pBuffer += 3;
        }
    }
}
