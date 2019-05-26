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

static inline btVector3 getBoundingCenter(btSoftBody* body) {
    return (body->m_bounds[0] + body->m_bounds[1]) / 2;
}

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_util_NativeSoftBodyUtil
     * Method:    updateClusterMesh
     * Signature: (JLjava/nio/FloatBuffer;Z)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeSoftBodyUtil_updateClusterMesh
    (JNIEnv *env, jclass clazz, jlong bodyId, jobject positionsBuffer,
            jboolean meshInLocalSpace) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        jfloat* positions
                = (jfloat*) env->GetDirectBufferAddress(positionsBuffer);

        const btVector3 offset =
                meshInLocalSpace ? getBoundingCenter(body) : btVector3(0, 0, 0);
        int numClusters = body->m_clusters.size();

        for (int i = 0; i < numClusters; ++i) {
            const btSoftBody::Cluster* cluster = body->m_clusters[i];
            const btVector3& clusterCom = cluster->m_com;
            positions[i * 3] = clusterCom.getX() - offset.getX();
            positions[i * 3 + 1] = clusterCom.getY() - offset.getY();
            positions[i * 3 + 2] = clusterCom.getZ() - offset.getZ();
        }
    }

    /*
     * Class:     com_jme3_bullet_util_NativeSoftBodyUtil
     * Method:    updateMesh
     * Signature: (JLjava/nio/IntBuffer;Ljava/nio/FloatBuffer;Ljava/nio/FloatBuffer;ZZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeSoftBodyUtil_updateMesh__JLjava_nio_IntBuffer_2Ljava_nio_FloatBuffer_2Ljava_nio_FloatBuffer_2ZZ
    (JNIEnv *env, jclass clazz, jlong bodyId, jobject indexMap,
            jobject positionsBuffer, jobject normalsBuffer,
            jboolean meshInLocalSpace, jboolean doNormalUpdate) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jint* jme2bulletMap = (jint*) env->GetDirectBufferAddress(indexMap);
        const int mapCapacity = env->GetDirectBufferCapacity(indexMap);

        jfloat* positions
                = (jfloat*) env->GetDirectBufferAddress(positionsBuffer);

        const btVector3 offset = (meshInLocalSpace ? getBoundingCenter(body) : btVector3(0, 0, 0));

        if (doNormalUpdate) {
            jfloat* normals
                    = (jfloat*) env->GetDirectBufferAddress(normalsBuffer);

            for (int i = 0; i < mapCapacity; ++i) {
                const btSoftBody::Node& n = body->m_nodes[jme2bulletMap[i]];
                positions[i * 3] = n.m_x.getX() - offset.getX();
                positions[i * 3 + 1] = n.m_x.getY() - offset.getY();
                positions[i * 3 + 2] = n.m_x.getZ() - offset.getZ();
                //--- normals
                normals[i * 3] = n.m_n.getX();
                normals[i * 3 + 1] = n.m_n.getY();
                normals[i * 3 + 2] = n.m_n.getZ();
            }
        } else {
            for (int i = 0; i < mapCapacity; ++i) {
                const btSoftBody::Node& n = body->m_nodes[jme2bulletMap[i]];
                positions[i * 3] = n.m_x.getX() - offset.getX();
                positions[i * 3 + 1] = n.m_x.getY() - offset.getY();
                positions[i * 3 + 2] = n.m_x.getZ() - offset.getZ();
            }
        }
    }

    /*
     * Class:     com_jme3_bullet_util_NativeSoftBodyUtil
     * Method:    updateMesh
     * Signature: (JLjava/nio/FloatBuffer;Ljava/nio/FloatBuffer;ZZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeSoftBodyUtil_updateMesh__JLjava_nio_FloatBuffer_2Ljava_nio_FloatBuffer_2ZZ
    (JNIEnv *env, jclass clazz, jlong bodyId, jobject positionsBuffer,
            jobject normalsBuffer, jboolean meshInLocalSpace,
            jboolean doNormalUpdate) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        jfloat* positions
                = (jfloat*) env->GetDirectBufferAddress(positionsBuffer);

        const btVector3 offset = (meshInLocalSpace ? getBoundingCenter(body) : btVector3(0, 0, 0));
        const int numNodes = body->m_nodes.size();

        if (doNormalUpdate) {
            jfloat* normals
                    = (jfloat*) env->GetDirectBufferAddress(normalsBuffer);

            for (int i = 0; i < numNodes; ++i) {
                const btSoftBody::Node& n = body->m_nodes[i];
                positions[i * 3] = n.m_x.getX() - offset.getX();
                positions[i * 3 + 1] = n.m_x.getY() - offset.getY();
                positions[i * 3 + 2] = n.m_x.getZ() - offset.getZ();
                //--- normals
                normals[i * 3] = n.m_n.getX();
                normals[i * 3 + 1] = n.m_n.getY();
                normals[i * 3 + 2] = n.m_n.getZ();
            }
        } else {
            for (int i = 0; i < numNodes; ++i) {
                const btSoftBody::Node& n = body->m_nodes[i];
                positions[i * 3] = n.m_x.getX() - offset.getX();
                positions[i * 3 + 1] = n.m_x.getY() - offset.getY();
                positions[i * 3 + 2] = n.m_x.getZ() - offset.getZ();
            }
        }
    }

#ifdef __cplusplus
}
#endif
