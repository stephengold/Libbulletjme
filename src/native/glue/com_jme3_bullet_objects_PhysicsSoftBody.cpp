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
#include "com_jme3_bullet_objects_PhysicsSoftBody.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

static inline btVector3 getBoundingCenter(btSoftBody *pBody) {
    return (pBody->m_bounds[0] + pBody->m_bounds[1]) / 2;
}

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addForce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addForce__JLcom_jme3_math_Vector3f_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject forceVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(forceVector, "The force vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, forceVector, &vec);

        pBody->addForce(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addForce
     * Signature: (JLcom/jme3/math/Vector3f;I)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addForce__JLcom_jme3_math_Vector3f_2I
    (JNIEnv *env, jobject object, jlong bodyId, jobject forceVector, jint nodeId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(forceVector, "The force vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, forceVector, &vec);

        btAssert(nodeId >= 0);
        btAssert(nodeId < pBody->m_nodes.size());

        pBody->addForce(vec, nodeId);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addVelocity__JLcom_jme3_math_Vector3f_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btVector3 bulletVector;
        jmeBulletUtil::convert(env, velocityVector, &bulletVector);

        pBody->addVelocity(bulletVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addVelocity
     * Signature: (JLcom/jme3/math/Vector3f;I)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addVelocity__JLcom_jme3_math_Vector3f_2I
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector,
            jint nodeId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btVector3 bulletVector;
        jmeBulletUtil::convert(env, velocityVector, &bulletVector);

        btAssert(nodeId >= 0);
        btAssert(nodeId < pBody->m_nodes.size());

        pBody->addVelocity(bulletVector, nodeId);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendCluster
     * Signature: (JILjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendCluster
    (JNIEnv *env, jobject object, jlong softBodyId, jint numMembers, jobject intBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (softBodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(intBuffer, "The IntBuffer does not exist.",);
        const jint *pBuffer = (jint *) env->GetDirectBufferAddress(intBuffer);

        int newClusterIndex = pBody->clusterCount();
        pBody->m_clusters.resize(newClusterIndex + 1);
        int clusterBytes = sizeof (btSoftBody::Cluster);
        btSoftBody::Cluster *pCluster
                = new (btAlignedAlloc(clusterBytes, 16)) btSoftBody::Cluster();
        pBody->m_clusters[newClusterIndex] = pCluster;
        pCluster->m_collide = true;

        for (int i = 0; i < numMembers; ++i) {
            int nodeIndex = pBuffer[i];
            btAssert(nodeIndex >= 0);
            btAssert(nodeIndex < pBody->m_nodes.size());
            btSoftBody::Node *pNode = &pBody->m_nodes[nodeIndex];
            pCluster->m_nodes.push_back(pNode);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendFaces
     * Signature: (JILjava/nio/ByteBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendFaces__JILjava_nio_ByteBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numFaces,
            jobject byteBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(byteBuffer, "The ByteBuffer does not exist.",);
        const jbyte *pBuffer
                = (jbyte *) env->GetDirectBufferAddress(byteBuffer);

        for (int i = 0; i < 3 * numFaces;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            int ni3 = pBuffer[i++];
            btAssert(ni3 >= 0);
            btAssert(ni3 < pBody->m_nodes.size());

            pBody->appendFace(ni1, ni2, ni3);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendFaces
     * Signature: (JILjava/nio/ShortBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendFaces__JILjava_nio_ShortBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numFaces,
            jobject shortBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(shortBuffer, "The ShortBuffer does not exist.",);
        const jshort *pBuffer
                = (jshort *) env->GetDirectBufferAddress(shortBuffer);

        for (int i = 0; i < 3 * numFaces;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            int ni3 = pBuffer[i++];
            btAssert(ni3 >= 0);
            btAssert(ni3 < pBody->m_nodes.size());

            pBody->appendFace(ni1, ni2, ni3);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendFaces
     * Signature: (JILjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendFaces__JILjava_nio_IntBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numFaces,
            jobject intBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(intBuffer, "The IntBuffer does not exist.",);
        const jint *pBuffer = (jint *) env->GetDirectBufferAddress(intBuffer);

        for (int i = 0; i < 3 * numFaces;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            int ni3 = pBuffer[i++];
            btAssert(ni3 >= 0);
            btAssert(ni3 < pBody->m_nodes.size());

            pBody->appendFace(ni1, ni2, ni3);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendLinks
     * Signature: (JILjava/nio/ByteBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendLinks__JILjava_nio_ByteBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numLinks,
            jobject byteBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(byteBuffer, "The ByteBuffer does not exist.",);
        const jbyte *pBuffer
                = (jbyte *) env->GetDirectBufferAddress(byteBuffer);

        for (int i = 0; i < 2 * numLinks;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            pBody->appendLink(ni1, ni2);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendLinks
     * Signature: (JILjava/nio/ShortBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendLinks__JILjava_nio_ShortBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numLinks,
            jobject shortBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(shortBuffer, "The ShortBuffer does not exist.",);
        const jshort *pBuffer
                = (jshort *) env->GetDirectBufferAddress(shortBuffer);

        for (int i = 0; i < 2 * numLinks;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            pBody->appendLink(ni1, ni2);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendLinks
     * Signature: (JILjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendLinks__JILjava_nio_IntBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numLinks,
            jobject intBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(intBuffer, "The IntBuffer does not exist.",);
        const jint *pBuffer = (jint *) env->GetDirectBufferAddress(intBuffer);

        for (int i = 0; i < 2 * numLinks;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            pBody->appendLink(ni1, ni2);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendNodes
     * Signature: (JILjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendNodes
    (JNIEnv *env, jobject object, jlong bodyId, jint numNodes,
            jobject floatBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(floatBuffer, "The FloatBuffer does not exist.",);
        const jfloat *pBuffer
                = (jfloat *) env->GetDirectBufferAddress(floatBuffer);

        for (int i = 0; i < 3 * numNodes;) {
            float x = pBuffer[i++];
            float y = pBuffer[i++];
            float z = pBuffer[i++];
            pBody->appendNode(btVector3(x, y, z), 1);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendTetras
     * Signature: (JILjava/nio/ByteBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendTetras__JILjava_nio_ByteBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numTetras,
            jobject byteBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(byteBuffer, "The ByteBuffer does not exist.",);
        const jbyte *pBuffer
                = (jbyte *) env->GetDirectBufferAddress(byteBuffer);

        for (int i = 0; i < 4 * numTetras;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            int ni3 = pBuffer[i++];
            btAssert(ni3 >= 0);
            btAssert(ni3 < pBody->m_nodes.size());

            int ni4 = pBuffer[i++];
            btAssert(ni4 >= 0);
            btAssert(ni4 < pBody->m_nodes.size());

            pBody->appendTetra(ni1, ni2, ni3, ni4);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendTetras
     * Signature: (JILjava/nio/ShortBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendTetras__JLjava_nio_ShortBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numTetras,
            jobject shortBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(shortBuffer, "The ShortBuffer does not exist.",);
        const jshort *pBuffer
                = (jshort *) env->GetDirectBufferAddress(shortBuffer);

        for (int i = 0; i < 4 * numTetras;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            int ni3 = pBuffer[i++];
            btAssert(ni3 >= 0);
            btAssert(ni3 < pBody->m_nodes.size());

            int ni4 = pBuffer[i++];
            btAssert(ni4 >= 0);
            btAssert(ni4 < pBody->m_nodes.size());

            pBody->appendTetra(ni1, ni2, ni3, ni4);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendTetras
     * Signature: (JILjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendTetras__JILjava_nio_IntBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jint numTetras,
            jobject intBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(intBuffer, "The IntBuffer does not exist.",);
        const jint *pBuffer = (jint *) env->GetDirectBufferAddress(intBuffer);

        for (int i = 0; i < 4 * numTetras;) {
            int ni1 = pBuffer[i++];
            btAssert(ni1 >= 0);
            btAssert(ni1 < pBody->m_nodes.size());

            int ni2 = pBuffer[i++];
            btAssert(ni2 >= 0);
            btAssert(ni2 < pBody->m_nodes.size());

            int ni3 = pBuffer[i++];
            btAssert(ni3 >= 0);
            btAssert(ni3 < pBody->m_nodes.size());

            int ni4 = pBuffer[i++];
            btAssert(ni4 >= 0);
            btAssert(ni4 < pBody->m_nodes.size());

            pBody->appendTetra(ni1, ni2, ni3, ni4);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    applyPhysicsRotation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsRotation
    (JNIEnv *env, jobject object, jlong bodyId, jobject rotation) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btQuaternion rot = btQuaternion();
        jmeBulletUtil::convert(env, rotation, &rot);
        pBody->rotate(rot);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    applyPhysicsScale
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsScale
    (JNIEnv *env, jobject object, jlong bodyId, jobject scaleVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btVector3 vec;
        jmeBulletUtil::convert(env, scaleVector, &vec);
        pBody->scale(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    applyPhysicsTransform
     * Signature: (JLcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsTransform
    (JNIEnv *env, jobject object, jlong bodyId, jobject transform) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(transform, "The transform does not exist.",)
        btTransform trs;
        btVector3 scale;
        jmeBulletUtil::convert(env, transform, &trs, &scale);

        pBody->scale(scale);
        pBody->transform(trs);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    applyPhysicsTranslate
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsTranslate
    (JNIEnv *env, jobject object, jlong bodyId, jobject offsetVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btVector3 vec;
        jmeBulletUtil::convert(env, offsetVector, &vec);
        pBody->translate(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    countNodesInCluster
     * Signature: (JI)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_countNodesInCluster
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        jint count = pCluster->m_nodes.size();

        return count;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    createEmptySoftBody
     * Signature: ()J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_createEmptySoftBody
    (JNIEnv *env, jobject object) {
        jmeClasses::initJavaClasses(env);
        /*
         * Create a temporary btSoftBodyWorldInfo for this body,
         * which will be replaced when the body gets added to a physics space.
         */
        btSoftBodyWorldInfo *pInfo = new btSoftBodyWorldInfo();

        btSoftBody *pBody = new btSoftBody(pInfo);
        pBody->getCollisionShape()->setMargin(CONVEX_DISTANCE_MARGIN);
        pBody->setUserPointer(NULL); // not in any jmePhysicsSoftSpace

        btSoftBody::Material *pMaterial = pBody->appendMaterial();
        pMaterial->m_kLST = 1;
        pMaterial->m_kAST = 1;
        pMaterial->m_kVST = 1;
        /*
         * The only available flag for materials is DebugDraw (on by Default).
         * Disable Bullet's debug draw because Minie has its own.
         */
        pMaterial->m_flags = 0x0000;

        return reinterpret_cast<jlong> (pBody);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    cutLink
     * Signature: (JIIF)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_cutLink
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeIndex0,
            jint nodeIndex1, jfloat position) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", JNI_FALSE);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(nodeIndex0 >= 0);
        btAssert(nodeIndex0 < pBody->m_nodes.size());

        btAssert(nodeIndex1 >= 0);
        btAssert(nodeIndex1 < pBody->m_nodes.size());

        bool success = pBody->cutLink((int) nodeIndex0, (int) nodeIndex1,
                (btScalar) position);
        return (jboolean) success;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    finishClusters
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_finishClusters
    (JNIEnv *env, jobject object, jlong softBodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (softBodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        int numClusters = pBody->clusterCount();
        if (numClusters > 0) {
            pBody->initializeClusters();
            pBody->updateClusters();
            /*
             * populate the cluster connectivity matrix (for self-collisions)
             */
            pBody->m_clusterConnectivity.resize(numClusters * numClusters);
            for (int c0 = 0; c0 < numClusters; ++c0) {
                btSoftBody::Cluster *pCluster0 = pBody->m_clusters[c0];
                pCluster0->m_clusterIndex = c0;
                for (int c1 = 0; c1 < numClusters; ++c1) {
                    bool connect = false;
                    btSoftBody::Cluster *pCluster1 = pBody->m_clusters[c1];
                    for (int i = 0; !connect && i < pCluster0->m_nodes.size(); ++i) {
                        for (int j = 0; j < pCluster1->m_nodes.size(); ++j) {
                            if (pCluster0->m_nodes[i] == pCluster1->m_nodes[j]) {
                                connect = true;
                                break;
                            }
                        }
                    }
                    int arrayIndex = c0 + c1 * numClusters;
                    pBody->m_clusterConnectivity[arrayIndex] = connect;
                }
            }
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    generateBendingConstraints
     * Signature: (JIJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_generateBendingConstraints
    (JNIEnv *env, jobject object, jlong bodyId, jint dist, jlong matId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btSoftBody::Material *pMaterial
                = reinterpret_cast<btSoftBody::Material *> (matId);
        NULL_CHECK(pMaterial, "The material does not exist.",)

        pBody->generateBendingConstraints(dist, pMaterial);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    generateClusters
     * Signature: (JII)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_generateClusters
    (JNIEnv *env, jobject object, jlong bodyId, jint k, jint maxIter) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->generateClusters(k, maxIter);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getBounds
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getBounds
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeMinima,
            jobject storeMaxima) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        const btVector3& minima = pBody->m_bounds[0];
        jmeBulletUtil::convert(env, &minima, storeMinima);

        const btVector3& maxima = pBody->m_bounds[1];
        jmeBulletUtil::convert(env, &maxima, storeMaxima);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterAngularDamping
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterAngularDamping
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        btScalar result = pCluster->m_adamping;

        return (jfloat) result;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterCenter
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterCenter
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jobject storeVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        NULL_CHECK(storeVector, "The store vector does not exist.",);
        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        jmeBulletUtil::convert(env, &pCluster->m_com, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterCount
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterCount
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return pBody->clusterCount();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterLinearDamping
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterLinearDamping
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        btScalar result = pCluster->m_ldamping;

        return (jfloat) result;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterMatching
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterMatching
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        btScalar result = pCluster->m_matching;

        return (jfloat) result;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterMaxSelfImpulse
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterMaxSelfImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        btScalar result = pCluster->m_maxSelfCollisionImpulse;

        return (jfloat) result;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterNodeDamping
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterNodeDamping
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        btScalar result = pCluster->m_ndamping;

        return (jfloat) result;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterSelfImpulse
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterSelfImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        btScalar result = pCluster->m_selfCollisionImpulseFactor;

        return (jfloat) result;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClustersMasses
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClustersMasses
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(storeBuffer, "The store buffer does not exist.",);
        jfloat *pBuffer = (jfloat *) env->GetDirectBufferAddress(storeBuffer);
        int numClusters = pBody->clusterCount();
        for (int clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex) {
            const btSoftBody::Cluster *pCluster
                    = pBody->m_clusters[clusterIndex];
            btScalar mass = btScalar(1.) / pCluster->m_imass;
            pBuffer[clusterIndex] = mass;
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClustersPositions
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClustersPositions
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(storeBuffer, "The store buffer does not exist.",);
        jfloat *pBuffer = (jfloat *) env->GetDirectBufferAddress(storeBuffer);
        int numClusters = pBody->clusterCount();
        for (int clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex) {
            const btSoftBody::Cluster *pCluster
                    = pBody->m_clusters[clusterIndex];
            pBuffer[0] = pCluster->m_com.getX();
            pBuffer[1] = pCluster->m_com.getY();
            pBuffer[2] = pCluster->m_com.getZ();
            pBuffer += 3;
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getFacesIndexes
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getFacesIndexes
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(intBuffer, "The IntBuffer does not exist.",);
        jint *pBuffer = (jint *) env->GetDirectBufferAddress(intBuffer);
        const int size = pBody->m_faces.size();

        btSoftBody::Node *pFirstNode = &pBody->m_nodes[0];

        for (int i = 0, buffi = 0; i < size; ++i) {
            const btSoftBody::Face& pFace = pBody->m_faces[i];
            pBuffer[buffi++] = int(pFace.m_n[0] - pFirstNode);
            pBuffer[buffi++] = int(pFace.m_n[1] - pFirstNode);
            pBuffer[buffi++] = int(pFace.m_n[2] - pFirstNode);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getLinksIndexes
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getLinksIndexes
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(intBuffer, "The IntBuffer does not exist.",);
        jint *pBuffer = (jint *) env->GetDirectBufferAddress(intBuffer);
        const int size = pBody->m_links.size();

        btSoftBody::Node *pFirstNode = &pBody->m_nodes[0];

        for (int i = 0, buffi = 0; i < size; ++i) {
            const btSoftBody::Link& l = pBody->m_links[i];
            pBuffer[buffi++] = int(l.m_n[0] - pFirstNode);
            pBuffer[buffi++] = int(l.m_n[1] - pFirstNode);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getMargin
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getMargin
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        const btCollisionShape *pShape = pBody->getCollisionShape();
        btScalar margin = pShape->getMargin();
        return (jfloat) margin;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getMass
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getMass
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(nodeId >= 0);
        btAssert(nodeId < pBody->m_nodes.size());

        return pBody->getMass(nodeId);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getMasses
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getMasses
    (JNIEnv *env, jobject object, jlong bodyId, jobject massBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(massBuffer, "The mass buffer does not exist.",);
        jfloat *pMasses = (jfloat *) env->GetDirectBufferAddress(massBuffer);
        int capacity = env->GetDirectBufferCapacity(massBuffer);
        int numNodes = pBody->m_nodes.size();
        for (int i = 0; i < numNodes && i < capacity; ++i) {
            pMasses[i] = pBody->getMass(i);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getMaterial
     * Signature: (J)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getMaterial
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return reinterpret_cast<long> (pBody->m_materials[0]);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNbFaces
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbFaces
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return pBody->m_faces.size();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNbLinks
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbLinks
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return pBody->m_links.size();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNbNodes
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbNodes
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return pBody->m_nodes.size();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNbTetras
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbTetras
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return pBody->m_tetras.size();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodeLocation
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodeLocation
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeIndex,
            jobject storeVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(nodeIndex >= 0);
        btAssert(nodeIndex < pBody->m_nodes.size());

        NULL_CHECK(storeVector, "The store vector does not exist.",);
        const btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
        jmeBulletUtil::convert(env, &node.m_x, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodeNormal
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodeNormal
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeIndex,
            jobject storeVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(nodeIndex >= 0);
        btAssert(nodeIndex < pBody->m_nodes.size());

        NULL_CHECK(storeVector, "The store vector does not exist.",);
        const btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
        jmeBulletUtil::convert(env, &node.m_n, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodesNormals
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodesNormals
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(storeBuffer, "The store buffer does not exist.",);
        jfloat *pWrite = (jfloat *) env->GetDirectBufferAddress(storeBuffer);
        int numNodes = pBody->m_nodes.size();
        for (int nodeIndex = 0; nodeIndex < numNodes; ++nodeIndex) {
            const btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
            pWrite[0] = node.m_n.getX();
            pWrite[1] = node.m_n.getY();
            pWrite[2] = node.m_n.getZ();
            pWrite += 3;
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodesPositions
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodesPositions
    (JNIEnv *env, jobject object, jlong bodyId, jobject floatBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(floatBuffer, "The FloatBuffer does not exist.",);
        jfloat *pBuffer = (jfloat *) env->GetDirectBufferAddress(floatBuffer);
        const int size = pBody->m_nodes.size();

        for (int i = 0, buffi = 0; i < size; i++) {
            const btSoftBody::Node& n = pBody->m_nodes[i];
            pBuffer[buffi++] = n.m_x.getX();
            pBuffer[buffi++] = n.m_x.getY();
            pBuffer[buffi++] = n.m_x.getZ();
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodesVelocities
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodesVelocities
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(storeBuffer, "The store buffer does not exist.",);
        jfloat *pWrite = (jfloat *) env->GetDirectBufferAddress(storeBuffer);
        int numNodes = pBody->m_nodes.size();
        for (int nodeIndex = 0; nodeIndex < numNodes; ++nodeIndex) {
            const btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
            pWrite[0] = node.m_v.getX();
            pWrite[1] = node.m_v.getY();
            pWrite[2] = node.m_v.getZ();
            pWrite += 3;
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodeVelocity
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodeVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeIndex,
            jobject storeVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(nodeIndex >= 0);
        btAssert(nodeIndex < pBody->m_nodes.size());

        NULL_CHECK(storeVector, "The store vector does not exist.",);
        const btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
        jmeBulletUtil::convert(env, &node.m_v, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getPhysicsLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getPhysicsLocation
    (JNIEnv *env, jobject object, jlong bodyId, jobject location) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btVector3 vec = getBoundingCenter(pBody);
        jmeBulletUtil::convert(env, &vec, location);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getRestLengthScale
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getRestLengthScale
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return pBody->getRestLengthScale();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getSoftBodyWorldInfo
     * Signature: (J)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getSoftBodyWorldInfo
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return reinterpret_cast<jlong> (pBody->getWorldInfo());
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getTetrasIndexes
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getTetrasIndexes
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(intBuffer, "The IntBuffer does not exist.",);
        jint *pBuffer = (jint *) env->GetDirectBufferAddress(intBuffer);
        const int size = pBody->m_tetras.size();

        btSoftBody::Node *pFirstNode = &pBody->m_nodes[0];

        for (int i = 0, buffi = 0; i < size; ++i) {
            const btSoftBody::Tetra& t = pBody->m_tetras[i];
            pBuffer[buffi++] = int(t.m_n[0] - pFirstNode);
            pBuffer[buffi++] = int(t.m_n[1] - pFirstNode);
            pBuffer[buffi++] = int(t.m_n[2] - pFirstNode);
            pBuffer[buffi++] = int(t.m_n[3] - pFirstNode);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getTotalMass
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getTotalMass
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return pBody->getTotalMass();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getVolume
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getVolume
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.", 0)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        return pBody->getVolume();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getWindVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getWindVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        const btVector3& vec = pBody->getWindVelocity();
        jmeBulletUtil::convert(env, &vec, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    initDefault
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_initDefault
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->initDefaults();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    isCollisionAllowed
     * Signature: (JJ)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_isCollisionAllowed
    (JNIEnv *env, jobject object, jlong softId, jlong rigidId) {
        btSoftBody *pSoftBody = reinterpret_cast<btSoftBody *> (softId);
        NULL_CHECK(pSoftBody, "The btSoftBody does not exist.", JNI_FALSE)
        btAssert(
                pSoftBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btCollisionObject *pRigidBody
                = reinterpret_cast<btCollisionObject *> (rigidId);
        NULL_CHECK(pRigidBody, "The btRigidBody does not exist.", JNI_FALSE);
        btAssert(pRigidBody->getInternalType()
                & btCollisionObject::CO_RIGID_BODY);

        btAlignedObjectArray<const class btCollisionObject *> cdos
        = pSoftBody->m_collisionDisabledObjects;
        int cdoIndex = cdos.findLinearSearch(pRigidBody);
        bool allowed = (cdoIndex == cdos.size());

        return (jboolean) allowed;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    listNodesInCluster
     * Signature: (JILjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_listNodesInCluster
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jobject intBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        NULL_CHECK(intBuffer, "The IntBuffer does not exist.",);
        jint *pBuffer = (jint *) env->GetDirectBufferAddress(intBuffer);

        const btSoftBody::Node *pFirstNode = &pBody->m_nodes[0];
        const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        int numNodes = pCluster->m_nodes.size();

        for (int i = 0; i < numNodes; ++i) {
            const btSoftBody::Node *pNode = pCluster->m_nodes[i];
            jint nodeIndex = pNode - pFirstNode;
            pBuffer[i] = nodeIndex;
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    randomizeConstraints
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_randomizeConstraints
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->randomizeConstraints();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    releaseCluster
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_releaseCluster
    (JNIEnv *env, jobject object, jlong bodyId, jint index) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(index >= 0);
        btAssert(index < pBody->clusterCount());

        pBody->releaseCluster(index);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    releaseClusters
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_releaseClusters
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->releaseClusters();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    resetLinkRestLengths
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_resetLinkRestLengths
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->resetLinkRestLengths();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setClusterAngularDamping
     * Signature: (JIF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setClusterAngularDamping
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jfloat damping) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        pCluster->m_adamping = (btScalar) damping;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setClusterLinearDamping
     * Signature: (JIF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setClusterLinearDamping
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jfloat damping) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        pCluster->m_ldamping = (btScalar) damping;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setClusterMatching
     * Signature: (JIF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setClusterMatching
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jfloat coefficient) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        pCluster->m_matching = (btScalar) coefficient;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setClusterMaxSelfImpulse
     * Signature: (JIF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setClusterMaxSelfImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jfloat maxImpulse) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        pCluster->m_maxSelfCollisionImpulse = (btScalar) maxImpulse;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setClusterNodeDamping
     * Signature: (JIF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setClusterNodeDamping
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jfloat damping) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        pCluster->m_ndamping = (btScalar) damping;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setClusterSelfImpulse
     * Signature: (JIF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setClusterSelfImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jfloat factor) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(clusterIndex >= 0);
        btAssert(clusterIndex < pBody->clusterCount());

        btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
        pCluster->m_selfCollisionImpulseFactor = (btScalar) factor;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setMargin
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setMargin
    (JNIEnv *env, jobject object, jlong bodyId, jfloat margin) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btCollisionShape *pShape = pBody->getCollisionShape();
        pShape->setMargin(margin);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setMass
     * Signature: (JIF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setMass
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeId, jfloat mass) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(nodeId >= 0);
        btAssert(nodeId < pBody->m_nodes.size());

        pBody->setMass(nodeId, mass);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setMasses
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setMasses
    (JNIEnv *env, jobject object, jlong bodyId, jobject massBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(massBuffer, "The mass buffer does not exist.",);
        const jfloat *pBuffer
                = (jfloat *) env->GetDirectBufferAddress(massBuffer);
        int capacity = env->GetDirectBufferCapacity(massBuffer);
        int numNodes = pBody->m_nodes.size();
        for (int i = 0; i < numNodes && i < capacity; ++i) {
            pBody->setMass(i, pBuffer[i]);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setNodeVelocity
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setNodeVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeIndex,
            jobject velocityVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btAssert(nodeIndex >= 0);
        btAssert(nodeIndex < pBody->m_nodes.size());

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
        jmeBulletUtil::convert(env, velocityVector, &node.m_v);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setNormals
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setNormals
    (JNIEnv *env, jobject object, jlong bodyId, jobject normalBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(normalBuffer, "The normal buffer does not exist.",);
        const jfloat *pBuffer
                = (jfloat *) env->GetDirectBufferAddress(normalBuffer);
        int capacity = env->GetDirectBufferCapacity(normalBuffer) - 2;
        int numNodes = pBody->m_nodes.size();
        for (int nodeIndex = 0, offset = 0;
                nodeIndex < numNodes && offset < capacity;) {
            btScalar x = pBuffer[offset++];
            btScalar y = pBuffer[offset++];
            btScalar z = pBuffer[offset++];
            btSoftBody::Node& node = pBody->m_nodes[nodeIndex++];
            node.m_n = btVector3(x, y, z);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setPhysicsLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setPhysicsLocation
    (JNIEnv *env, jobject object, jlong bodyId, jobject locationVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(locationVector, "The location vector does not exist.",);
        btVector3 vec;
        jmeBulletUtil::convert(env, locationVector, &vec);

        vec -= getBoundingCenter(pBody);
        pBody->translate(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setPose
     * Signature: (JZZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setPose
    (JNIEnv *env, jobject object, jlong bodyId, jboolean bvolume,
            jboolean bframe) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->setPose(bvolume, bframe);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setRestLengthScale
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setRestLengthScale
    (JNIEnv *env, jobject object, jlong bodyId, jfloat value) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->setRestLengthScale(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setSoftBodyWorldInfo
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setSoftBodyWorldInfo
    (JNIEnv *env, jobject object, jlong bodyId, jlong worldId) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        btSoftBodyWorldInfo *pWorldInfo
                = reinterpret_cast<btSoftBodyWorldInfo *> (worldId);
        NULL_CHECK(pWorldInfo, "The btSoftBodyWorldInfo does not exist.",)

        pBody->m_worldInfo = pWorldInfo;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setTotalDensity
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setTotalDensity
    (JNIEnv *env, jobject object, jlong bodyId, jfloat density) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->setTotalDensity(density);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setTotalMass
     * Signature: (JFZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setTotalMass
    (JNIEnv *env, jobject object, jlong bodyId, jfloat mass,
            jboolean fromFaces) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->setTotalMass(mass, fromFaces);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setVelocities
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVelocities
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityBuffer) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(velocityBuffer, "The velocity buffer does not exist.",);
        const jfloat *pBuffer
                = (jfloat *) env->GetDirectBufferAddress(velocityBuffer);
        int capacity = env->GetDirectBufferCapacity(velocityBuffer) - 2;
        int numNodes = pBody->m_nodes.size();
        for (int nodeIndex = 0, offset = 0;
                nodeIndex < numNodes && offset < capacity;) {
            btScalar x = pBuffer[offset++];
            btScalar y = pBuffer[offset++];
            btScalar z = pBuffer[offset++];
            btSoftBody::Node& node = pBody->m_nodes[nodeIndex++];
            node.m_v = btVector3(x, y, z);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btVector3 bulletVector;
        jmeBulletUtil::convert(env, velocityVector, &bulletVector);

        pBody->setVelocity(bulletVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setVolumeDensity
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVolumeDensity
    (JNIEnv *env, jobject object, jlong bodyId, jfloat density) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->setVolumeDensity(density);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setVolumeMass
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVolumeMass
    (JNIEnv *env, jobject object, jlong bodyId, jfloat mass) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        pBody->setVolumeMass(mass);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setWindVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setWindVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
        NULL_CHECK(pBody, "The btSoftBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btVector3 bulletVector;
        jmeBulletUtil::convert(env, velocityVector, &bulletVector);

        pBody->setWindVelocity(bulletVector);
    }

#ifdef __cplusplus
}
#endif