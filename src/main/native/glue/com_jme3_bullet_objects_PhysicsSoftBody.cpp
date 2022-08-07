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

static inline btVector3 getBoundingCenter(const btSoftBody *pBody) {
    return (pBody->m_bounds[0] + pBody->m_bounds[1]) / 2;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    addForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addForce__JLcom_jme3_math_Vector3f_2
(JNIEnv *pEnv, jclass, jlong bodyId, jobject forceVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, forceVector, "The force vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, forceVector, &vec);

    pBody->addForce(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    addForce
 * Signature: (JLcom/jme3/math/Vector3f;I)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addForce__JLcom_jme3_math_Vector3f_2I
(JNIEnv *pEnv, jclass, jlong bodyId, jobject forceVector, jint nodeId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, forceVector, "The force vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, forceVector, &vec);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",);
    btVector3 bulletVector;
    jmeBulletUtil::convert(pEnv, velocityVector, &bulletVector);

    pBody->addVelocity(bulletVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    addVelocity
 * Signature: (JLcom/jme3/math/Vector3f;I)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addVelocity__JLcom_jme3_math_Vector3f_2I
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVector, jint nodeId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",);
    btVector3 bulletVector;
    jmeBulletUtil::convert(pEnv, velocityVector, &bulletVector);

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
(JNIEnv *pEnv, jclass, jlong softBodyId, jint numMembers, jobject intBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (softBodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, intBuffer, "The IntBuffer does not exist.",);
    const jint * const pBuffer
            = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pBuffer, "The IntBuffer is not direct.",);

    int newClusterIndex = pBody->clusterCount();
    pBody->m_clusters.resize(newClusterIndex + 1);
    int clusterBytes = sizeof (btSoftBody::Cluster);
    btSoftBody::Cluster * const
            pCluster = new (btAlignedAlloc(clusterBytes, 16)) btSoftBody::Cluster(); // TODO leak
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numFaces, jobject byteBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, byteBuffer, "The ByteBuffer does not exist.",);
    const jbyte * const pBuffer
            = (jbyte *) pEnv->GetDirectBufferAddress(byteBuffer);
    NULL_CHK(pEnv, pBuffer, "The ByteBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numFaces, jobject shortBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, shortBuffer, "The ShortBuffer does not exist.",);
    const jshort * const pBuffer
            = (jshort *) pEnv->GetDirectBufferAddress(shortBuffer);
    NULL_CHK(pEnv, pBuffer, "The ShortBuffer is not direct.",);

    for (int i = 0; i < 3 * numFaces;) {
        int ni1 = 0xFFFF & pBuffer[i++];
        btAssert(ni1 >= 0);
        btAssert(ni1 < pBody->m_nodes.size());

        int ni2 = 0xFFFF & pBuffer[i++];
        btAssert(ni2 >= 0);
        btAssert(ni2 < pBody->m_nodes.size());

        int ni3 = 0xFFFF & pBuffer[i++];
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numFaces, jobject intBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, intBuffer, "The IntBuffer does not exist.",);
    const jint * const pBuffer
            = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pBuffer, "The IntBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numLinks, jobject byteBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, byteBuffer, "The ByteBuffer does not exist.",);
    const jbyte * const pBuffer
            = (jbyte *) pEnv->GetDirectBufferAddress(byteBuffer);
    NULL_CHK(pEnv, pBuffer, "The ByteBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numLinks, jobject shortBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, shortBuffer, "The ShortBuffer does not exist.",);
    const jshort * const pBuffer
            = (jshort *) pEnv->GetDirectBufferAddress(shortBuffer);
    NULL_CHK(pEnv, pBuffer, "The ShortBuffer is not direct.",);

    for (int i = 0; i < 2 * numLinks;) {
        int ni1 = 0xFFFF & pBuffer[i++];
        btAssert(ni1 >= 0);
        btAssert(ni1 < pBody->m_nodes.size());

        int ni2 = 0xFFFF & pBuffer[i++];
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numLinks, jobject intBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, intBuffer, "The IntBuffer does not exist.",);
    const jint * const pBuffer
            = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pBuffer, "The IntBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numNodes, jobject floatBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, floatBuffer, "The FloatBuffer does not exist.",);
    const jfloat * const pBuffer
            = (jfloat *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pBuffer, "The FloatBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numTetras, jobject byteBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, byteBuffer, "The ByteBuffer does not exist.",);
    const jbyte * const pBuffer
            = (jbyte *) pEnv->GetDirectBufferAddress(byteBuffer);
    NULL_CHK(pEnv, pBuffer, "The ByteBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numTetras, jobject shortBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, shortBuffer, "The ShortBuffer does not exist.",);
    const jshort * const pBuffer
            = (jshort *) pEnv->GetDirectBufferAddress(shortBuffer);
    NULL_CHK(pEnv, pBuffer, "The ShortBuffer is not direct.",);

    for (int i = 0; i < 4 * numTetras;) {
        int ni1 = 0xFFFF & pBuffer[i++];
        btAssert(ni1 >= 0);
        btAssert(ni1 < pBody->m_nodes.size());

        int ni2 = 0xFFFF & pBuffer[i++];
        btAssert(ni2 >= 0);
        btAssert(ni2 < pBody->m_nodes.size());

        int ni3 = 0xFFFF & pBuffer[i++];
        btAssert(ni3 >= 0);
        btAssert(ni3 < pBody->m_nodes.size());

        int ni4 = 0xFFFF & pBuffer[i++];
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint numTetras, jobject intBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, intBuffer, "The IntBuffer does not exist.",);
    const jint * const pBuffer
            = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pBuffer, "The IntBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject rotation) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btQuaternion rot = btQuaternion();
    jmeBulletUtil::convert(pEnv, rotation, &rot);
    pBody->rotate(rot);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    applyPhysicsScale
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsScale
(JNIEnv *pEnv, jclass, jlong bodyId, jobject scaleVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btVector3 vec;
    jmeBulletUtil::convert(pEnv, scaleVector, &vec);
    pBody->scale(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    applyPhysicsTransform
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsTransform
(JNIEnv *pEnv, jclass, jlong bodyId, jobject transform) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, transform, "The transform does not exist.",)
    btTransform trs;
    btVector3 scale;
    jmeBulletUtil::convert(pEnv, transform, &trs, &scale);

    pBody->scale(scale);
    pBody->transform(trs);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    applyPhysicsTranslate
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsTranslate
(JNIEnv *pEnv, jclass, jlong bodyId, jobject offsetVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btVector3 vec;
    jmeBulletUtil::convert(pEnv, offsetVector, &vec);
    pBody->translate(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    countNodesInCluster
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_countNodesInCluster
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btAssert(clusterIndex >= 0);
    btAssert(clusterIndex < pBody->clusterCount());

    const btSoftBody::Cluster * const pCluster
            = pBody->m_clusters[clusterIndex];
    jint count = pCluster->m_nodes.size();

    return count;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    createEmpty
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_createEmpty
(JNIEnv *pEnv, jclass, jlong infoId) {
    jmeClasses::initJavaClasses(pEnv);

    btSoftBodyWorldInfo * const
            pInfo = reinterpret_cast<btSoftBodyWorldInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btSoftBodyWorldInfo does not exist.", 0);

    btSoftBody * const pBody = new btSoftBody(pInfo); //dance014
    pBody->getCollisionShape()->setMargin(CONVEX_DISTANCE_MARGIN);
    pBody->setUserPointer(NULL); // TODO unnecessary?

    btSoftBody::Material *pMaterial = pBody->appendMaterial();
    pMaterial->m_kLST = 1;
    pMaterial->m_kAST = 1;
    pMaterial->m_kVST = 1;
    /*
     * The only available flag for materials is DebugDraw (on by Default).
     * Disable Bullet's debug draw because Minie has its own.
     */
    pMaterial->m_flags = 0x0;

    return reinterpret_cast<jlong> (pBody);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    cutLink
 * Signature: (JIIF)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_cutLink
(JNIEnv *pEnv, jclass, jlong bodyId, jint nodeIndex0, jint nodeIndex1,
        jfloat position) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", JNI_FALSE);
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
(JNIEnv *pEnv, jclass, jlong softBodyId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (softBodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    const int numClusters = pBody->clusterCount();
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint dist, jlong matId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btSoftBody::Material * const pMaterial
            = reinterpret_cast<btSoftBody::Material *> (matId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.",)

    pBody->generateBendingConstraints(dist, pMaterial);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    generateClusters
 * Signature: (JII)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_generateClusters
(JNIEnv *pEnv, jclass, jlong bodyId, jint k, jint maxIter) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->generateClusters(k, maxIter);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getBounds
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getBounds
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeMinima,
        jobject storeMaxima) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    const btVector3& minima = pBody->m_bounds[0];
    jmeBulletUtil::convert(pEnv, &minima, storeMinima);

    const btVector3& maxima = pBody->m_bounds[1];
    jmeBulletUtil::convert(pEnv, &maxima, storeMaxima);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getClusterAngularDamping
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterAngularDamping
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex, jobject storeVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btAssert(clusterIndex >= 0);
    btAssert(clusterIndex < pBody->clusterCount());

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
    jmeBulletUtil::convert(pEnv, &pCluster->m_com, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getClusterCount
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterCount
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return pBody->clusterCount();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getClusterLinearDamping
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterLinearDamping
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btAssert(clusterIndex >= 0);
    btAssert(clusterIndex < pBody->clusterCount());

    const btSoftBody::Cluster *pCluster = pBody->m_clusters[clusterIndex];
    btScalar result = pCluster->m_selfCollisionImpulseFactor;

    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getClustersLinearVelocities
 * Signature: (JLjava/nio/FloatBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClustersLinearVelocities
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, storeBuffer, "The store buffer does not exist.",);
    jfloat * pBuffer = (jfloat *) pEnv->GetDirectBufferAddress(storeBuffer);
    NULL_CHK(pEnv, pBuffer, "The store buffer is not direct.",);

    int numClusters = pBody->clusterCount();
    for (int clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex) {
        const btSoftBody::Cluster * const
                pCluster = pBody->m_clusters[clusterIndex];
        pBuffer[0] = pCluster->m_lv.getX();
        pBuffer[1] = pCluster->m_lv.getY();
        pBuffer[2] = pCluster->m_lv.getZ();
        pBuffer += 3;
    }
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getClustersMasses
 * Signature: (JLjava/nio/FloatBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClustersMasses
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeBuffer) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, storeBuffer, "The store buffer does not exist.",);
    jfloat * const pBuffer
            = (jfloat *) pEnv->GetDirectBufferAddress(storeBuffer);
    NULL_CHK(pEnv, pBuffer, "The store buffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, storeBuffer, "The store buffer does not exist.",);
    jfloat * pBuffer = (jfloat *) pEnv->GetDirectBufferAddress(storeBuffer);
    NULL_CHK(pEnv, pBuffer, "The store buffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject intBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, intBuffer, "The IntBuffer does not exist.",);
    jint * const pBuffer = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pBuffer, "The IntBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject intBuffer) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, intBuffer, "The IntBuffer does not exist.",);
    jint * const pBuffer = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pBuffer, "The IntBuffer is not direct.",);

    const int size = pBody->m_links.size();
    const btSoftBody::Node *pFirstNode = &pBody->m_nodes[0];

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
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0);
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint nodeId) {
    const btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject massBuffer) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, massBuffer, "The mass buffer does not exist.",);
    jfloat * const pMasses
            = (jfloat *) pEnv->GetDirectBufferAddress(massBuffer);
    NULL_CHK(pEnv, pMasses, "The mass buffer is not direct.",);

    const jlong capacity = pEnv->GetDirectBufferCapacity(massBuffer);
    int numNodes = pBody->m_nodes.size();
    for (int i = 0; i < numNodes && i < capacity; ++i) {
        pMasses[i] = pBody->getMass(i);
    }
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getNbFaces
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbFaces
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return pBody->m_faces.size();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getNbLinks
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbLinks
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return pBody->m_links.size();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getNbNodes
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbNodes
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return pBody->m_nodes.size();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getNbPinnedNodes
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbPinnedNodes
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    jint result = 0;
    int numNodes = pBody->m_nodes.size();
    for (int i = 0; i < numNodes; ++i) {
        btScalar mass = pBody->getMass(i);
        if (mass == 0) {
            ++result;
        }
    }

    return result;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getNbTetras
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbTetras
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return pBody->m_tetras.size();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getNodeLocation
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodeLocation
(JNIEnv *pEnv, jclass, jlong bodyId, jint nodeIndex, jobject storeVector) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btAssert(nodeIndex >= 0);
    btAssert(nodeIndex < pBody->m_nodes.size());

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    const btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
    jmeBulletUtil::convert(pEnv, &node.m_x, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getNodeNormal
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodeNormal
(JNIEnv *pEnv, jclass, jlong bodyId, jint nodeIndex, jobject storeVector) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btAssert(nodeIndex >= 0);
    btAssert(nodeIndex < pBody->m_nodes.size());

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    const btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
    jmeBulletUtil::convert(pEnv, &node.m_n, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getNodesNormals
 * Signature: (JLjava/nio/FloatBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodesNormals
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeBuffer) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, storeBuffer, "The store buffer does not exist.",);
    jfloat * pWrite
            = (jfloat *) pEnv->GetDirectBufferAddress(storeBuffer);
    NULL_CHK(pEnv, pWrite, "The store buffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject floatBuffer) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, floatBuffer, "The FloatBuffer does not exist.",);
    jfloat * const pBuffer
            = (jfloat *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pBuffer, "The FloatBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeBuffer) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, storeBuffer, "The store buffer does not exist.",);
    jfloat *pWrite = (jfloat *) pEnv->GetDirectBufferAddress(storeBuffer);
    NULL_CHK(pEnv, pWrite, "The store buffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId, jint nodeIndex, jobject storeVector) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btAssert(nodeIndex >= 0);
    btAssert(nodeIndex < pBody->m_nodes.size());

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    const btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
    jmeBulletUtil::convert(pEnv, &node.m_v, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getPhysicsLocation
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getPhysicsLocation
(JNIEnv *pEnv, jclass, jlong bodyId, jobject location) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btVector3 vec = getBoundingCenter(pBody);
    jmeBulletUtil::convert(pEnv, &vec, location);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getRestLengthScale
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getRestLengthScale
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return pBody->getRestLengthScale();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getSoftBodyWorldInfo
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getSoftBodyWorldInfo
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return reinterpret_cast<jlong> (pBody->getWorldInfo());
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getTetrasIndexes
 * Signature: (JLjava/nio/IntBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getTetrasIndexes
(JNIEnv *pEnv, jclass, jlong bodyId, jobject intBuffer) {
    const btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, intBuffer, "The IntBuffer does not exist.",);
    jint * const pBuffer = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pBuffer, "The IntBuffer is not direct.",);

    const int size = pBody->m_tetras.size();
    const btSoftBody::Node * const pFirstNode = &pBody->m_nodes[0];

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
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return pBody->getTotalMass();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getVolume
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getVolume
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    return pBody->getVolume();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    getWindVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getWindVelocity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    const btVector3& vec = pBody->getWindVelocity();
    jmeBulletUtil::convert(pEnv, &vec, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    initDefault
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_initDefault
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->initDefaults();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    isCollisionAllowed
 * Signature: (JJ)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_isCollisionAllowed
(JNIEnv *pEnv, jclass, jlong softId, jlong rigidId) {
    const btSoftBody * const pSoftBody
            = reinterpret_cast<btSoftBody *> (softId);
    NULL_CHK(pEnv, pSoftBody, "The btSoftBody does not exist.", JNI_FALSE)
    btAssert(pSoftBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btCollisionObject *pRigidBody
            = reinterpret_cast<btCollisionObject *> (rigidId);
    NULL_CHK(pEnv, pRigidBody, "The btRigidBody does not exist.", JNI_FALSE);
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex, jobject intBuffer) {
    const btSoftBody * const pBody
            = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btAssert(clusterIndex >= 0);
    btAssert(clusterIndex < pBody->clusterCount());

    NULL_CHK(pEnv, intBuffer, "The IntBuffer does not exist.",);
    jint * const pBuffer = (jint *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pBuffer, "The IntBuffer is not direct.",);

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
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->randomizeConstraints();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    releaseCluster
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_releaseCluster
(JNIEnv *pEnv, jclass, jlong bodyId, jint index) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
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
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->releaseClusters();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    resetLinkRestLengths
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_resetLinkRestLengths
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->resetLinkRestLengths();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setClusterAngularDamping
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setClusterAngularDamping
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex, jfloat damping) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex, jfloat damping) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex, jfloat coefficient) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex, jfloat maxImpulse) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex, jfloat damping) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint clusterIndex, jfloat factor) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat margin) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint nodeId, jfloat mass) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject massBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, massBuffer, "The mass buffer does not exist.",);
    const jfloat * const pBuffer
            = (jfloat *) pEnv->GetDirectBufferAddress(massBuffer);
    NULL_CHK(pEnv, pBuffer, "The mass buffer is not direct.",);

    const jlong capacity = pEnv->GetDirectBufferCapacity(massBuffer);
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
(JNIEnv *pEnv, jclass, jlong bodyId, jint nodeIndex, jobject velocityVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btAssert(nodeIndex >= 0);
    btAssert(nodeIndex < pBody->m_nodes.size());

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",);
    btSoftBody::Node& node = pBody->m_nodes[nodeIndex];
    jmeBulletUtil::convert(pEnv, velocityVector, &node.m_v);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setNormals
 * Signature: (JLjava/nio/FloatBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setNormals
(JNIEnv *pEnv, jclass, jlong bodyId, jobject normalBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, normalBuffer, "The normal buffer does not exist.",);
    const jfloat * const pBuffer
            = (jfloat *) pEnv->GetDirectBufferAddress(normalBuffer);
    NULL_CHK(pEnv, pBuffer, "The normal buffer is not direct.",);

    const jlong capacity = pEnv->GetDirectBufferCapacity(normalBuffer) - 2;
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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject locationVector) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",);
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, locationVector, &vec);

    vec -= getBoundingCenter(pBody);
    pBody->translate(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setPose
 * Signature: (JZZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setPose
(JNIEnv *pEnv, jclass, jlong bodyId, jboolean bvolume, jboolean bframe) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->setPose(bvolume, bframe);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setRestLengthScale
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setRestLengthScale
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat value) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->setRestLengthScale(value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setSoftBodyWorldInfo
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setSoftBodyWorldInfo
(JNIEnv *pEnv, jclass, jlong bodyId, jlong worldId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    btSoftBodyWorldInfo *pWorldInfo
            = reinterpret_cast<btSoftBodyWorldInfo *> (worldId);
    NULL_CHK(pEnv, pWorldInfo, "The btSoftBodyWorldInfo does not exist.",)

    pBody->m_worldInfo = pWorldInfo;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setTotalDensity
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setTotalDensity
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat density) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->setTotalDensity(density);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setTotalMass
 * Signature: (JFZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setTotalMass
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat mass, jboolean fromFaces) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->setTotalMass(mass, fromFaces);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setVelocities
 * Signature: (JLjava/nio/FloatBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVelocities
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityBuffer) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, velocityBuffer, "The velocity buffer does not exist.",);
    const jfloat * const pBuffer
            = (jfloat *) pEnv->GetDirectBufferAddress(velocityBuffer);
    NULL_CHK(pEnv, pBuffer, "The velocity buffer is not direct.",);

    const jlong capacity = pEnv->GetDirectBufferCapacity(velocityBuffer) - 2;
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
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",);
    btVector3 bulletVector;
    jmeBulletUtil::convert(pEnv, velocityVector, &bulletVector);

    pBody->setVelocity(bulletVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setVolumeDensity
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVolumeDensity
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat density) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->setVolumeDensity(density);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setVolumeMass
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVolumeMass
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat mass) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    pBody->setVolumeMass(mass);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody
 * Method:    setWindVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setWindVelocity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVector) {
    btSoftBody * const pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",);
    btVector3 bulletVector;
    jmeBulletUtil::convert(pEnv, velocityVector, &bulletVector);

    pBody->setWindVelocity(bulletVector);
}
