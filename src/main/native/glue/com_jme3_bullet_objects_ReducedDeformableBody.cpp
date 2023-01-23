/*
 * Copyright (c) 2022-2023 jMonkeyEngine
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
#include "com_jme3_bullet_objects_ReducedDeformableBody.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/BulletReducedDeformableBody/btReducedDeformableBody.h"

/*
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    create
 * Signature: (J[Lcom/jme3/math/Vector3f;[FI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_create
(JNIEnv *pEnv, jclass, jlong infoId, jobjectArray locations, jfloatArray masses,
        jint numNodes) {
    jmeClasses::initJavaClasses(pEnv);

    const int n = numNodes;
    btVector3 * const pLocations = new btVector3[n]; //dance037
    for (int i = 0; i < n; ++i) {
        jobject location = pEnv->GetObjectArrayElement(locations, i);
        EXCEPTION_CHK(pEnv, 0);
        jmeBulletUtil::convert(pEnv, location, &pLocations[i]);
        EXCEPTION_CHK(pEnv, 0);
    }

    btScalar *pMasses;
#ifdef BT_USE_DOUBLE_PRECISION
    float * const pFloats = pEnv->GetFloatArrayElements(masses, 0);
    EXCEPTION_CHK(pEnv, 0);
    pMasses = new btScalar[n]; //dance038
    for (int i = 0; i < n; ++i) {
        pMasses[i] = pFloats[i];
    }
#else
    pMasses = pEnv->GetFloatArrayElements(masses, 0);
    EXCEPTION_CHK(pEnv, 0);
#endif

    btSoftBodyWorldInfo * const
            pInfo = reinterpret_cast<btSoftBodyWorldInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btSoftBodyWorldInfo does not exist.", 0);

    btReducedDeformableBody * const pBody
            = new btReducedDeformableBody(pInfo, n, pLocations, pMasses); //dance014

#ifdef BT_USE_DOUBLE_PRECISION
    delete[] pMasses; //dance038
    pEnv->ReleaseFloatArrayElements(masses, pFloats, 0);
#else
    pEnv->ReleaseFloatArrayElements(masses, pMasses, 0);
#endif
    EXCEPTION_CHK(pEnv, 0);
    delete[] pLocations; //dance037

    pBody->getCollisionShape()->setMargin(CONVEX_DISTANCE_MARGIN);
    pBody->setUserPointer(NULL); // TODO unnecessary?

    btSoftBody::Material * const pMaterial = pBody->appendMaterial();
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
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    getLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_getLinearVelocity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btReducedDeformableBody * const
            pBody = reinterpret_cast<btReducedDeformableBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btReducedDeformableBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    btVector3 velocity = pBody->getLinearVelocity();
    jmeBulletUtil::convert(pEnv, &velocity, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    isReducedModesEnabled
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_isReducedModesEnabled
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btReducedDeformableBody * const
            pBody = reinterpret_cast<btReducedDeformableBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btReducedDeformableBody does not exist.", JNI_FALSE);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, JNI_FALSE);

    bool enabled = !pBody->isReducedModesOFF();
    return (jboolean) enabled;
}

/*
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    pinNode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_pinNode
(JNIEnv *pEnv, jclass, jlong bodyId, jint nodeIndex) {
    btReducedDeformableBody * const
            pBody = reinterpret_cast<btReducedDeformableBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btReducedDeformableBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    int node_i = nodeIndex;
    pBody->setFixedNodes(node_i);
}

/*
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    setDamping
 * Signature: (JFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_setDamping
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat alpha, jfloat beta) {
    btReducedDeformableBody * const
            pBody = reinterpret_cast<btReducedDeformableBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btReducedDeformableBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    btScalar a = alpha;
    btScalar b = beta;
    pBody->setDamping(a, b);
}

/*
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    setLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_setLinearVelocity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVector) {
    btReducedDeformableBody * const
            pBody = reinterpret_cast<btReducedDeformableBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btReducedDeformableBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);
    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",);

    btVector3 velocity;
    jmeBulletUtil::convert(pEnv, velocityVector, &velocity);
    EXCEPTION_CHK(pEnv,);
    pBody->setRigidVelocity(velocity);
}

/*
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    setReducedModes
 * Signature: (JII)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_setReducedModes
(JNIEnv *pEnv, jclass, jlong bodyId, jint numReduced, jint numFull) {
    btReducedDeformableBody * const
            pBody = reinterpret_cast<btReducedDeformableBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btReducedDeformableBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    int num_modes = numReduced;
    int full_size = numFull;
    pBody->setReducedModes(num_modes, full_size);
}

/*
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    setReducedModesEnabled
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_setReducedModesEnabled
(JNIEnv *pEnv, jclass, jlong bodyId, jboolean enabled) {
    btReducedDeformableBody * const
            pBody = reinterpret_cast<btReducedDeformableBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btReducedDeformableBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    bool rigid_only = !enabled;
    pBody->disableReducedModes(rigid_only);
}

/*
 * Class:     com_jme3_bullet_objects_ReducedDeformableBody
 * Method:    setStiffnessScale
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_ReducedDeformableBody_setStiffnessScale
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat scale) {
    btReducedDeformableBody * const
            pBody = reinterpret_cast<btReducedDeformableBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btReducedDeformableBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    btScalar ks = scale;
    pBody->setStiffnessScale(ks);
}
