/*
 * Copyright (c) 2009-2015 jMonkeyEngine
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
#include "com_jme3_bullet_objects_infos_SoftBodyConfig.h"
#include "jmeClasses.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    copyValues
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_copyValues
(JNIEnv *pEnv, jclass, jlong destId, jlong sourceId) {
    btSoftBody *pDest = reinterpret_cast<btSoftBody *> (destId);
    NULL_CHK(pEnv, pDest, "The destination btSoftBody does not exist.",);
    ASSERT_CHK(pEnv, pDest->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    btSoftBody *pSource = reinterpret_cast<btSoftBody *> (sourceId);
    NULL_CHK(pEnv, pSource, "The source btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pSource->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    if (pDest != pSource) {
        pDest->m_cfg = pSource->m_cfg;
    }
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getAeroModel
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getAeroModel
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.aeromodel;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getAnchorsHardness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getAnchorsHardness
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kAHR;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getClusterIterations
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getClusterIterations
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.citerations;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getClusterKineticHardness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getClusterKineticHardness
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kSKHR_CL;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getClusterKineticImpulseSplitCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getClusterKineticImpulseSplitCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kSK_SPLT_CL;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getClusterRigidHardness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getClusterRigidHardness
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kSRHR_CL;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getClusterRigidImpulseSplitCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getClusterRigidImpulseSplitCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kSR_SPLT_CL;
}

/*
 * Class:     com_jme3_bullet_objects_objects_infos_SoftBodyConfig
 * Method:    getClusterSoftHardness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getClusterSoftHardness
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kSSHR_CL;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getClusterSoftImpulseSplitCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getClusterSoftImpulseSplitCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kSS_SPLT_CL;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getCollisionsFlags
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getCollisionsFlags
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.collisions;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getDampingCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getDampingCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kDP;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getDragCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getDragCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kDG;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getDriftIterations
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getDriftIterations
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.diterations;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getDynamicFrictionCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getDynamicFrictionCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kDF;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getKineticContactsHardness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getKineticContactsHardness
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kKHR;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getLiftCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getLiftCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kLF;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getMaximumVolumeRatio
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getMaximumVolumeRatio
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.maxvolume;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getPoseMatchingCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getPoseMatchingCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kMT;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getPositionIterations
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getPositionIterations
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.piterations;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getPressureCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getPressureCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kPR;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getRigidContactsHardness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getRigidContactsHardness
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kCHR;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getSoftContactsHardness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getSoftContactsHardness
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kSHR;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getTimeScale
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getTimeScale
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.timescale;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getVelocitiesCorrectionFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getVelocitiesCorrectionFactor
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kVCF;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getVelocitiesIterations
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getVelocitiesIterations
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.viterations;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    getVolumeConservationCoef
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_getVolumeConservationCoef
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    return pBody->m_cfg.kVC;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setAeroModel
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setAeroModel
(JNIEnv *pEnv, jclass, jlong bodyId, jint model) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.aeromodel = (btSoftBody::eAeroModel::_) model;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setAnchorsHardness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setAnchorsHardness
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kAHR = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setClusterIterations
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setClusterIterations
(JNIEnv *pEnv, jclass, jlong bodyId, jint iter) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.citerations = iter;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setClusterKineticHardness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setClusterKineticHardness
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kSKHR_CL = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setClusterKineticImpulseSplitCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setClusterKineticImpulseSplitCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kSK_SPLT_CL = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setClusterRigidHardness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setClusterRigidHardness
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kSRHR_CL = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setClusterRigidImpulseSplitCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setClusterRigidImpulseSplitCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kSR_SPLT_CL = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setClusterSoftHardness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setClusterSoftHardness
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kSSHR_CL = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setClusterSoftImpulseSplitCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setClusterSoftImpulseSplitCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kSS_SPLT_CL = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setCollisionsFlags
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setCollisionsFlags
(JNIEnv *pEnv, jclass, jlong bodyId, jint flags) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.collisions = flags;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setDampingCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setDampingCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat ceof) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kDP = ceof;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setDragCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setDragCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kDG = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setDriftIterations
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setDriftIterations
(JNIEnv *pEnv, jclass, jlong bodyId, jint iter) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.diterations = iter;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setDynamicFrictionCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setDynamicFrictionCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kDF = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setKineticContactsHardness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setKineticContactsHardness
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kKHR = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setLiftCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setLiftCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kLF = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setMaximumVolumeRatio
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setMaximumVolumeRatio
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat ratio) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.maxvolume = ratio;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setPoseMatchingCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setPoseMatchingCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kMT = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setPositionIterations
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setPositionIterations
(JNIEnv *pEnv, jclass, jlong bodyId, jint iter) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.piterations = iter;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setPressureCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setPressureCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kPR = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setRigidContactsHardness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setRigidContactsHardness
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kCHR = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setSoftContactsHardness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setSoftContactsHardness
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kSHR = coef;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setTimeScale
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setTimeScale
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat scale) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.timescale = scale;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setVelocitiesCorrectionFactor
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setVelocitiesCorrectionFactor
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat factor) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kVCF = factor;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setVelocitiesIterations
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setVelocitiesIterations
(JNIEnv *pEnv, jclass, jlong bodyId, jint iter) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.viterations = iter;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyConfig
 * Method:    setVolumeConservationCoef
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyConfig_setVolumeConservationCoef
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat coef) {
    btSoftBody *pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY,);

    pBody->m_cfg.kVC = coef;
}
