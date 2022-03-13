/*
 * Copyright (c) 2020-2022 jMonkeyEngine
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
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "com_jme3_bullet_SolverInfo.h"
#include "jmeBulletUtil.h"

/*
 * Author: Stephen Gold
 */

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    copyAllParameters
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_copyAllParameters
(JNIEnv *pEnv, jclass, jlong targetId, jlong sourceId) {
    btContactSolverInfo * const
            pTarget = reinterpret_cast<btContactSolverInfo *> (targetId);
    NULL_CHK(pEnv, pTarget, "The target btContactSolverInfo does not exist.",);

    const btContactSolverInfo * const
            pSource = reinterpret_cast<btContactSolverInfo *> (sourceId);
    NULL_CHK(pEnv, pSource, "The source btContactSolverInfo does not exist.",);

    if (pTarget != pSource) {
        *pTarget = *pSource;
    }
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    getContactErp
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_SolverInfo_getContactErp
(JNIEnv *pEnv, jclass, jlong infoId) {
    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", 0);

    btScalar result = pInfo->m_erp2;
    return jfloat(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    getGlobalCfm
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_SolverInfo_getGlobalCfm
(JNIEnv *pEnv, jclass, jlong infoId) {
    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", 0);

    btScalar result = pInfo->m_globalCfm;
    return jfloat(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    getJointErp
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_SolverInfo_getJointErp
(JNIEnv *pEnv, jclass, jlong infoId) {
    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", 0);

    btScalar result = pInfo->m_erp;
    return jfloat(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    getMinBatch
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_SolverInfo_getMinBatch
(JNIEnv *pEnv, jclass, jlong infoId) {
    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", 0);

    int result = pInfo->m_minimumSolverBatchSize;
    return jint(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    getMode
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_SolverInfo_getMode
(JNIEnv *pEnv, jclass, jlong infoId) {
    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", 0);

    int result = pInfo->m_solverMode;
    return jint(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    getNumIterations
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_SolverInfo_getNumIterations
(JNIEnv *pEnv, jclass, jlong infoId) {
    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", 0);

    int result = pInfo->m_numIterations;
    return jint(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    getSplitImpulseErp
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_SolverInfo_getSplitImpulseErp
(JNIEnv *pEnv, jclass, jlong infoId) {
    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", 0);

    btScalar result = pInfo->m_splitImpulseTurnErp;
    return jfloat(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    getSplitImpulseThreshold
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_SolverInfo_getSplitImpulseThreshold
(JNIEnv *pEnv, jclass, jlong infoId) {
    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", 0);

    btScalar result = pInfo->m_splitImpulsePenetrationThreshold;
    return jfloat(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    isSplitImpulseEnabled
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_SolverInfo_isSplitImpulseEnabled
(JNIEnv *pEnv, jclass, jlong infoId) {

    const btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.", JNI_FALSE);

    int result = pInfo->m_splitImpulse;
    return jint(result);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setContactErp
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setContactErp
(JNIEnv *pEnv, jclass, jlong infoId, jfloat erp) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_erp2 = btScalar(erp);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setGlobalCfm
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setGlobalCfm
(JNIEnv *pEnv, jclass, jlong infoId, jfloat cfm) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_globalCfm = btScalar(cfm);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setJointErp
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setJointErp
(JNIEnv *pEnv, jclass, jlong infoId, jfloat erp) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_erp = btScalar(erp);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setMinBatch
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setMinBatch
(JNIEnv *pEnv, jclass, jlong infoId, jint numConstraints) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_minimumSolverBatchSize = int(numConstraints);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setMode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setMode
(JNIEnv *pEnv, jclass, jlong infoId, jint flags) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_solverMode = int(flags);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setNumIterations
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setNumIterations
(JNIEnv *pEnv, jclass, jlong infoId, jint numIterations) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_numIterations = int(numIterations);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setSplitImpulseEnabled
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setSplitImpulseEnabled
(JNIEnv *pEnv, jclass, jlong infoId, jboolean enable) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_splitImpulse = int(enable);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setSplitImpulseErp
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setSplitImpulseErp
(JNIEnv *pEnv, jclass, jlong infoId, jfloat erp) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_splitImpulseTurnErp = btScalar(erp);
}

/*
 * Class:     com_jme3_bullet_SolverInfo
 * Method:    setSplitImpulseThreshold
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_SolverInfo_setSplitImpulseThreshold
(JNIEnv *pEnv, jclass, jlong infoId, jfloat penetration) {
    btContactSolverInfo * const
            pInfo = reinterpret_cast<btContactSolverInfo *> (infoId);
    NULL_CHK(pEnv, pInfo, "The btContactSolverInfo does not exist.",)

    pInfo->m_splitImpulsePenetrationThreshold = btScalar(penetration);
}