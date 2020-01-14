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
 * Author: Normen Hansen
 */
#include "com_jme3_bullet_joints_motors_TranslationalLimitMotor.h"
#include "jmeBulletUtil.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getAccumulatedImpulse
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getAccumulatedImpulse
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, storeVector, "The storeVector does not exist.",);

        jmeBulletUtil::convert(env, &pMotor->m_accumulatedImpulse, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getDamping
    (JNIEnv *env, jobject object, jlong motorId) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.", 0)

        return pMotor->m_damping;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getERP
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getERP
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        NULL_CHECK(env, storeVector, "The store vector does not exist.",);
        jmeBulletUtil::convert(env, &pMotor->m_stopERP, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getLimitSoftness
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getLimitSoftness
    (JNIEnv *env, jobject object, jlong motorId) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.", 0)

        return pMotor->m_limitSoftness;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getLowerLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getLowerLimit
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, storeVector, "The store vector does not exist.",);

        jmeBulletUtil::convert(env, &pMotor->m_lowerLimit, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getMaxMotorForce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getMaxMotorForce
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        NULL_CHECK(env, storeVector, "The store vector does not exist.",);
        jmeBulletUtil::convert(env, &pMotor->m_maxMotorForce, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getNormalCFM
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getNormalCFM
    (JNIEnv *env, jobject object, jlong motorId, jobject storeResult) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        NULL_CHECK(env, storeResult, "The store vector does not exist.",);
        jmeBulletUtil::convert(env, &pMotor->m_normalCFM, storeResult);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getOffset
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getOffset
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        NULL_CHECK(env, storeVector, "The store vector does not exist.",);
        jmeBulletUtil::convert(env, &pMotor->m_currentLinearDiff, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getRestitution
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getRestitution
    (JNIEnv *env, jobject object, jlong motorId) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.", 0);

        return pMotor->m_restitution;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getStopCFM
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getStopCFM
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        NULL_CHECK(env, storeVector, "The store vector does not exist.",);
        jmeBulletUtil::convert(env, &pMotor->m_stopCFM, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getTargetVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getTargetVelocity
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        NULL_CHECK(env, storeVector, "The store vector does not exist.",);
        jmeBulletUtil::convert(env, &pMotor->m_targetVelocity, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    getUpperLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_getUpperLimit
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, storeVector, "The store vector does not exist.",);

        jmeBulletUtil::convert(env, &pMotor->m_upperLimit, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    isEnabled
     * Signature: (JI)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_isEnabled
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor, "The btTranslationalLimitMotor does not exist.",
                JNI_FALSE);

        bool result = pMotor->m_enableMotor[axisIndex];
        return (jboolean) result;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setAccumulatedImpulse
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setAccumulatedImpulse
    (JNIEnv *env, jobject object, jlong motorId, jobject impulseVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, impulseVector, "The impulseVector does not exist.",);

        jmeBulletUtil::convert(env, impulseVector,
                &pMotor->m_accumulatedImpulse);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setDamping
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setDamping
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        pMotor->m_damping = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setEnabled
     * Signature: (JIZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setEnabled
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex, jboolean newSetting) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        pMotor->m_enableMotor[axisIndex] = (bool)newSetting;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setERP
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setERP
    (JNIEnv *env, jobject object, jlong motorId, jobject erpVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, erpVector, "The ERP vector does not exist.",);

        jmeBulletUtil::convert(env, erpVector, &pMotor->m_stopERP);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setLimitSoftness
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setLimitSoftness
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        pMotor->m_limitSoftness = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setLowerLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setLowerLimit
    (JNIEnv *env, jobject object, jlong motorId, jobject limitVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, limitVector, "The limit vector does not exist.",);

        jmeBulletUtil::convert(env, limitVector, &pMotor->m_lowerLimit);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setMaxMotorForce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setMaxMotorForce
    (JNIEnv *env, jobject object, jlong motorId, jobject forceVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, forceVector, "The force vector does not exist.",);

        jmeBulletUtil::convert(env, forceVector, &pMotor->m_maxMotorForce);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setNormalCFM
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setNormalCFM
    (JNIEnv *env, jobject object, jlong motorId, jobject cfmVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, cfmVector, "The CFM vector does not exist.",);

        jmeBulletUtil::convert(env, cfmVector, &pMotor->m_normalCFM);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setRestitution
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setRestitution
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);

        pMotor->m_restitution = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setStopCFM
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setStopCFM
    (JNIEnv *env, jobject object, jlong motorId, jobject cfmVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, cfmVector, "The CFM vector does not exist.",);

        jmeBulletUtil::convert(env, cfmVector, &pMotor->m_stopCFM);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setTargetVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setTargetVelocity
    (JNIEnv *env, jobject object, jlong motorId, jobject velocityVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, velocityVector, "The velocity vector does not exist.",);

        jmeBulletUtil::convert(env, velocityVector, &pMotor->m_targetVelocity);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationalLimitMotor
     * Method:    setUpperLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationalLimitMotor_setUpperLimit
    (JNIEnv *env, jobject object, jlong motorId, jobject limitVector) {
        btTranslationalLimitMotor *pMotor
                = reinterpret_cast<btTranslationalLimitMotor *> (motorId);
        NULL_CHECK(env, pMotor,
                "The btTranslationalLimitMotor does not exist.",);
        NULL_CHECK(env, limitVector, "The limit vector does not exist.",);

        jmeBulletUtil::convert(env, limitVector, &pMotor->m_upperLimit);
    }

#ifdef __cplusplus
}
#endif
