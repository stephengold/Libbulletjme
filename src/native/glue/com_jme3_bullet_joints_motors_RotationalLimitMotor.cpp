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
#include "com_jme3_bullet_joints_motors_RotationalLimitMotor.h"
#include "jmeBulletUtil.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getAccumulatedImpulse
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getAccumulatedImpulse
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_accumulatedImpulse;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getBounce
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getBounce
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_bounce;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getCurrentPosition
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getCurrentPosition
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_currentPosition;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getDamping
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_damping;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getERP
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getERP
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_stopERP;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getHiLimit
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getHiLimit
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_hiLimit;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getLimitSoftness
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getLimitSoftness
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_limitSoftness;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getLoLimit
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getLoLimit
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_loLimit;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getMaxLimitForce
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getMaxLimitForce
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_maxLimitForce;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getMaxMotorForce
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getMaxMotorForce
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_maxMotorForce;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getNormalCFM
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getNormalCFM
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_normalCFM;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getStopCFM
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getStopCFM
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_stopCFM;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    getTargetVelocity
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_getTargetVelocity
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.", 0)

        return pMotor->m_targetVelocity;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    isEnableMotor
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_isEnableMotor
    (JNIEnv *env, jobject object, jlong motorId) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",
                JNI_FALSE)

        return pMotor->m_enableMotor;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setAccumulatedImpulse
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setAccumulatedImpulse
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_accumulatedImpulse = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setBounce
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setBounce
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_bounce = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setDamping
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setDamping
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_damping = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setEnableMotor
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setEnableMotor
    (JNIEnv *env, jobject object, jlong motorId, jboolean value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_enableMotor = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setERP
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setERP
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_stopERP = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setHiLimit
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setHiLimit
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_hiLimit = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setLimitSoftness
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setLimitSoftness
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_limitSoftness = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setLoLimit
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setLoLimit
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_loLimit = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setMaxLimitForce
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setMaxLimitForce
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_maxLimitForce = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setMaxMotorForce
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setMaxMotorForce
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_maxMotorForce = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setNormalCFM
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setNormalCFM
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_normalCFM = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setStopCFM
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setStopCFM
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_stopCFM = value;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_RotationalLimitMotor
     * Method:    setTargetVelocity
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationalLimitMotor_setTargetVelocity
    (JNIEnv *env, jobject object, jlong motorId, jfloat value) {
        btRotationalLimitMotor *pMotor
                = reinterpret_cast<btRotationalLimitMotor *> (motorId);
        NULL_CHECK(pMotor, "The btRotationalLimitMotor does not exist.",)

        pMotor->m_targetVelocity = value;
    }

#ifdef __cplusplus
}
#endif
