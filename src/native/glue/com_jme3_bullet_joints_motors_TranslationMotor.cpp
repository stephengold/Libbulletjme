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
#include "com_jme3_bullet_joints_motors_TranslationMotor.h"
#include "jmeBulletUtil.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getBounce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getBounce
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_bounce, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getDamping
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getDamping
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_springDamping, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getEquilibrium
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getEquilibrium
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_equilibriumPoint, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getLowerLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getLowerLimit
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_lowerLimit, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getMaxMotorForce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getMaxMotorForce
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_maxMotorForce, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getParameter
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getParameter
    (JNIEnv *env, jobject object, jlong motorId, jint parameterIndex,
            jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        switch (parameterIndex) {
            case BT_CONSTRAINT_CFM:
                jmeBulletUtil::convert(env, &pMotor->m_motorCFM, storeVector);
                break;
            case BT_CONSTRAINT_ERP:
                jmeBulletUtil::convert(env, &pMotor->m_motorERP, storeVector);
                break;
            case BT_CONSTRAINT_STOP_CFM:
                jmeBulletUtil::convert(env, &pMotor->m_stopCFM, storeVector);
                break;
            case BT_CONSTRAINT_STOP_ERP:
                jmeBulletUtil::convert(env, &pMotor->m_stopERP, storeVector);
                break;
            default:
                btAssert(0);
        }
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getServoTarget
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getServoTarget
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_servoTarget, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getStiffness
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getStiffness
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_springStiffness, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getTargetVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getTargetVelocity
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_targetVelocity, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    getUpperLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_getUpperLimit
    (JNIEnv *env, jobject object, jlong motorId, jobject storeVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pMotor->m_upperLimit, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    isDampingLimited
     * Signature: (JI)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_isDampingLimited
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",
                JNI_FALSE);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = pMotor->m_springDampingLimited[axisIndex];
        return (jboolean) flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    isMotorEnabled
     * Signature: (JI)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_isMotorEnabled
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",
                JNI_FALSE);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = pMotor->m_enableMotor[axisIndex];
        return (jboolean) flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    isServoEnabled
     * Signature: (JI)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_isServoEnabled
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",
                JNI_FALSE);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = pMotor->m_servoMotor[axisIndex];
        return (jboolean) flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    isSpringEnabled
     * Signature: (JI)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_isSpringEnabled
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",
                JNI_FALSE);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = pMotor->m_enableSpring[axisIndex];
        return (jboolean) flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    isStiffnessLimited
     * Signature: (JI)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_isStiffnessLimited
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",
                JNI_FALSE);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = pMotor->m_springStiffnessLimited[axisIndex];
        return (jboolean) flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setBounce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setBounce
    (JNIEnv *env, jobject object, jlong motorId, jobject bounceVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(bounceVector, "The bounce vector does not exist.",)

        jmeBulletUtil::convert(env, bounceVector, &pMotor->m_bounce);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setDamping
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setDamping
    (JNIEnv *env, jobject object, jlong motorId, jobject dampingVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(dampingVector, "The damping vector does not exist.",)

        jmeBulletUtil::convert(env, dampingVector, &pMotor->m_springDamping);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setDampingLimited
     * Signature: (JIZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setDampingLimited
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex,
            jboolean limitFlag) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = (bool) limitFlag;
        pMotor->m_springDampingLimited[axisIndex] = flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setEquilibrium
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setEquilibrium
    (JNIEnv *env, jobject object, jlong motorId, jobject offsetVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(offsetVector, "The offset vector does not exist.",)

        jmeBulletUtil::convert(env, offsetVector, &pMotor->m_equilibriumPoint);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setLowerLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setLowerLimit
    (JNIEnv *env, jobject object, jlong motorId, jobject offsetVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(offsetVector, "The offset vector does not exist.",)

        jmeBulletUtil::convert(env, offsetVector, &pMotor->m_lowerLimit);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setMaxMotorForce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setMaxMotorForce
    (JNIEnv *env, jobject object, jlong motorId, jobject forceVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(forceVector, "The force vector does not exist.",)

        jmeBulletUtil::convert(env, forceVector, &pMotor->m_maxMotorForce);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setMotorEnabled
     * Signature: (JIZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setMotorEnabled
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex,
            jboolean enableFlag) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = (bool) enableFlag;
        pMotor->m_enableMotor[axisIndex] = flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setParameter
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setParameter
    (JNIEnv *env, jobject object, jlong motorId, jint parameterIndex,
            jobject vector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(vector, "The vector does not exist.",)

        switch (parameterIndex) {
            case BT_CONSTRAINT_CFM:
                jmeBulletUtil::convert(env, vector, &pMotor->m_motorCFM);
                break;
            case BT_CONSTRAINT_ERP:
                jmeBulletUtil::convert(env, vector, &pMotor->m_motorERP);
                break;
            case BT_CONSTRAINT_STOP_CFM:
                jmeBulletUtil::convert(env, vector, &pMotor->m_stopCFM);
                break;
            case BT_CONSTRAINT_STOP_ERP:
                jmeBulletUtil::convert(env, vector, &pMotor->m_stopERP);
                break;
            default:
                btAssert(0);
        }
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setServoEnabled
     * Signature: (JIZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setServoEnabled
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex, jboolean enableFlag) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = (bool) enableFlag;
        pMotor->m_servoMotor[axisIndex] = flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setServoTarget
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setServoTarget
    (JNIEnv *env, jobject object, jlong motorId, jobject targetVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(targetVector, "The target vector does not exist.",)

        jmeBulletUtil::convert(env, targetVector, &pMotor->m_servoTarget);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setSpringEnabled
     * Signature: (JIZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setSpringEnabled
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex,
            jboolean enableFlag) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = (bool) enableFlag;
        pMotor->m_enableSpring[axisIndex] = flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setStiffness
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setStiffness
    (JNIEnv *env, jobject object, jlong motorId, jobject stiffnessVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(stiffnessVector, "The stiffness vector does not exist.",)

        jmeBulletUtil::convert(env, stiffnessVector,
                &pMotor->m_springStiffness);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setStiffnessLimited
     * Signature: (JIZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setStiffnessLimited
    (JNIEnv *env, jobject object, jlong motorId, jint axisIndex,
            jboolean limitFlag) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        bool flag = (bool) limitFlag;
        pMotor->m_springStiffnessLimited[axisIndex] = flag;
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setTargetVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setTargetVelocity
    (JNIEnv *env, jobject object, jlong motorId, jobject velocityVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(velocityVector, "The velocity vector does not exist.",)

        jmeBulletUtil::convert(env, velocityVector, &pMotor->m_targetVelocity);
    }

    /*
     * Class:     com_jme3_bullet_joints_motors_TranslationMotor
     * Method:    setUpperLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_TranslationMotor_setUpperLimit
    (JNIEnv *env, jobject object, jlong motorId, jobject offsetVector) {
        btTranslationalLimitMotor2 *pMotor
                = reinterpret_cast<btTranslationalLimitMotor2 *> (motorId);
        NULL_CHECK(pMotor, "The btTranslationalLimitMotor2 does not exist.",)
        NULL_CHECK(offsetVector, "The offset vector does not exist.",)

        jmeBulletUtil::convert(env, offsetVector, &pMotor->m_upperLimit);
    }

#ifdef __cplusplus
}
#endif
