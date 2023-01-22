/*
 * Copyright (c) 2019-2020 jMonkeyEngine
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
#include "com_jme3_bullet_joints_motors_RotationMotor.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getBounce
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getBounce
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_bounce;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getDamping
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_springDamping;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getEquilibrium
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getEquilibrium
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_equilibriumPoint;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getLowerLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getLowerLimit
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_loLimit;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getMaxMotorForce
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getMaxMotorForce
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_maxMotorForce;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getParameter
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getParameter
(JNIEnv *pEnv, jclass, jlong motorId, jint parameterIndex) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar;
    switch (parameterIndex) {
        case BT_CONSTRAINT_CFM:
            scalar = pMotor->m_motorCFM;
            break;
        case BT_CONSTRAINT_ERP:
            scalar = pMotor->m_motorERP;
            break;
        case BT_CONSTRAINT_STOP_CFM:
            scalar = pMotor->m_stopCFM;
            break;
        case BT_CONSTRAINT_STOP_ERP:
            scalar = pMotor->m_stopERP;
            break;
        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The parameter is unknown.");
            return 0;
    }

    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getServoTarget
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getServoTarget
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_servoTarget;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getStiffness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getStiffness
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_springStiffness;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getTargetVelocity
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getTargetVelocity
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_targetVelocity;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    getUpperLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_getUpperLimit
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.", 0)

    btScalar scalar = pMotor->m_hiLimit;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    isDampingLimited
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_isDampingLimited
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",
            JNI_FALSE);

    bool flag = pMotor->m_springDampingLimited;
    return (jboolean) flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    isMotorEnabled
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_isMotorEnabled
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",
            JNI_FALSE);

    bool flag = pMotor->m_enableMotor;
    return (jboolean) flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    isServoEnabled
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_isServoEnabled
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",
            JNI_FALSE);

    bool flag = pMotor->m_servoMotor;
    return (jboolean) flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    isSpringEnabled
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_isSpringEnabled
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",
            JNI_FALSE);

    bool flag = pMotor->m_enableSpring;
    return (jboolean) flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    isStiffnessLimited
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_isStiffnessLimited
(JNIEnv *pEnv, jclass, jlong motorId) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",
            JNI_FALSE);

    bool flag = pMotor->m_springStiffnessLimited;
    return (jboolean) flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setBounce
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setBounce
(JNIEnv *pEnv, jclass, jlong motorId, jfloat bounce) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) bounce;
    pMotor->m_bounce = scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setDamping
(JNIEnv *pEnv, jclass, jlong motorId, jfloat damping) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) damping;
    pMotor->m_springDamping = scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setDampingLimited
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setDampingLimited
(JNIEnv *pEnv, jclass, jlong motorId, jboolean limitFlag) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",);

    bool flag = (bool) limitFlag;
    pMotor->m_springDampingLimited = flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setEquilibrium
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setEquilibrium
(JNIEnv *pEnv, jclass, jlong motorId, jfloat angle) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) angle;
    pMotor->m_equilibriumPoint = scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setLowerLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setLowerLimit
(JNIEnv *pEnv, jclass, jlong motorId, jfloat angle) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) angle;
    pMotor->m_loLimit = scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setMaxMotorForce
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setMaxMotorForce
(JNIEnv *pEnv, jclass, jlong motorId, jfloat force) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) force;
    pMotor->m_maxMotorForce = scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setMotorEnabled
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setMotorEnabled
(JNIEnv *pEnv, jclass, jlong motorId, jboolean enableFlag) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",);

    bool flag = (bool) enableFlag;
    pMotor->m_enableMotor = flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setParameter
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setParameter
(JNIEnv *pEnv, jclass, jlong motorId, jint parameterIndex,
        jfloat value) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) value;
    switch (parameterIndex) {
        case BT_CONSTRAINT_CFM:
            pMotor->m_motorCFM = scalar;
            break;
        case BT_CONSTRAINT_ERP:
            pMotor->m_motorERP = scalar;
            break;
        case BT_CONSTRAINT_STOP_CFM:
            pMotor->m_stopCFM = scalar;
            break;
        case BT_CONSTRAINT_STOP_ERP:
            pMotor->m_stopERP = scalar;
            break;
        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The parameter is unknown.");
    }
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setServoEnabled
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setServoEnabled
(JNIEnv *pEnv, jclass, jlong motorId, jboolean enableFlag) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",);

    bool flag = (bool) enableFlag;
    pMotor->m_servoMotor = flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setServoTarget
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setServoTarget
(JNIEnv *pEnv, jclass, jlong motorId, jfloat target) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) target;
    pMotor->m_servoTarget = scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setSpringEnabled
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setSpringEnabled
(JNIEnv *pEnv, jclass, jlong motorId, jboolean enableFlag) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",);

    bool flag = (bool) enableFlag;
    pMotor->m_enableSpring = flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setStiffness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setStiffness
(JNIEnv *pEnv, jclass, jlong motorId, jfloat stiffness) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) stiffness;
    pMotor->m_springStiffness = scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setStiffnessLimited
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setStiffnessLimited
(JNIEnv *pEnv, jclass, jlong motorId, jboolean limitFlag) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",);

    bool flag = (bool) limitFlag;
    pMotor->m_springStiffnessLimited = flag;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setTargetVelocity
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setTargetVelocity
(JNIEnv *pEnv, jclass, jlong motorId, jfloat velocity) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) velocity;
    pMotor->m_targetVelocity = scalar;
}

/*
 * Class:     com_jme3_bullet_joints_motors_RotationMotor
 * Method:    setUpperLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_motors_RotationMotor_setUpperLimit
(JNIEnv *pEnv, jclass, jlong motorId, jfloat angle) {
    btRotationalLimitMotor2 *pMotor
            = reinterpret_cast<btRotationalLimitMotor2 *> (motorId);
    NULL_CHK(pEnv, pMotor, "The btRotationalLimitMotor2 does not exist.",)

    btScalar scalar = (btScalar) angle;
    pMotor->m_hiLimit = scalar;
}