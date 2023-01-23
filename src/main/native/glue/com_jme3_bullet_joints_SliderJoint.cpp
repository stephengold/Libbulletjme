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
#include "com_jme3_bullet_joints_SliderJoint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SliderJoint_createJoint
(JNIEnv *pEnv, jclass, jlong bodyIdA, jlong bodyIdB, jobject pivotInA,
        jobject rotInA, jobject pivotInB, jobject rotInB,
        jboolean useLinearReferenceFrameA) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    NULL_CHK(pEnv, pivotInA, "The pivotInA vector does not exist.", 0)
    NULL_CHK(pEnv, rotInA, "The rotInA matrix does not exist.", 0)
    btTransform frameInA;
    jmeBulletUtil::convert(pEnv, pivotInA, &frameInA.getOrigin());
    EXCEPTION_CHK(pEnv, 0);
    jmeBulletUtil::convert(pEnv, rotInA, &frameInA.getBasis());
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHK(pEnv, rotInB, "The rotInB matrix does not exist.", 0)
    btTransform frameInB;
    jmeBulletUtil::convert(pEnv, pivotInB, &frameInB.getOrigin());
    EXCEPTION_CHK(pEnv, 0);
    jmeBulletUtil::convert(pEnv, rotInB, &frameInB.getBasis());
    EXCEPTION_CHK(pEnv, 0);

    btSliderConstraint *pJoint = new btSliderConstraint(*pBodyA, *pBodyB,
            frameInA, frameInB, useLinearReferenceFrameA); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SliderJoint_createJoint1
(JNIEnv *pEnv, jclass, jlong bodyIdB, jobject pivotInB, jobject rotInB,
        jboolean useLinearReferenceFrameA) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    NULL_CHK(pEnv, pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHK(pEnv, rotInB, "The rotInB matrix does not exist.", 0)
    btTransform frameInB;
    jmeBulletUtil::convert(pEnv, pivotInB, &frameInB.getOrigin());
    EXCEPTION_CHK(pEnv, 0);
    jmeBulletUtil::convert(pEnv, rotInB, &frameInB.getBasis());
    EXCEPTION_CHK(pEnv, 0);

    btSliderConstraint *pJoint = new btSliderConstraint(*pBodyB, frameInB,
            useLinearReferenceFrameA); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingDirAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingDirAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingDirAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingDirLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingDirLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingDirLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingLimAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingLimAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingLimAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingLimLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingLimLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingLimLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingOrthoAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingOrthoAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingOrthoAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingOrthoLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingOrthoLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingOrthoLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getFrameOffsetA
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_getFrameOffsetA
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeTransform) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)
    NULL_CHK(pEnv, storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetA();
    jmeBulletUtil::convert(pEnv, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getFrameOffsetB
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_getFrameOffsetB
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeTransform) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)
    NULL_CHK(pEnv, storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetB();
    jmeBulletUtil::convert(pEnv, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getLowerAngLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getLowerAngLimit
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getLowerAngLimit();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getLowerLinLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getLowerLinLimit
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getLowerLinLimit();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getMaxAngMotorForce
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getMaxAngMotorForce
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getMaxAngMotorForce();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getMaxLinMotorForce
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getMaxLinMotorForce
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getMaxLinMotorForce();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionDirAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionDirAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionDirAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionDirLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionDirLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionDirLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionLimAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionLimAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionLimAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionLimLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionLimLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionLimLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionOrthoAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionOrthoAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionOrthoAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionOrthoLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionOrthoLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionOrthoLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessDirAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessDirAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessDirAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessDirLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessDirLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessDirLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessLimAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessLimAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessLimAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessLimLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessLimLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessLimLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessOrthoAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessOrthoAng
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessOrthoAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessOrthoLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessOrthoLin
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessOrthoLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getTargetAngMotorVelocity
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getTargetAngMotorVelocity
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getTargetAngMotorVelocity();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getTargetLinMotorVelocity
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getTargetLinMotorVelocity
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getTargetLinMotorVelocity();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getUpperAngLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getUpperAngLimit
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getUpperAngLimit();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getUpperLinLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getUpperLinLimit
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getUpperLinLimit();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    isPoweredAngMotor
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_SliderJoint_isPoweredAngMotor
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", JNI_FALSE)

    return pJoint->getPoweredAngMotor();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    isPoweredLinMotor
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_SliderJoint_isPoweredLinMotor
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.", JNI_FALSE)

    return pJoint->getPoweredLinMotor();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingDirAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingDirAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingDirAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingDirLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingDirLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingDirLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingLimAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingLimAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingLimAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingLimLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingLimLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingLimLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingOrthoAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingOrthoAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingOrthoAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingOrthoLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingOrthoLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingOrthoLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setLowerAngLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setLowerAngLimit
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setLowerAngLimit(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setLowerLinLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setLowerLinLimit
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setLowerLinLimit(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setMaxAngMotorForce
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setMaxAngMotorForce
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setMaxAngMotorForce(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setMaxLinMotorForce
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setMaxLinMotorForce
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setMaxLinMotorForce(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setPoweredAngMotor
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setPoweredAngMotor
(JNIEnv *pEnv, jclass, jlong jointId, jboolean value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setPoweredAngMotor(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setPoweredLinMotor
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setPoweredLinMotor
(JNIEnv *pEnv, jclass, jlong jointId, jboolean value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setPoweredLinMotor(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionDirAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionDirAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionDirAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionDirLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionDirLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionDirLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionLimAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionLimAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionLimAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionLimLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionLimLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionLimLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionOrthoAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionOrthoAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionOrthoAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionOrthoLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionOrthoLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionOrthoLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessDirAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessDirAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessDirAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessDirLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessDirLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessDirLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessLimAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessLimAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessLimAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessLimLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessLimLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessLimLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessOrthoAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessOrthoAng
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessOrthoAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessOrthoLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessOrthoLin
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessOrthoLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setTargetAngMotorVelocity
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setTargetAngMotorVelocity
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setTargetAngMotorVelocity(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setTargetLinMotorVelocity
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setTargetLinMotorVelocity
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setTargetLinMotorVelocity(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setUpperAngLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setUpperAngLimit
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setUpperAngLimit(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setUpperLinLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setUpperLinLimit
(JNIEnv *pEnv, jclass, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setUpperLinLimit(value);
}
