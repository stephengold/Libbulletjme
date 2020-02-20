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
(JNIEnv *env, jobject object, jlong bodyIdA, jlong bodyIdB,
        jobject pivotInA, jobject rotInA, jobject pivotInB, jobject rotInB,
        jboolean useLinearReferenceFrameA) {
    jmeClasses::initJavaClasses(env);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHECK(pBodyA, "Rigid body A does not exist.", 0)
    btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHECK(pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHECK(pivotInA, "The pivotInA vector does not exist.", 0)
    NULL_CHECK(rotInA, "The rotInA matrix does not exist.", 0)
    btTransform frameInA;
    jmeBulletUtil::convert(env, pivotInA, &frameInA.getOrigin());
    jmeBulletUtil::convert(env, rotInA, &frameInA.getBasis());

    NULL_CHECK(pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHECK(rotInB, "The rotInB matrix does not exist.", 0)
    btTransform frameInB;
    jmeBulletUtil::convert(env, pivotInB, &frameInB.getOrigin());
    jmeBulletUtil::convert(env, rotInB, &frameInB.getBasis());

    btSliderConstraint *pJoint = new btSliderConstraint(*pBodyA, *pBodyB,
            frameInA, frameInB, useLinearReferenceFrameA);

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SliderJoint_createJoint1
(JNIEnv *env, jobject object, jlong bodyIdB, jobject pivotInB,
        jobject rotInB, jboolean useLinearReferenceFrameA) {
    jmeClasses::initJavaClasses(env);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHECK(pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHECK(pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHECK(rotInB, "The rotInB matrix does not exist.", 0)
    btTransform frameInB;
    jmeBulletUtil::convert(env, pivotInB, &frameInB.getOrigin());
    jmeBulletUtil::convert(env, rotInB, &frameInB.getBasis());

    btSliderConstraint *pJoint = new btSliderConstraint(*pBodyB, frameInB,
            useLinearReferenceFrameA);

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingDirAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingDirAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingDirAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingDirLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingDirLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingDirLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingLimAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingLimAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingLimAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingLimLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingLimLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingLimLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingOrthoAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingOrthoAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingOrthoAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getDampingOrthoLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getDampingOrthoLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getDampingOrthoLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getFrameOffsetA
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_getFrameOffsetA
(JNIEnv *env, jobject object, jlong jointId, jobject storeTransform) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)
    NULL_CHECK(storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetA();
    jmeBulletUtil::convert(env, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getFrameOffsetB
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_getFrameOffsetB
(JNIEnv *env, jobject object, jlong jointId, jobject storeTransform) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)
    NULL_CHECK(storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetB();
    jmeBulletUtil::convert(env, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getLowerAngLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getLowerAngLimit
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getLowerAngLimit();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getLowerLinLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getLowerLinLimit
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getLowerLinLimit();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getMaxAngMotorForce
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getMaxAngMotorForce
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getMaxAngMotorForce();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getMaxLinMotorForce
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getMaxLinMotorForce
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getMaxLinMotorForce();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionDirAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionDirAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionDirAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionDirLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionDirLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionDirLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionLimAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionLimAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionLimAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionLimLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionLimLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionLimLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionOrthoAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionOrthoAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionOrthoAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getRestitutionOrthoLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getRestitutionOrthoLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getRestitutionOrthoLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessDirAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessDirAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessDirAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessDirLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessDirLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessDirLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessLimAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessLimAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessLimAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessLimLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessLimLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessLimLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessOrthoAng
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessOrthoAng
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessOrthoAng();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getSoftnessOrthoLin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getSoftnessOrthoLin
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getSoftnessOrthoLin();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getTargetAngMotorVelocity
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getTargetAngMotorVelocity
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getTargetAngMotorVelocity();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getTargetLinMotorVelocity
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getTargetLinMotorVelocity
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getTargetLinMotorVelocity();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getUpperAngLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getUpperAngLimit
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getUpperAngLimit();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    getUpperLinLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SliderJoint_getUpperLinLimit
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", 0)

    return pJoint->getUpperLinLimit();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    isPoweredAngMotor
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_SliderJoint_isPoweredAngMotor
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", JNI_FALSE)

    return pJoint->getPoweredAngMotor();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    isPoweredLinMotor
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_SliderJoint_isPoweredLinMotor
(JNIEnv *env, jobject object, jlong jointId) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.", JNI_FALSE)

    return pJoint->getPoweredLinMotor();
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingDirAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingDirAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingDirAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingDirLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingDirLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingDirLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingLimAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingLimAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingLimAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingLimLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingLimLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingLimLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingOrthoAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingOrthoAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingOrthoAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setDampingOrthoLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setDampingOrthoLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setDampingOrthoLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setLowerAngLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setLowerAngLimit
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setLowerAngLimit(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setLowerLinLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setLowerLinLimit
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setLowerLinLimit(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setMaxAngMotorForce
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setMaxAngMotorForce
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setMaxAngMotorForce(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setMaxLinMotorForce
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setMaxLinMotorForce
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setMaxLinMotorForce(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setPoweredAngMotor
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setPoweredAngMotor
(JNIEnv *env, jobject object, jlong jointId, jboolean value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setPoweredAngMotor(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setPoweredLinMotor
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setPoweredLinMotor
(JNIEnv *env, jobject object, jlong jointId, jboolean value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setPoweredLinMotor(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionDirAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionDirAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionDirAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionDirLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionDirLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionDirLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionLimAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionLimAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionLimAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionLimLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionLimLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionLimLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionOrthoAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionOrthoAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionOrthoAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setRestitutionOrthoLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setRestitutionOrthoLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setRestitutionOrthoLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessDirAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessDirAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessDirAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessDirLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessDirLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessDirLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessLimAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessLimAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessLimAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessLimLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessLimLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessLimLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessOrthoAng
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessOrthoAng
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessOrthoAng(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setSoftnessOrthoLin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setSoftnessOrthoLin
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setSoftnessOrthoLin(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setTargetAngMotorVelocity
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setTargetAngMotorVelocity
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setTargetAngMotorVelocity(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setTargetLinMotorVelocity
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setTargetLinMotorVelocity
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setTargetLinMotorVelocity(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setUpperAngLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setUpperAngLimit
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setUpperAngLimit(value);
}

/*
 * Class:     com_jme3_bullet_joints_SliderJoint
 * Method:    setUpperLinLimit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SliderJoint_setUpperLinLimit
(JNIEnv *env, jobject object, jlong jointId, jfloat value) {
    btSliderConstraint *pJoint
            = reinterpret_cast<btSliderConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btSliderConstraint does not exist.",)

    pJoint->setUpperLinLimit(value);
}
