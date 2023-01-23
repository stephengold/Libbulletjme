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
#include "com_jme3_bullet_joints_HingeJoint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_HingeJoint_createJoint
(JNIEnv *pEnv, jclass, jlong bodyIdA, jlong bodyIdB, jobject pivotInA,
        jobject axisInA, jobject pivotInB, jobject axisInB) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    NULL_CHK(pEnv, pivotInA, "The pivotInA vector does not exist.", 0)
    btVector3 pivotA;
    jmeBulletUtil::convert(pEnv, pivotInA, &pivotA);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, pivotInB, "The pivotInB vector does not exist.", 0)
    btVector3 pivotB;
    jmeBulletUtil::convert(pEnv, pivotInB, &pivotB);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, axisInA, "The axisInA vector does not exist.", 0)
    btVector3 axisA;
    jmeBulletUtil::convert(pEnv, axisInA, &axisA);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, axisInB, "The axisInB vector does not exist.", 0)
    btVector3 axisB;
    jmeBulletUtil::convert(pEnv, axisInB, &axisB);
    EXCEPTION_CHK(pEnv, 0);

    btHingeConstraint *
            pJoint = new btHingeConstraint(*pBodyA, *pBodyB, pivotA, pivotB,
            axisA, axisB); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_HingeJoint_createJoint1
(JNIEnv *pEnv, jclass, jlong bodyIdA, jobject pivotInA, jobject axisInA,
        jboolean useReferenceFrameA) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    NULL_CHK(pEnv, pivotInA, "The pivotInA vector does not exist.", 0)
    btVector3 pivot;
    jmeBulletUtil::convert(pEnv, pivotInA, &pivot);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, axisInA, "The axisInA vector does not exist.", 0)
    btVector3 axis;
    jmeBulletUtil::convert(pEnv, axisInA, &axis);
    EXCEPTION_CHK(pEnv, 0);

    btHingeConstraint *pJoint = new btHingeConstraint(*pBodyA, pivot, axis,
            useReferenceFrameA); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    enableMotor
 * Signature: (JZFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_enableMotor
(JNIEnv *pEnv, jclass, jlong jointId, jboolean enable, jfloat targetVelocity,
        jfloat maxMotorImpulse) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE,);

    pJoint->enableAngularMotor(enable, targetVelocity, maxMotorImpulse);
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    getEnableAngularMotor
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_HingeJoint_getEnableAngularMotor
(JNIEnv *pEnv, jclass, jlong jointId) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.", JNI_FALSE)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE, JNI_FALSE);

    return pJoint->getEnableAngularMotor();
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    getFrameOffsetA
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_getFrameOffsetA
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeTransform) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE,);

    NULL_CHK(pEnv, storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetA();
    jmeBulletUtil::convert(pEnv, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    getFrameOffsetB
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_getFrameOffsetB
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeTransform) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE,);

    NULL_CHK(pEnv, storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetB();
    jmeBulletUtil::convert(pEnv, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    getHingeAngle
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getHingeAngle
(JNIEnv *pEnv, jclass, jlong jointId) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE, 0);

    return pJoint->getHingeAngle();
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    getLowerLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getLowerLimit
(JNIEnv *pEnv, jclass, jlong jointId) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE, 0);

    return pJoint->getLowerLimit();
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    getMaxMotorImpulse
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getMaxMotorImpulse
(JNIEnv *pEnv, jclass, jlong jointId) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE, 0);

    return pJoint->getMaxMotorImpulse();
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    getMotorTargetVelocity
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getMotorTargetVelocity
(JNIEnv *pEnv, jclass, jlong jointId) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE, 0);

    return pJoint->getMotorTargetVelocity();
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    getUpperLimit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getUpperLimit
(JNIEnv *pEnv, jclass, jlong jointId) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE, 0);

    return pJoint->getUpperLimit();
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    setAngularOnly
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_setAngularOnly
(JNIEnv *pEnv, jclass, jlong jointId, jboolean angular) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE,);

    pJoint->setAngularOnly(angular);
}

/*
 * Class:     com_jme3_bullet_joints_HingeJoint
 * Method:    setLimit
 * Signature: (JFFFFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_setLimit
(JNIEnv *pEnv, jclass, jlong jointId, jfloat low, jfloat high,
        jfloat softness, jfloat biasFactor, jfloat relaxationFactor) {
    btHingeConstraint *pJoint
            = reinterpret_cast<btHingeConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btHingeConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE,);

    return pJoint->setLimit(low, high, softness, biasFactor,
            relaxationFactor);
}
