/*
 * Copyright (c) 2019-2022 jMonkeyEngine
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
#include "com_jme3_bullet_joints_New6Dof.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    createDoubleEnded
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;I)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_New6Dof_createDoubleEnded
(JNIEnv *pEnv, jclass, jlong bodyIdA, jlong bodyIdB, jobject pivotInA,
        jobject rotInA, jobject pivotInB, jobject rotInB, jint rotOrderIndex) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHK(pEnv, pivotInA, "The pivotInA vector does not exist.", 0)
    NULL_CHK(pEnv, rotInA, "The rotInA matrix does not exist.", 0)
    btTransform frameInA;
    jmeBulletUtil::convert(pEnv, pivotInA, &frameInA.getOrigin());
    jmeBulletUtil::convert(pEnv, rotInA, &frameInA.getBasis());

    NULL_CHK(pEnv, pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHK(pEnv, rotInB, "The rotInB matrix does not exist.", 0)
    btTransform frameInB;
    jmeBulletUtil::convert(pEnv, pivotInB, &frameInB.getOrigin());
    jmeBulletUtil::convert(pEnv, rotInB, &frameInB.getBasis());

    RotateOrder rotateOrder = (RotateOrder) rotOrderIndex;
    btGeneric6DofSpring2Constraint *
            pConstraint = new btGeneric6DofSpring2Constraint(*pBodyA, *pBodyB,
            frameInA, frameInB, rotateOrder); //dance021

    return reinterpret_cast<jlong> (pConstraint);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    createSingleEnded
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;I)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_New6Dof_createSingleEnded
(JNIEnv *pEnv, jclass, jlong bodyIdB, jobject pivotInB, jobject rotInB,
        jint rotOrderIndex) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHK(pEnv, pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHK(pEnv, rotInB, "The rotInB matrix does not exist.", 0)
    btTransform frameInB;
    jmeBulletUtil::convert(pEnv, pivotInB, &frameInB.getOrigin());
    jmeBulletUtil::convert(pEnv, rotInB, &frameInB.getBasis());

    RotateOrder rotateOrder = (RotateOrder) rotOrderIndex;
    btGeneric6DofSpring2Constraint *
            pConstraint = new btGeneric6DofSpring2Constraint(*pBodyB, frameInB,
            rotateOrder); //dance021

    return reinterpret_cast<jlong> (pConstraint);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    enableSpring
 * Signature: (JIZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_enableSpring
(JNIEnv *pEnv, jclass, jlong constraintId, jint dofIndex,
        jboolean enableFlag) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    btAssert(dofIndex >= 0);
    btAssert(dofIndex < 6);

    pConstraint->enableSpring(dofIndex, enableFlag);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getAngles
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_getAngles
(JNIEnv *pEnv, jclass, jlong constraintId, jobject storeVector) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)

    pConstraint->calculateTransforms();

    btScalar x = pConstraint->getAngle(0);
    btScalar y = pConstraint->getAngle(1);
    btScalar z = pConstraint->getAngle(2);
    const btVector3& angles = btVector3(x, y, z);
    jmeBulletUtil::convert(pEnv, &angles, storeVector);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getAxis
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_getAxis
(JNIEnv *pEnv, jclass, jlong constraintId, jint axisIndex,
        jobject storeVector) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    btAssert(axisIndex >= 0);
    btAssert(axisIndex < 3);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)

    pConstraint->calculateTransforms();

    btVector3 axis = pConstraint->getAxis(axisIndex);
    jmeBulletUtil::convert(pEnv, &axis, storeVector);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getCalculatedOriginA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_getCalculatedOriginA
(JNIEnv *pEnv, jclass, jlong constraintId, jobject storeVector) {
    const btGeneric6DofSpring2Constraint * const pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btTransform& cta = pConstraint->getCalculatedTransformA();
    const btVector3 * const pOrigin = &cta.getOrigin();
    jmeBulletUtil::convert(pEnv, pOrigin, storeVector);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getCalculatedOriginB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_getCalculatedOriginB
(JNIEnv *pEnv, jclass, jlong constraintId, jobject storeVector) {
    const btGeneric6DofSpring2Constraint * const pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btTransform& ctb = pConstraint->getCalculatedTransformB();
    const btVector3 * const pOrigin = &ctb.getOrigin();
    jmeBulletUtil::convert(pEnv, pOrigin, storeVector);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getFrameOffsetA
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_getFrameOffsetA
(JNIEnv *pEnv, jclass, jlong constraintId, jobject storeTransform) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    NULL_CHK(pEnv, storeTransform, "The store transform does not exist.",)

    btTransform a = pConstraint->getFrameOffsetA();
    jmeBulletUtil::convert(pEnv, &a, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getFrameOffsetB
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_getFrameOffsetB
(JNIEnv *pEnv, jclass, jlong constraintId, jobject storeTransform) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    NULL_CHK(pEnv, storeTransform, "The store transform does not exist.",)

    btTransform b = pConstraint->getFrameOffsetB();
    jmeBulletUtil::convert(pEnv, &b, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getPivotOffset
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_getPivotOffset
(JNIEnv *pEnv, jclass, jlong constraintId, jobject storeVector) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    NULL_CHK(pEnv, storeVector, "The storeVector does not exist.",)

    pConstraint->calculateTransforms();

    btScalar x = pConstraint->getRelativePivotPosition(0);
    btScalar y = pConstraint->getRelativePivotPosition(1);
    btScalar z = pConstraint->getRelativePivotPosition(2);
    const btVector3& offset = btVector3(x, y, z);
    jmeBulletUtil::convert(pEnv, &offset, storeVector);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getRotationalMotor
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_New6Dof_getRotationalMotor
(JNIEnv *pEnv, jclass, jlong constraintId, jint axisIndex) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.", 0);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    btAssert(axisIndex >= 0);
    btAssert(axisIndex < 3);

    btRotationalLimitMotor2 *pMotor
            = pConstraint->getRotationalLimitMotor(axisIndex);
    return reinterpret_cast<jlong> (pMotor);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getRotationOrder
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_joints_New6Dof_getRotationOrder
(JNIEnv *pEnv, jclass, jlong constraintId) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.", 0);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);

    RotateOrder order = pConstraint->getRotationOrder();
    return (jint) order;
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    getTranslationalMotor
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_New6Dof_getTranslationalMotor
(JNIEnv *pEnv, jclass, jlong constraintId) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.", 0);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);

    btTranslationalLimitMotor2 *pMotor
            = pConstraint->getTranslationalLimitMotor();
    return reinterpret_cast<jlong> (pMotor);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    setAllEquilibriumPointsToCurrent
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setAllEquilibriumPointsToCurrent
(JNIEnv *pEnv, jclass, jlong constraintId) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);

    pConstraint->setEquilibriumPoint();
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    setDamping
 * Signature: (JIFZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setDamping
(JNIEnv *pEnv, jclass, jlong constraintId, jint dofIndex,
        jfloat damping, jboolean limitIfNeeded) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    btAssert(dofIndex >= 0);
    btAssert(dofIndex < 6);

    pConstraint->setDamping(dofIndex, damping, limitIfNeeded);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    setEquilibriumPoint
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setEquilibriumPoint
(JNIEnv *pEnv, jclass, jlong constraintId, jint dofIndex,
        jfloat value) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    btAssert(dofIndex >= 0);
    btAssert(dofIndex < 6);

    pConstraint->setEquilibriumPoint(dofIndex, value);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    setEquilibriumPointToCurrent
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setEquilibriumPointToCurrent
(JNIEnv *pEnv, jclass, jlong constraintId, jint dofIndex) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    btAssert(dofIndex >= 0);
    btAssert(dofIndex < 6);

    pConstraint->setEquilibriumPoint(dofIndex);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    setPivotInA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setPivotInA
(JNIEnv *pEnv, jclass, jlong constraintId, jobject pivotA) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    NULL_CHK(pEnv, pivotA, "The pivotA vector does not exist.",)

    btVector3 pivotInA;
    jmeBulletUtil::convert(pEnv, pivotA, &pivotInA);

    btTransform frameA = pConstraint->getFrameOffsetA();
    btTransform frameB = pConstraint->getFrameOffsetB();
    frameB.setOrigin(pivotInA);
    pConstraint->setFrames(frameA, frameB);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    setPivotInB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setPivotInB
(JNIEnv *pEnv, jclass, jlong constraintId, jobject pivotB) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    NULL_CHK(pEnv, pivotB, "The pivotB vector does not exist.",)

    btVector3 pivotInB;
    jmeBulletUtil::convert(pEnv, pivotB, &pivotInB);

    btTransform frameA = pConstraint->getFrameOffsetA();
    btTransform frameB = pConstraint->getFrameOffsetB();
    frameB.setOrigin(pivotInB);
    pConstraint->setFrames(frameA, frameB);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    setRotationOrder
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setRotationOrder
(JNIEnv *pEnv, jclass, jlong constraintId, jint rotOrder) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);

    RotateOrder order = (RotateOrder) rotOrder;
    pConstraint->setRotationOrder(order);
}

/*
 * Class:     com_jme3_bullet_joints_New6Dof
 * Method:    setStiffness
 * Signature: (JIFZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setStiffness
(JNIEnv *pEnv, jclass, jlong constraintId, jint dofIndex, jfloat stiffness,
        jboolean limitIfNeeded) {
    btGeneric6DofSpring2Constraint *pConstraint
            = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
            constraintId);
    NULL_CHK(pEnv, pConstraint,
            "The btGeneric6DofSpring2Constraint does not exist.",);
    btTypedConstraintType type = pConstraint->getConstraintType();
    btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
    btAssert(dofIndex >= 0);
    btAssert(dofIndex < 6);

    pConstraint->setStiffness(dofIndex, stiffness, limitIfNeeded);
}