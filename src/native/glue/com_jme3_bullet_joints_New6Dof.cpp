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
#include "com_jme3_bullet_joints_New6Dof.h"
#include "jmeBulletUtil.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_New6Dof
     * Method:    createDoubleEnded
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;I)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_New6Dof_createDoubleEnded
    (JNIEnv *env, jobject object, jlong bodyIdA, jlong bodyIdB,
            jobject pivotInA, jobject rotInA, jobject pivotInB, jobject rotInB,
            jint rotOrderIndex) {
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

        RotateOrder rotateOrder = (RotateOrder) rotOrderIndex;
        btGeneric6DofSpring2Constraint *pConstraint
                = new btGeneric6DofSpring2Constraint(*pBodyA, *pBodyB,
                frameInA, frameInB, rotateOrder);

        return reinterpret_cast<jlong> (pConstraint);
    }

    /*
     * Class:     com_jme3_bullet_joints_New6Dof
     * Method:    createSingleEnded
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;I)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_New6Dof_createSingleEnded
    (JNIEnv *env, jobject object, jlong bodyIdB, jobject pivotInB,
            jobject rotInB, jint rotOrderIndex) {
        jmeClasses::initJavaClasses(env);

        btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
        NULL_CHECK(pBodyB, "Rigid body B does not exist.", 0)
        btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(pivotInB, "The pivotInB vector does not exist.", 0)
        NULL_CHECK(rotInB, "The rotInB matrix does not exist.", 0)
        btTransform frameInB;
        jmeBulletUtil::convert(env, pivotInB, &frameInB.getOrigin());
        jmeBulletUtil::convert(env, rotInB, &frameInB.getBasis());

        RotateOrder rotateOrder = (RotateOrder) rotOrderIndex;
        btGeneric6DofSpring2Constraint *pConstraint
                = new btGeneric6DofSpring2Constraint(*pBodyB, frameInB,
                rotateOrder);

        return reinterpret_cast<jlong> (pConstraint);
    }

    /*
     * Class:     com_jme3_bullet_joints_New6Dof
     * Method:    enableSpring
     * Signature: (JIZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_enableSpring
    (JNIEnv *env, jobject object, jlong constraintId, jint dofIndex,
            jboolean enableFlag) {
        btGeneric6DofSpring2Constraint *pConstraint
                = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
                constraintId);
        NULL_CHECK(pConstraint,
                "The btGeneric6DofSpring2Constraint does not exist.",);
        btTypedConstraintType type = pConstraint->getConstraintType();
        btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
        btAssert(dofIndex >= 0);
        btAssert(dofIndex < 6);

        pConstraint->enableSpring(dofIndex, enableFlag);
    }

    /*
     * Class:     com_jme3_bullet_joints_New6Dof
     * Method:    getRotationalMotor
     * Signature: (JI)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_New6Dof_getRotationalMotor
    (JNIEnv *env, jobject object, jlong constraintId, jint axisIndex) {
        btGeneric6DofSpring2Constraint *pConstraint
                = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
                constraintId);
        NULL_CHECK(pConstraint,
                "The btGeneric6DofSpring2Constraint does not exist.", 0);
        btTypedConstraintType type = pConstraint->getConstraintType();
        btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
        btAssert(axisIndex >= 0);
        btAssert(axisIndex < 3);

        btRotationalLimitMotor2* pMotor
                = pConstraint->getRotationalLimitMotor(axisIndex);
        return reinterpret_cast<jlong> (pMotor);
    }

    /*
     * Class:     com_jme3_bullet_joints_New6Dof
     * Method:    getTranslationalMotor
     * Signature: (J)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_New6Dof_getTranslationalMotor
    (JNIEnv *env, jobject object, jlong constraintId) {
        btGeneric6DofSpring2Constraint *pConstraint
                = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
                constraintId);
        NULL_CHECK(pConstraint,
                "The btGeneric6DofSpring2Constraint does not exist.", 0);
        btTypedConstraintType type = pConstraint->getConstraintType();
        btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);

        btTranslationalLimitMotor2* pMotor
                = pConstraint->getTranslationalLimitMotor();
        return reinterpret_cast<jlong> (pMotor);
    }

    /*
     * Class:     com_jme3_bullet_joints_New6Dof
     * Method:    setAllEquilibriumPointsToCurrent
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setAllEquilibriumPointsToCurrent
    (JNIEnv *env, jobject object, jlong constraintId) {
        btGeneric6DofSpring2Constraint *pConstraint
                = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
                constraintId);
        NULL_CHECK(pConstraint,
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
    (JNIEnv *env, jobject object, jlong constraintId, jint dofIndex,
            jfloat damping, jboolean limitIfNeeded) {
        btGeneric6DofSpring2Constraint *pConstraint
                = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
                constraintId);
        NULL_CHECK(pConstraint,
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
    (JNIEnv *env, jobject object, jlong constraintId, jint dofIndex,
            jfloat value) {
        btGeneric6DofSpring2Constraint *pConstraint
                = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
                constraintId);
        NULL_CHECK(pConstraint,
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
    (JNIEnv *env, jobject object, jlong constraintId, jint dofIndex) {
        btGeneric6DofSpring2Constraint *pConstraint
                = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
                constraintId);
        NULL_CHECK(pConstraint,
                "The btGeneric6DofSpring2Constraint does not exist.",);
        btTypedConstraintType type = pConstraint->getConstraintType();
        btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
        btAssert(dofIndex >= 0);
        btAssert(dofIndex < 6);

        pConstraint->setEquilibriumPoint(dofIndex);
    }

    /*
     * Class:     com_jme3_bullet_joints_New6Dof
     * Method:    setStiffness
     * Signature: (JIFZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_New6Dof_setStiffness
    (JNIEnv *env, jobject object, jlong constraintId, jint dofIndex,
            jfloat stiffness, jboolean limitIfNeeded) {
        btGeneric6DofSpring2Constraint *pConstraint
                = reinterpret_cast<btGeneric6DofSpring2Constraint *> (
                constraintId);
        NULL_CHECK(pConstraint,
                "The btGeneric6DofSpring2Constraint does not exist.",);
        btTypedConstraintType type = pConstraint->getConstraintType();
        btAssert(type == D6_SPRING_2_CONSTRAINT_TYPE);
        btAssert(dofIndex >= 0);
        btAssert(dofIndex < 6);

        pConstraint->setStiffness(dofIndex, stiffness, limitIfNeeded);
    }

#ifdef __cplusplus
}
#endif
