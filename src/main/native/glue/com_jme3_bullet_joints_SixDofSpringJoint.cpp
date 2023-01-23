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
#include "com_jme3_bullet_joints_SixDofSpringJoint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_createJoint
(JNIEnv *pEnv, jobject, jlong bodyIdA, jlong bodyIdB, jobject pivotInA,
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

    btGeneric6DofSpringConstraint *
            pJoint = new btGeneric6DofSpringConstraint(*pBodyA, *pBodyB,
            frameInA, frameInB, useLinearReferenceFrameA); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_createJoint1
(JNIEnv *pEnv, jobject, jlong bodyIdB, jobject pivotInB, jobject rotInB,
        jboolean useLinearReferenceFrameB) {
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

    btGeneric6DofSpringConstraint *
            pJoint = new btGeneric6DofSpringConstraint(*pBodyB, frameInB,
            useLinearReferenceFrameB); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    enableSpring
 * Signature: (JIZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_enableSpring
(JNIEnv *pEnv, jclass, jlong jointId, jint index, jboolean onOff) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, index >= 0,);
    ASSERT_CHK(pEnv, index < 6,);

    pJoint->enableSpring(index, onOff);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    getDamping
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_getDamping
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, index >= 0, 0);
    ASSERT_CHK(pEnv, index < 6, 0);

    btScalar result = pJoint->getDamping(index);
    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    getEquilibriumPoint
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_getEquilibriumPoint
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, index >= 0, 0);
    ASSERT_CHK(pEnv, index < 6, 0);

    btScalar result = pJoint->getEquilibriumPoint(index);
    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    getStiffness
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_getStiffness
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, index >= 0, 0);
    ASSERT_CHK(pEnv, index < 6, 0);

    btScalar result = pJoint->getStiffness(index);
    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    isSpringEnabled
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_isSpringEnabled
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.",
            JNI_FALSE);
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, index >= 0, 0);
    ASSERT_CHK(pEnv, index < 6, 0);

    bool result = pJoint->isSpringEnabled(index);
    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    setDamping
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_setDamping
(JNIEnv *pEnv, jclass, jlong jointId, jint index, jfloat damping) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, index >= 0,);
    ASSERT_CHK(pEnv, index < 6,);

    pJoint->setDamping(index, damping);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    setEquilibriumPoint
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_setEquilibriumPoint__J
(JNIEnv *pEnv, jclass, jlong jointId) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE,);

    pJoint->setEquilibriumPoint();
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    setEquilibriumPoint
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_setEquilibriumPoint__JI
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, index >= 0,);
    ASSERT_CHK(pEnv, index < 6,);

    pJoint->setEquilibriumPoint(index);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofSpringJoint
 * Method:    setStiffness
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofSpringJoint_setStiffness
(JNIEnv *pEnv, jclass, jlong jointId, jint index, jfloat stiffness) {
    btGeneric6DofSpringConstraint *pJoint
            = reinterpret_cast<btGeneric6DofSpringConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofSpringConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, index >= 0,);
    ASSERT_CHK(pEnv, index < 6,);

    pJoint->setStiffness(index, stiffness);
}
