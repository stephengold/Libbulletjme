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
#include "com_jme3_bullet_joints_ConeJoint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_ConeJoint_createJoint
(JNIEnv *pEnv, jclass, jlong bodyIdA, jlong bodyIdB, jobject pivotInA,
        jobject rotInA, jobject pivotInB, jobject rotInB) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    NULL_CHK(pEnv, pivotInA, "The pivotInA vector does not exist.", 0)
    NULL_CHK(pEnv, rotInA, "The rotInA matrix does not exist.", 0)
    btTransform rbAFrame;
    jmeBulletUtil::convert(pEnv, pivotInA, &rbAFrame.getOrigin());
    EXCEPTION_CHK(pEnv, 0);
    jmeBulletUtil::convert(pEnv, rotInA, &rbAFrame.getBasis());
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHK(pEnv, rotInB, "The rotInB matrix does not exist.", 0)
    btTransform rbBFrame;
    jmeBulletUtil::convert(pEnv, pivotInB, &rbBFrame.getOrigin());
    EXCEPTION_CHK(pEnv, 0);
    jmeBulletUtil::convert(pEnv, rotInB, &rbBFrame.getBasis());
    EXCEPTION_CHK(pEnv, 0);

    btConeTwistConstraint *
            pJoint = new btConeTwistConstraint(*pBodyA, *pBodyB, rbAFrame,
            rbBFrame); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_ConeJoint_createJoint1
(JNIEnv *pEnv, jclass, jlong bodyIdA, jobject pivotInA, jobject rotInA) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    ASSERT_CHK(pEnv, pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    NULL_CHK(pEnv, pivotInA, "The pivotInA vector does not exist.", 0)
    NULL_CHK(pEnv, rotInA, "The rotInA matrix does not exist.", 0)
    btTransform rbAFrame;
    jmeBulletUtil::convert(pEnv, pivotInA, &rbAFrame.getOrigin());
    EXCEPTION_CHK(pEnv, 0);
    jmeBulletUtil::convert(pEnv, rotInA, &rbAFrame.getBasis());
    EXCEPTION_CHK(pEnv, 0);

    btConeTwistConstraint *
            pJoint = new btConeTwistConstraint(*pBodyA, rbAFrame); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    getFrameOffsetA
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_getFrameOffsetA
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeTransform) {
    btConeTwistConstraint *pJoint
            = reinterpret_cast<btConeTwistConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btConeTwistConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE,);

    NULL_CHK(pEnv, storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetA();
    jmeBulletUtil::convert(pEnv, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    getFrameOffsetB
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_getFrameOffsetB
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeTransform) {
    btConeTwistConstraint *pJoint
            = reinterpret_cast<btConeTwistConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btConeTwistConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE,);

    NULL_CHK(pEnv, storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetB();
    jmeBulletUtil::convert(pEnv, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    setAngularOnly
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_setAngularOnly
(JNIEnv *pEnv, jclass, jlong jointId, jboolean angularOnly) {
    btConeTwistConstraint *pJoint
            = reinterpret_cast<btConeTwistConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btConeTwistConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE,);

    pJoint->setAngularOnly(angularOnly);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    setLimit
 * Signature: (JFFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_setLimit
(JNIEnv *pEnv, jclass, jlong jointId, jfloat swingSpan1, jfloat swingSpan2,
        jfloat twistSpan) {
    btConeTwistConstraint *pJoint
            = reinterpret_cast<btConeTwistConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btConeTwistConstraint does not exist.",)
    ASSERT_CHK(pEnv, pJoint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE,);

    //TODO: extended setLimit!
    pJoint->setLimit(swingSpan1, swingSpan2, twistSpan);
}
