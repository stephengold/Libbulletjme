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
(JNIEnv *env, jobject object, jlong bodyIdA, jlong bodyIdB,
        jobject pivotInA, jobject rotInA, jobject pivotInB,
        jobject rotInB) {
    jmeClasses::initJavaClasses(env);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHECK(pBodyA, "Rigid body A does not exist.", 0)
    btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHECK(pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHECK(pivotInA, "The pivotInA vector does not exist.", 0)
    NULL_CHECK(rotInA, "The rotInA matrix does not exist.", 0)
    btTransform rbAFrame;
    jmeBulletUtil::convert(env, pivotInA, &rbAFrame.getOrigin());
    jmeBulletUtil::convert(env, rotInA, &rbAFrame.getBasis());

    NULL_CHECK(pivotInB, "The pivotInB vector does not exist.", 0)
    NULL_CHECK(rotInB, "The rotInB matrix does not exist.", 0)
    btTransform rbBFrame;
    jmeBulletUtil::convert(env, pivotInB, &rbBFrame.getOrigin());
    jmeBulletUtil::convert(env, rotInB, &rbBFrame.getBasis());

    btConeTwistConstraint *pJoint = new btConeTwistConstraint(*pBodyA,
            *pBodyB, rbAFrame, rbBFrame);

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_ConeJoint_createJoint1
(JNIEnv *env, jobject object, jlong bodyIdA, jobject pivotInA,
        jobject rotInA) {
    jmeClasses::initJavaClasses(env);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHECK(pBodyA, "Rigid body A does not exist.", 0)
    btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHECK(pivotInA, "The pivotInA vector does not exist.", 0)
    NULL_CHECK(rotInA, "The rotInA matrix does not exist.", 0)
    btTransform rbAFrame;
    jmeBulletUtil::convert(env, pivotInA, &rbAFrame.getOrigin());
    jmeBulletUtil::convert(env, rotInA, &rbAFrame.getBasis());

    btConeTwistConstraint *pJoint
            = new btConeTwistConstraint(*pBodyA, rbAFrame);

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    getFrameOffsetA
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_getFrameOffsetA
(JNIEnv *env, jobject object, jlong jointId, jobject storeTransform) {
    btConeTwistConstraint *pJoint
            = reinterpret_cast<btConeTwistConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btConeTwistConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE);

    NULL_CHECK(storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetA();
    jmeBulletUtil::convert(env, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    getFrameOffsetB
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_getFrameOffsetB
(JNIEnv *env, jobject object, jlong jointId, jobject storeTransform) {
    btConeTwistConstraint *pJoint
            = reinterpret_cast<btConeTwistConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btConeTwistConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE);

    NULL_CHECK(storeTransform, "The storeTransform does not exist.",);

    const btTransform& transform = pJoint->getFrameOffsetB();
    jmeBulletUtil::convert(env, &transform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    setAngularOnly
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_setAngularOnly
(JNIEnv *env, jobject object, jlong jointId, jboolean angularOnly) {
    btConeTwistConstraint *pJoint
            = reinterpret_cast<btConeTwistConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btConeTwistConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE);

    pJoint->setAngularOnly(angularOnly);
}

/*
 * Class:     com_jme3_bullet_joints_ConeJoint
 * Method:    setLimit
 * Signature: (JFFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_setLimit
(JNIEnv *env, jobject object, jlong jointId, jfloat swingSpan1,
        jfloat swingSpan2, jfloat twistSpan) {
    btConeTwistConstraint *pJoint
            = reinterpret_cast<btConeTwistConstraint *> (jointId);
    NULL_CHECK(pJoint, "The btConeTwistConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == CONETWIST_CONSTRAINT_TYPE);

    //TODO: extended setLimit!
    pJoint->setLimit(swingSpan1, swingSpan2, twistSpan);
}
