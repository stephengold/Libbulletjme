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
#include "com_jme3_bullet_joints_SixDofJoint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofJoint_createJoint
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

    btGeneric6DofConstraint *
            pJoint = new btGeneric6DofConstraint(*pBodyA, *pBodyB, frameInA,
            frameInB, useLinearReferenceFrameA); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    createJoint1
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofJoint_createJoint1
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

    btGeneric6DofConstraint *
            pJoint = new btGeneric6DofConstraint(*pBodyB, frameInB,
            useLinearReferenceFrameB); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    getAngles
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getAngles
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeVector) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.",)
    NULL_CHK(pEnv, storeVector, "The storeVector does not exist.",)

    pJoint->calculateTransforms();
    btScalar x = pJoint->getAngle(0);
    btScalar y = pJoint->getAngle(1);
    btScalar z = pJoint->getAngle(2);
    const btVector3& angles = btVector3(x, y, z);
    jmeBulletUtil::convert(pEnv, &angles, storeVector);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    getFrameOffsetA
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getFrameOffsetA
(JNIEnv *pEnv, jclass, jlong jointId, jobject frameA) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.",)
    NULL_CHK(pEnv, frameA, "The frameA transform does not exist.",)

    btTransform a = pJoint->getFrameOffsetA();
    jmeBulletUtil::convert(pEnv, &a, frameA);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    getFrameOffsetB
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getFrameOffsetB
(JNIEnv *pEnv, jclass, jlong jointId, jobject frameB) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.",)
    NULL_CHK(pEnv, frameB, "The frameB transform does not exist.",)

    btTransform b = pJoint->getFrameOffsetB();
    jmeBulletUtil::convert(pEnv, &b, frameB);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    getPivotOffset
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getPivotOffset
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeVector) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.",)
    NULL_CHK(pEnv, storeVector, "The storeVector does not exist.",)

    pJoint->calculateTransforms();
    btScalar x = pJoint->getRelativePivotPosition(0);
    btScalar y = pJoint->getRelativePivotPosition(1);
    btScalar z = pJoint->getRelativePivotPosition(2);
    const btVector3& offset = btVector3(x, y, z);
    jmeBulletUtil::convert(pEnv, &offset, storeVector);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    getRotationalLimitMotor
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getRotationalLimitMotor
(JNIEnv *pEnv, jclass, jlong jointId, jint index) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.", 0)

    return reinterpret_cast<jlong> (pJoint->getRotationalLimitMotor(index));
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    getTranslationalLimitMotor
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getTranslationalLimitMotor
(JNIEnv *pEnv, jclass, jlong jointId) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.", 0)

    return reinterpret_cast<jlong> (pJoint->getTranslationalLimitMotor());
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    setAngularLowerLimit
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setAngularLowerLimit
(JNIEnv *pEnv, jclass, jlong jointId, jobject limitVector) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.",)
    NULL_CHK(pEnv, limitVector, "The limit vector does not exist.",)

    btVector3 vec;
    jmeBulletUtil::convert(pEnv, limitVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pJoint->setAngularLowerLimit(vec);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    setAngularUpperLimit
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setAngularUpperLimit
(JNIEnv *pEnv, jclass, jlong jointId, jobject limitVector) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.",)
    NULL_CHK(pEnv, limitVector, "The limit vector does not exist.",)

    btVector3 vec;
    jmeBulletUtil::convert(pEnv, limitVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pJoint->setAngularUpperLimit(vec);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    setLinearLowerLimit
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setLinearLowerLimit
(JNIEnv *pEnv, jclass, jlong jointId, jobject limitVector) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.",)

    NULL_CHK(pEnv, limitVector, "The limit vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, limitVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pJoint->setLinearLowerLimit(vec);
}

/*
 * Class:     com_jme3_bullet_joints_SixDofJoint
 * Method:    setLinearUpperLimit
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setLinearUpperLimit
(JNIEnv *pEnv, jclass, jlong jointId, jobject limitVector) {
    btGeneric6DofConstraint *pJoint
            = reinterpret_cast<btGeneric6DofConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGeneric6DofConstraint does not exist.",)

    NULL_CHK(pEnv, limitVector, "The limit vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, limitVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pJoint->setLinearUpperLimit(vec);
}
