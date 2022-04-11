/*
 * Copyright (c) 2022 jMonkeyEngine
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

#include "com_jme3_bullet_joints_GearJoint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    createJoint
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_GearJoint_createJoint
(JNIEnv *pEnv, jclass, jlong bodyIdA, jlong bodyIdB, jobject axisInA,
    jobject axisInB, jfloat ratio) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
    NULL_CHK(pEnv, pBodyA, "Rigid body A does not exist.", 0)
    btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
    NULL_CHK(pEnv, pBodyB, "Rigid body B does not exist.", 0)
    btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    NULL_CHK(pEnv, axisInA, "The axisInA vector does not exist.", 0)
    btVector3 axisA;
    jmeBulletUtil::convert(pEnv, axisInA, &axisA);

    NULL_CHK(pEnv, axisInB, "The axisInB vector does not exist.", 0)
    btVector3 axisB;
    jmeBulletUtil::convert(pEnv, axisInB, &axisB);

    btGearConstraint *pJoint
        = new btGearConstraint(*pBodyA, *pBodyB, axisA, axisB, ratio); //dance021

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    getAxisA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_getAxisA
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeResult) {
    const btGearConstraint * const pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    NULL_CHK(pEnv, storeResult, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pJoint->getAxisA(), storeResult);
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    getAxisB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_getAxisB
(JNIEnv *pEnv, jclass, jlong jointId, jobject storeResult) {
    const btGearConstraint * const pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    NULL_CHK(pEnv, storeResult, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pJoint->getAxisB(), storeResult);
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    getRatio
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_GearJoint_getRatio
(JNIEnv *pEnv, jclass, jlong jointId) {
    const btGearConstraint * const pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.", 0)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    btScalar result = pJoint->getRatio();
    return result;
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    setAxisA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_setAxisA
(JNIEnv *pEnv, jclass, jlong jointId, jobject axisA) {
    btGearConstraint * const pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    NULL_CHK(pEnv, axisA, "The axisA vector does not exist.",)
    btVector3 axisInA;
    jmeBulletUtil::convert(pEnv, axisA, &axisInA);

    pJoint->setAxisA(axisInA);
}

/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    setAxisB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_setAxisB
(JNIEnv *pEnv, jclass, jlong jointId, jobject axisB) {
    btGearConstraint * const pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    NULL_CHK(pEnv, axisB, "The axisB vector does not exist.",)
    btVector3 axisInB;
    jmeBulletUtil::convert(pEnv, axisB, &axisInB);

    pJoint->setAxisB(axisInB);
}
/*
 * Class:     com_jme3_bullet_joints_GearJoint
 * Method:    setRatio
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_GearJoint_setRatio
(JNIEnv *pEnv, jclass, jlong jointId, jfloat ratio) {
    btGearConstraint * const pJoint
        = reinterpret_cast<btGearConstraint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The btGearConstraint does not exist.",)
    btAssert(pJoint->getConstraintType() == GEAR_CONSTRAINT_TYPE);

    btScalar gearRatio = btScalar(ratio);
    pJoint->setRatio(gearRatio);
}