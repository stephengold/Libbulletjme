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

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    createJoint
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_HingeJoint_createJoint
    (JNIEnv *env, jobject object, jlong bodyIdA, jlong bodyIdB,
            jobject pivotInA, jobject axisInA, jobject pivotInB,
            jobject axisInB) {
        jmeClasses::initJavaClasses(env);

        btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
        NULL_CHECK(pBodyA, "Rigid body A does not exist.", 0)
        btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        btRigidBody *pBodyB = reinterpret_cast<btRigidBody *> (bodyIdB);
        NULL_CHECK(pBodyB, "Rigid body B does not exist.", 0)
        btAssert(pBodyB->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(pivotInA, "The pivotInA vector does not exist.", 0)
        btVector3 pivotA;
        jmeBulletUtil::convert(env, pivotInA, &pivotA);

        NULL_CHECK(pivotInB, "The pivotInB vector does not exist.", 0)
        btVector3 pivotB;
        jmeBulletUtil::convert(env, pivotInB, &pivotB);

        NULL_CHECK(axisInA, "The axisInA vector does not exist.", 0)
        btVector3 axisA;
        jmeBulletUtil::convert(env, axisInA, &axisA);

        NULL_CHECK(axisInB, "The axisInB vector does not exist.", 0)
        btVector3 axisB;
        jmeBulletUtil::convert(env, axisInB, &axisB);

        btHingeConstraint *pJoint = new btHingeConstraint(*pBodyA, *pBodyB,
                pivotA, pivotB, axisA, axisB);

        return reinterpret_cast<jlong> (pJoint);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    createJoint1
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Z)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_HingeJoint_createJoint1
    (JNIEnv *env, jobject object, jlong bodyIdA, jobject pivotInA,
            jobject axisInA, jboolean useReferenceFrameA) {
        jmeClasses::initJavaClasses(env);

        btRigidBody *pBodyA = reinterpret_cast<btRigidBody *> (bodyIdA);
        NULL_CHECK(pBodyA, "Rigid body A does not exist.", 0)
        btAssert(pBodyA->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(pivotInA, "The pivotInA vector does not exist.", 0)
        btVector3 pivot;
        jmeBulletUtil::convert(env, pivotInA, &pivot);

        NULL_CHECK(axisInA, "The axisInA vector does not exist.", 0)
        btVector3 axis;
        jmeBulletUtil::convert(env, axisInA, &axis);

        btHingeConstraint *pJoint = new btHingeConstraint(*pBodyA, pivot, axis,
                useReferenceFrameA);

        return reinterpret_cast<jlong> (pJoint);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    enableMotor
     * Signature: (JZFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_enableMotor
    (JNIEnv *env, jobject object, jlong jointId, jboolean enable, jfloat targetVelocity, jfloat maxMotorImpulse) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.",)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        pJoint->enableAngularMotor(enable, targetVelocity, maxMotorImpulse);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getEnableAngularMotor
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_HingeJoint_getEnableAngularMotor
    (JNIEnv *env, jobject object, jlong jointId) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.", JNI_FALSE)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        return pJoint->getEnableAngularMotor();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getFrameOffsetA
     * Signature: (JLcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_getFrameOffsetA
    (JNIEnv *env, jobject object, jlong jointId, jobject storeTransform) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.",)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        NULL_CHECK(storeTransform, "The storeTransform does not exist.",);

        const btTransform& transform = pJoint->getFrameOffsetA();
        jmeBulletUtil::convert(env, &transform, storeTransform);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getFrameOffsetB
     * Signature: (JLcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_getFrameOffsetB
    (JNIEnv *env, jobject object, jlong jointId, jobject storeTransform) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.",)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        NULL_CHECK(storeTransform, "The storeTransform does not exist.",);

        const btTransform& transform = pJoint->getFrameOffsetB();
        jmeBulletUtil::convert(env, &transform, storeTransform);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getHingeAngle
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getHingeAngle
    (JNIEnv *env, jobject object, jlong jointId) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.", 0)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        return pJoint->getHingeAngle();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getLowerLimit
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getLowerLimit
    (JNIEnv *env, jobject object, jlong jointId) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.", 0)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        return pJoint->getLowerLimit();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getMaxMotorImpulse
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getMaxMotorImpulse
    (JNIEnv *env, jobject object, jlong jointId) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.", 0)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        return pJoint->getMaxMotorImpulse();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getMotorTargetVelocity
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getMotorTargetVelocity
    (JNIEnv *env, jobject object, jlong jointId) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.", 0)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        return pJoint->getMotorTargetVelocity();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    getUpperLimit
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_HingeJoint_getUpperLimit
    (JNIEnv *env, jobject object, jlong jointId) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.", 0)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        return pJoint->getUpperLimit();
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    setAngularOnly
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_setAngularOnly
    (JNIEnv *env, jobject object, jlong jointId, jboolean angular) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.",)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        pJoint->setAngularOnly(angular);
    }

    /*
     * Class:     com_jme3_bullet_joints_HingeJoint
     * Method:    setLimit
     * Signature: (JFFFFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_HingeJoint_setLimit
    (JNIEnv *env, jobject object, jlong jointId, jfloat low, jfloat high,
            jfloat softness, jfloat biasFactor, jfloat relaxationFactor) {
        btHingeConstraint *pJoint
                = reinterpret_cast<btHingeConstraint *> (jointId);
        NULL_CHECK(pJoint, "The btHingeConstraint does not exist.",)
        btAssert(pJoint->getConstraintType() == HINGE_CONSTRAINT_TYPE);

        return pJoint->setLimit(low, high, softness, biasFactor,
                relaxationFactor);
    }

#ifdef __cplusplus
}
#endif
