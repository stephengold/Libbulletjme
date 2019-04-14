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

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    createJoint
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofJoint_createJoint
    (JNIEnv * env, jobject object, jlong bodyIdA, jlong bodyIdB,
            jobject pivotInA, jobject rotInA, jobject pivotInB, jobject rotInB,
            jboolean useLinearReferenceFrameA) {
        jmeClasses::initJavaClasses(env);

        btRigidBody* rbA = reinterpret_cast<btRigidBody*> (bodyIdA);
        NULL_CHECK(rbA, "Rigid body A does not exist.", 0)

        btRigidBody* rbB = reinterpret_cast<btRigidBody*> (bodyIdB);
        NULL_CHECK(rbB, "Rigid body B does not exist.", 0)

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

        btGeneric6DofConstraint* joint = new btGeneric6DofConstraint(*rbA, *rbB,
                frameInA, frameInB, useLinearReferenceFrameA);
        NULL_CHECK(joint, "A btGeneric6DofConstraint was not created.", 0)

        return reinterpret_cast<jlong> (joint);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    createJoint1
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Z)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofJoint_createJoint1
    (JNIEnv * env, jobject object, jlong bodyIdB, jobject pivotInB,
            jobject rotInB, jboolean useLinearReferenceFrameB) {
        jmeClasses::initJavaClasses(env);

        btRigidBody* rbB = reinterpret_cast<btRigidBody*> (bodyIdB);
        NULL_CHECK(rbB, "Rigid body B does not exist.", 0)

        NULL_CHECK(pivotInB, "The pivotInB vector does not exist.", 0)
        NULL_CHECK(rotInB, "The rotInB matrix does not exist.", 0)
        btTransform frameInB;
        jmeBulletUtil::convert(env, pivotInB, &frameInB.getOrigin());
        jmeBulletUtil::convert(env, rotInB, &frameInB.getBasis());

        btGeneric6DofConstraint* joint = new btGeneric6DofConstraint(
                *rbB, frameInB, useLinearReferenceFrameB);
        NULL_CHECK(joint, "A btGeneric6DofConstraint was not created.", 0)

        return reinterpret_cast<jlong> (joint);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    getAngles
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getAngles
    (JNIEnv * env, jobject object, jlong jointId, jobject storeVector) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)
        NULL_CHECK(storeVector, "The storeVector does not exist.",)

        joint->calculateAngleInfo();
        btScalar x = joint->getAngle(0);
        btScalar y = joint->getAngle(1);
        btScalar z = joint->getAngle(2);
        const btVector3& angles = btVector3(x, y, z);
        jmeBulletUtil::convert(env, &angles, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    getFrameOffsetA
     * Signature: (JLcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getFrameOffsetA
    (JNIEnv * env, jobject object, jlong jointId, jobject frameA) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)
        NULL_CHECK(frameA, "The frameA transform does not exist.",)

        btTransform a = joint->getFrameOffsetA();
        jmeBulletUtil::convert(env, &a, frameA);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    getFrameOffsetB
     * Signature: (JLcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getFrameOffsetB
    (JNIEnv * env, jobject object, jlong jointId, jobject frameB) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)
        NULL_CHECK(frameB, "The frameB transform does not exist.",)

        btTransform b = joint->getFrameOffsetB();
        jmeBulletUtil::convert(env, &b, frameB);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    getPivotOffset
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getPivotOffset
    (JNIEnv * env, jobject object, jlong jointId, jobject storeVector) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)
        NULL_CHECK(storeVector, "The storeVector does not exist.",)

        btScalar x = joint->getRelativePivotPosition(0);
        btScalar y = joint->getRelativePivotPosition(1);
        btScalar z = joint->getRelativePivotPosition(2);
        const btVector3& offset = btVector3(x, y, z);
        jmeBulletUtil::convert(env, &offset, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    getRotationalLimitMotor
     * Signature: (JI)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getRotationalLimitMotor
    (JNIEnv * env, jobject object, jlong jointId, jint index) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.", 0)

        return reinterpret_cast<jlong> (joint->getRotationalLimitMotor(index));
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    getTranslationalLimitMotor
     * Signature: (J)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SixDofJoint_getTranslationalLimitMotor
    (JNIEnv * env, jobject object, jlong jointId) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.", 0)

        return reinterpret_cast<jlong> (joint->getTranslationalLimitMotor());
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    setAngularLowerLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setAngularLowerLimit
    (JNIEnv * env, jobject object, jlong jointId, jobject vector) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)
        NULL_CHECK(vector, "The limit vector does not exist.",)

        btVector3 vec;
        jmeBulletUtil::convert(env, vector, &vec);

        joint->setAngularLowerLimit(vec);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    setAngularUpperLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setAngularUpperLimit
    (JNIEnv * env, jobject object, jlong jointId, jobject vector) {
        btGeneric6DofConstraint* joint = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)
        NULL_CHECK(vector, "The limit vector does not exist.",)

        btVector3 vec;
        jmeBulletUtil::convert(env, vector, &vec);

        joint->setAngularUpperLimit(vec);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    setFrames
     * Signature: (JLcom/jme3/math/Transform;Lcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setFrames
    (JNIEnv * env, jobject object, jlong jointId, jobject frameA, jobject frameB) {
        btGeneric6DofConstraint* joint = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)

        NULL_CHECK(frameA, "The frameA transform does not exist.",)
        btTransform a = btTransform();
        jmeBulletUtil::convert(env, frameA, &a);

        NULL_CHECK(frameB, "The frameB transform does not exist.",)
        btTransform b = btTransform();
        jmeBulletUtil::convert(env, frameB, &b);

        joint->setFrames(a, b);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    setLinearLowerLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setLinearLowerLimit
    (JNIEnv * env, jobject object, jlong jointId, jobject vector) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)

        NULL_CHECK(vector, "The vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, vector, &vec);

        joint->setLinearLowerLimit(vec);
    }

    /*
     * Class:     com_jme3_bullet_joints_SixDofJoint
     * Method:    setLinearUpperLimit
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SixDofJoint_setLinearUpperLimit
    (JNIEnv * env, jobject object, jlong jointId, jobject vector) {
        btGeneric6DofConstraint* joint
                = reinterpret_cast<btGeneric6DofConstraint*> (jointId);
        NULL_CHECK(joint, "The btGeneric6DofConstraint does not exist.",)

        NULL_CHECK(vector, "The vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, vector, &vec);

        joint->setLinearUpperLimit(vec);
    }

#ifdef __cplusplus
}
#endif
