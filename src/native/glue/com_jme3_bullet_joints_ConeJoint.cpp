/*
 * Copyright (c) 2009-2019 jMonkeyEngine
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

/**
 * Author: Normen Hansen
 */
#include "com_jme3_bullet_joints_ConeJoint.h"
#include "jmeBulletUtil.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_ConeJoint
     * Method:    createJoint
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_ConeJoint_createJoint
    (JNIEnv * env, jobject object, jlong bodyIdA, jlong bodyIdB,
            jobject pivotInA, jobject rotInA, jobject pivotInB,
            jobject rotInB) {
        jmeClasses::initJavaClasses(env);

        btRigidBody* rbA = reinterpret_cast<btRigidBody*> (bodyIdA);
        if (rbA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "Rigid body A does not exist.");
            return 0L;
        }

        btRigidBody* rbB = reinterpret_cast<btRigidBody*> (bodyIdB);
        if (rbB == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "Rigid body B does not exist.");
            return 0L;
        }

        if (pivotInA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null pivotInA");
            return 0L;
        }
        if (rotInA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null rotInA");
            return 0L;
        }
        btTransform rbAFrame;
        jmeBulletUtil::convert(env, pivotInA, &rbAFrame.getOrigin());
        jmeBulletUtil::convert(env, rotInA, &rbAFrame.getBasis());

        if (pivotInB == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null pivotInB");
            return 0L;
        }
        if (rotInB == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null rotInB");
            return 0L;
        }
        btTransform rbBFrame;
        jmeBulletUtil::convert(env, pivotInB, &rbBFrame.getOrigin());
        jmeBulletUtil::convert(env, rotInB, &rbBFrame.getBasis());

        btConeTwistConstraint* joint
                = new btConeTwistConstraint(*rbA, *rbB, rbAFrame, rbBFrame);
        return reinterpret_cast<jlong> (joint);
    }

    /*
     * Class:     com_jme3_bullet_joints_ConeJoint
     * Method:    createJoint1
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_ConeJoint_createJoint1
    (JNIEnv * env, jobject object, jlong bodyIdA, jobject pivotInA,
            jobject rotInA) {
        jmeClasses::initJavaClasses(env);

        btRigidBody* rbA = reinterpret_cast<btRigidBody*> (bodyIdA);
        if (rbA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btRigidBody does not exist.");
            return 0L;
        }

        if (pivotInA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null pivotInA");
            return 0L;
        }
        if (rotInA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "null rotInA");
            return 0L;
        }
        btTransform rbAFrame;
        jmeBulletUtil::convert(env, pivotInA, &rbAFrame.getOrigin());
        jmeBulletUtil::convert(env, rotInA, &rbAFrame.getBasis());

        btConeTwistConstraint* joint
                = new btConeTwistConstraint(*rbA, rbAFrame);
        return reinterpret_cast<jlong> (joint);
    }

    /*
     * Class:     com_jme3_bullet_joints_ConeJoint
     * Method:    setAngularOnly
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_setAngularOnly
    (JNIEnv * env, jobject object, jlong jointId, jboolean angularOnly) {
        btConeTwistConstraint* joint = reinterpret_cast<btConeTwistConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btConeTwistConstraint does not exist.");
            return;
        }

        joint->setAngularOnly(angularOnly);
    }

    /*
     * Class:     com_jme3_bullet_joints_ConeJoint
     * Method:    setLimit
     * Signature: (JFFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_ConeJoint_setLimit
    (JNIEnv * env, jobject object, jlong jointId, jfloat swingSpan1, jfloat swingSpan2, jfloat twistSpan) {
        btConeTwistConstraint* joint = reinterpret_cast<btConeTwistConstraint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btConeTwistConstraint does not exist.");
            return;
        }

        //TODO: extended setLimit!
        joint->setLimit(swingSpan1, swingSpan2, twistSpan);
    }


#ifdef __cplusplus
}
#endif
