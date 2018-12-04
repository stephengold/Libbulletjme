/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
 * Author: Dokthar
 */
#include "com_jme3_bullet_joints_SoftPhysicsJoint.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    getErrorReductionParameter
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_getErrorReductionParameter
    (JNIEnv *env, jobject object, jlong jointId) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }

        return joint->m_erp;
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    setErrorReductionParameter
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_setErrorReductionParameter
    (JNIEnv *env, jobject object, jlong jointId, jfloat erp) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }

        joint->m_erp = erp;
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    getConstraintForceMixing
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_getConstraintForceMixing
    (JNIEnv *env, jobject object, jlong jointId) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }

        return joint->m_cfm;
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    setConstraintForceMixing
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_setConstraintForceMixing
    (JNIEnv *env, jobject object, jlong jointId, jfloat cfm) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }

        joint->m_cfm = cfm;
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    getSplit
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_getSplit
    (JNIEnv *env, jobject object, jlong jointId) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }

        return joint->m_split;
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    setSplit
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_setSplit
    (JNIEnv *env, jobject object, jlong jointId, jfloat split) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }

        joint->m_split = split;
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    addConstraint
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_addConstraint
    (JNIEnv *env, jobject object, jlong jointId, jlong bodyId) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }

        btSoftBody* soft = reinterpret_cast<btSoftBody*> (bodyId);
        if (soft == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }

        soft->m_joints.push_back(joint);
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    removeConstraint
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_removeConstraint
    (JNIEnv *env, jobject object, jlong jointId, jlong bodyId) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }

        btSoftBody* soft = reinterpret_cast<btSoftBody*> (bodyId);
        if (soft == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }

        soft->m_joints.remove(joint);
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
     * Method:    finalizeNative
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_finalizeNative
    (JNIEnv *env, jobject object, jlong jointId) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }
        btAlignedFree(joint);
    }

#ifdef __cplusplus
}
#endif