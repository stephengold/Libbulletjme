
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
#include "com_jme3_bullet_joints_SoftAngularJoint.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_SoftAngularJoint
     * Method:    setAxis
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftAngularJoint_setAxis
    (JNIEnv *env, jobject object, jlong jointId, jobject axis) {
        btSoftBody::Joint* joint = reinterpret_cast<btSoftBody::Joint*> (jointId);
        if (joint == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }

        btVector3 ax = btVector3();
        jmeBulletUtil::convert(env, axis, &ax);

        joint->m_refs[0] = joint->m_bodies[0].xform().inverse().getBasis() * ax;
        joint->m_refs[1] = joint->m_bodies[1].xform().inverse().getBasis() * ax;
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftAngularJoint
     * Method:    createJointSoftRigid
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;FFFLcom/jme3/math/Vector3f;)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SoftAngularJoint_createJointSoftRigid
    (JNIEnv *env, jobject object, jlong softIdA, jlong rigidIdB, jobject pivotA, jobject pivotB, jfloat erp, jfloat cfm, jfloat split, jobject axis) {
        btSoftBody* softA = reinterpret_cast<btSoftBody*> (softIdA);
        if (softA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }

        btRigidBody* rigidB = reinterpret_cast<btRigidBody*> (rigidIdB);
        if (rigidB == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }

        btVector3 ax = btVector3();
        jmeBulletUtil::convert(env, axis, &ax);

        btVector3 pivA = btVector3();
        jmeBulletUtil::convert(env, pivotA, &pivA);

        btVector3 pivB = btVector3();
        jmeBulletUtil::convert(env, pivotB, &pivB);

        btSoftBody::AJoint* ajoint = new(btAlignedAlloc(sizeof (btSoftBody::AJoint), 16)) btSoftBody::AJoint();
        ajoint->m_bodies[0] = softA->m_clusters[0];
        ajoint->m_bodies[1] = rigidB;

        ajoint->m_refs[0] = ajoint->m_bodies[0].xform().inverse().getBasis() * ax;
        ajoint->m_refs[1] = ajoint->m_bodies[1].xform().inverse().getBasis() * ax;

        ajoint->m_icontrol = btSoftBody::AJoint::IControl::Default();

        ajoint->m_cfm = cfm;
        ajoint->m_erp = erp;
        ajoint->m_split = split;

        return reinterpret_cast<long> (ajoint);
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftAngularJoint
     * Method:    createJointSoftSoft
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;FFFLcom/jme3/math/Vector3f;)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SoftAngularJoint_createJointSoftSoft
    (JNIEnv *env, jobject object, jlong softIdA, jlong softIdB, jobject pivotA, jobject pivotB, jfloat erp, jfloat cfm, jfloat split, jobject axis) {
        btSoftBody* softA = reinterpret_cast<btSoftBody*> (softIdA);
        if (softA == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }

        btSoftBody* softB = reinterpret_cast<btSoftBody*> (softIdB);
        if (softB == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }

        btVector3 ax = btVector3();
        jmeBulletUtil::convert(env, axis, &ax);

        btVector3 pivA = btVector3();
        jmeBulletUtil::convert(env, pivotA, &pivA);

        btVector3 pivB = btVector3();
        jmeBulletUtil::convert(env, pivotB, &pivB);

        btSoftBody::AJoint* ajoint = new(btAlignedAlloc(sizeof (btSoftBody::AJoint), 16)) btSoftBody::AJoint();
        ajoint->m_bodies[0] = softA->m_clusters[0];
        ajoint->m_bodies[1] = softB->m_clusters[0];

        ajoint->m_refs[0] = ajoint->m_bodies[0].xform().inverse().getBasis() * ax;
        ajoint->m_refs[1] = ajoint->m_bodies[1].xform().inverse().getBasis() * ax;

        ajoint->m_cfm = cfm;
        ajoint->m_erp = erp;
        ajoint->m_split = split;

        ajoint->m_icontrol = btSoftBody::AJoint::IControl::Default();

        return reinterpret_cast<long> (ajoint);
    }

#ifdef __cplusplus
}
#endif