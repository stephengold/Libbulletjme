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

/*
 * Author: Dokthar
 */
#include "com_jme3_bullet_joints_SoftLinearJoint.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_SoftLinearJoint
     * Method:    createJointSoftRigid
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;FFFLcom/jme3/math/Vector3f;)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SoftLinearJoint_createJointSoftRigid
    (JNIEnv *env, jobject object, jlong softIdA, jlong rigidIdB, jobject pivotA,
            jobject pivotB, jfloat erp, jfloat cfm, jfloat split,
            jobject position) {
        btSoftBody* softA = reinterpret_cast<btSoftBody*> (softIdA);
        NULL_CHECK(softA, "Soft body A does not exist.", 0)

        btRigidBody* rigidB = reinterpret_cast<btRigidBody*> (rigidIdB);
        NULL_CHECK(rigidB, "Rigid body B does not exist.", 0)

        NULL_CHECK(position, "The position vector does not exist.", 0)
        btVector3 pos;
        jmeBulletUtil::convert(env, position, &pos);

        NULL_CHECK(pivotA, "The pivotA vector does not exist.", 0)
        btVector3 pivA;
        jmeBulletUtil::convert(env, pivotA, &pivA);

        NULL_CHECK(pivotB, "The pivotB vector does not exist.", 0)
        btVector3 pivB;
        jmeBulletUtil::convert(env, pivotB, &pivB);

        btSoftBody::LJoint* ljoint = new(btAlignedAlloc(sizeof (btSoftBody::LJoint), 16)) btSoftBody::LJoint();

        ljoint->m_bodies[0] = softA->m_clusters[0];
        ljoint->m_bodies[1] = rigidB;

        ljoint->m_refs[0] = ljoint->m_bodies[0].xform().inverse() * pos;
        ljoint->m_refs[1] = ljoint->m_bodies[1].xform().inverse() * pos;

        ljoint->m_cfm = cfm;
        ljoint->m_erp = erp;
        ljoint->m_split = split;

        return reinterpret_cast<long> (ljoint);
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftLinearJoint
     * Method:    createJointSoftSoft
     * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;FFFLcom/jme3/math/Vector3f;)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SoftLinearJoint_createJointSoftSoft
    (JNIEnv *env, jobject object, jlong softIdA, jlong softIdB, jobject pivotA,
            jobject pivotB, jfloat erp, jfloat cfm, jfloat split,
            jobject position) {
        btSoftBody* softA = reinterpret_cast<btSoftBody*> (softIdA);
        NULL_CHECK(softA, "Soft body A does not exist.", 0)

        btSoftBody* softB = reinterpret_cast<btSoftBody*> (softIdB);
        NULL_CHECK(softB, "Soft body B does not exist.", 0)

        NULL_CHECK(position, "The position vector does not exist.", 0)
        btVector3 pos;
        jmeBulletUtil::convert(env, position, &pos);

        NULL_CHECK(pivotA, "The pivotA vector does not exist.", 0)
        btVector3 pivA;
        jmeBulletUtil::convert(env, pivotA, &pivA);

        NULL_CHECK(pivotB, "The pivotB vector does not exist.", 0)
        btVector3 pivB;
        jmeBulletUtil::convert(env, pivotB, &pivB);

        btSoftBody::LJoint* ljoint = new(btAlignedAlloc(sizeof (btSoftBody::LJoint), 16)) btSoftBody::LJoint();
        ljoint->m_bodies[0] = softA->m_clusters[0];
        ljoint->m_bodies[1] = softB->m_clusters[0];

        ljoint->m_refs[0] = ljoint->m_bodies[0].xform().inverse() * pos;
        ljoint->m_refs[1] = ljoint->m_bodies[1].xform().inverse() * pos;

        ljoint->m_cfm = cfm;
        ljoint->m_erp = erp;
        ljoint->m_split = split;

        return reinterpret_cast<long> (ljoint);
    }

    /*
     * Class:     com_jme3_bullet_joints_SoftLinearJoint
     * Method:    setPosition
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftLinearJoint_setPosition
    (JNIEnv *env, jobject object, jlong jointId, jobject position) {
        btSoftBody::LJoint* joint = reinterpret_cast<btSoftBody::LJoint*> (jointId);
        NULL_CHECK(joint, "The joint does not exist.",)

        NULL_CHECK(position, "The position vector does not exist.",)
        btVector3 pos;
        jmeBulletUtil::convert(env, position, &pos);

        joint->m_refs[0] = joint->m_bodies[0].xform().inverse() * pos;
        joint->m_refs[1] = joint->m_bodies[1].xform().inverse() * pos;
    }

#ifdef __cplusplus
}
#endif
