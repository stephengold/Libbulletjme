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

/*
 * Class:     com_jme3_bullet_joints_SoftLinearJoint
 * Method:    createJointSoftRigid
 * Signature: (JIJFFFLcom/jme3/math/Vector3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SoftLinearJoint_createJointSoftRigid
(JNIEnv *pEnv, jclass, jlong softIdA, jint clusterIndexA, jlong rigidIdB,
        jfloat erp, jfloat cfm, jfloat split, jobject positionVector) {
    btSoftBody *pSoftA = reinterpret_cast<btSoftBody *> (softIdA);
    NULL_CHK(pEnv, pSoftA, "Soft body A does not exist.", 0)
    ASSERT_CHK(pEnv, pSoftA->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    btRigidBody *pRigidB = reinterpret_cast<btRigidBody *> (rigidIdB);
    NULL_CHK(pEnv, pRigidB, "Rigid body B does not exist.", 0)
    ASSERT_CHK(pEnv, pRigidB->getInternalType() & btCollisionObject::CO_RIGID_BODY, 0);

    NULL_CHK(pEnv, positionVector, "The position vector does not exist.", 0)
    btVector3 pos;
    jmeBulletUtil::convert(pEnv, positionVector, &pos);
    EXCEPTION_CHK(pEnv, 0);

    btSoftBody::LJoint::Specs specs;
    specs.cfm = cfm;
    specs.erp = erp;
    specs.split = split;
    specs.position = pos;

    btSoftBody::Cluster *pClusterA = pSoftA->m_clusters[clusterIndexA];
    pSoftA->appendLinearJoint(specs, pClusterA, pRigidB);

    int lastIndex = pSoftA->m_joints.size() - 1;
    btSoftBody::Joint *pJoint = pSoftA->m_joints[lastIndex];

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SoftLinearJoint
 * Method:    createJointSoftSoft
 * Signature: (JIJIFFFLcom/jme3/math/Vector3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_SoftLinearJoint_createJointSoftSoft
(JNIEnv *pEnv, jclass, jlong softIdA, jint clusterIndexA, jlong softIdB,
        jint clusterIndexB, jfloat erp, jfloat cfm, jfloat split,
        jobject positionVector) {
    btSoftBody *pSoftA = reinterpret_cast<btSoftBody *> (softIdA);
    NULL_CHK(pEnv, pSoftA, "Soft body A does not exist.", 0)
    ASSERT_CHK(pEnv, pSoftA->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    btSoftBody *pSoftB = reinterpret_cast<btSoftBody *> (softIdB);
    NULL_CHK(pEnv, pSoftB, "Soft body B does not exist.", 0)
    ASSERT_CHK(pEnv, pSoftB->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);

    NULL_CHK(pEnv, positionVector, "The position vector does not exist.", 0)
    btVector3 pos;
    jmeBulletUtil::convert(pEnv, positionVector, &pos);
    EXCEPTION_CHK(pEnv, 0);

    btSoftBody::LJoint::Specs specs;
    specs.cfm = cfm;
    specs.erp = erp;
    specs.split = split;
    specs.position = pos;

    btSoftBody::Cluster *pClusterA = pSoftA->m_clusters[clusterIndexA];
    btSoftBody::Cluster *pClusterB = pSoftB->m_clusters[clusterIndexB];
    pSoftA->appendLinearJoint(specs, pClusterA, pClusterB);

    int lastIndex = pSoftA->m_joints.size() - 1;
    btSoftBody::Joint *pJoint = pSoftA->m_joints[lastIndex];

    return reinterpret_cast<jlong> (pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SoftLinearJoint
 * Method:    setPosition
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftLinearJoint_setPosition
(JNIEnv *pEnv, jclass, jlong jointId, jobject positionVector) {
    btSoftBody::LJoint *pJoint
            = reinterpret_cast<btSoftBody::LJoint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The joint does not exist.",)

    NULL_CHK(pEnv, positionVector, "The position vector does not exist.",)
    btVector3 pos;
    jmeBulletUtil::convert(pEnv, positionVector, &pos);
    EXCEPTION_CHK(pEnv,);

    pJoint->m_refs[0] = pJoint->m_bodies[0].xform().inverse() * pos;
    pJoint->m_refs[1] = pJoint->m_bodies[1].xform().inverse() * pos;
}
