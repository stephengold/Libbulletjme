/*
 * Copyright (c) 2019-2021 jMonkeyEngine
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
 * Author: Stephen Gold
 */
#include "com_jme3_bullet_joints_Anchor.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"

/*
 * Class:     com_jme3_bullet_joints_Anchor
 * Method:    createAnchor
 * Signature: (JIJLcom/jme3/math/Vector3f;ZF)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_joints_Anchor_createAnchor
(JNIEnv *pEnv, jclass, jlong softBodyId, jint nodeIndex, jlong rigidBodyId,
        jobject pivotVector, jboolean allowCollisions, jfloat influence) {
    btSoftBody *pSoftBody = reinterpret_cast<btSoftBody *> (softBodyId);
    NULL_CHK(pEnv, pSoftBody, "The btSoftBody does not exist.", 0)
    ASSERT_CHK(pEnv, pSoftBody->getInternalType()
            & btCollisionObject::CO_SOFT_BODY, 0);

    btRigidBody *pRigidBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHK(pEnv, pRigidBody, "The btRigidBody does not exist.", 0);
    ASSERT_CHK(pEnv, pRigidBody->getInternalType()
            & btCollisionObject::CO_RIGID_BODY, 0);

    ASSERT_CHK(pEnv, nodeIndex >= 0, 0);
    ASSERT_CHK(pEnv, nodeIndex < pSoftBody->m_nodes.size(), 0);

    btVector3 vec;
    jmeBulletUtil::convert(pEnv, pivotVector, &vec);
    EXCEPTION_CHK(pEnv, 0);
    pSoftBody->appendAnchor(nodeIndex, pRigidBody, vec, !allowCollisions,
            influence);

    int lastIndex = pSoftBody->m_anchors.size() - 1;
    btSoftBody::Anchor *pAnchor = &pSoftBody->m_anchors[lastIndex];

    return reinterpret_cast<jlong> (pAnchor);
}

/*
 * Class:     com_jme3_bullet_joints_Anchor
 * Method:    setInfluence
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Anchor_setInfluence
(JNIEnv *pEnv, jclass, jlong anchorId, jfloat influence) {
    btSoftBody::Anchor *pAnchor
            = reinterpret_cast<btSoftBody::Anchor *> (anchorId);
    NULL_CHK(pEnv, pAnchor, "The btSoftBody::Anchor does not exist.",)

    pAnchor->m_influence = (btScalar) influence;
}

/*
 * Class:     com_jme3_bullet_joints_Anchor
 * Method:    setPivotInB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Anchor_setPivotInB
(JNIEnv *pEnv, jclass, jlong anchorId, jobject locationVector) {
    btSoftBody::Anchor *pAnchor
            = reinterpret_cast<btSoftBody::Anchor *> (anchorId);
    NULL_CHK(pEnv, pAnchor, "The btSoftBody::Anchor does not exist.",)
    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)

    jmeBulletUtil::convert(pEnv, locationVector, &pAnchor->m_local);
}
