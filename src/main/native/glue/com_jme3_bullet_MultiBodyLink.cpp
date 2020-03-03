/*
 * Copyright (c) 2020 jMonkeyEngine
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
#include "com_jme3_bullet_MultiBodyLink.h"
#include "btMultiBody.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    addConstraintForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_addConstraintForce
(JNIEnv *env, jobject, jlong linkId, jobject forceVector) {
    btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);

    NULL_CHECK(forceVector, "The force vector does not exist.",);
    btVector3 force;
    jmeBulletUtil::convert(env, forceVector, &force);

    pLink->m_appliedConstraintForce += force;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    addContraintTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_addContraintTorque
(JNIEnv *env, jobject, jlong linkId, jobject torqueVector) {
    btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);

    NULL_CHECK(torqueVector, "The torque vector does not exist.",);
    btVector3 torque;
    jmeBulletUtil::convert(env, torqueVector, &torque);

    pLink->m_appliedConstraintTorque += torque;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    addForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_addForce
(JNIEnv *env, jobject, jlong linkId, jobject forceVector) {
    btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);

    NULL_CHECK(forceVector, "The force vector does not exist.",);
    btVector3 force;
    jmeBulletUtil::convert(env, forceVector, &force);

    pLink->m_appliedForce += force;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    addJointTorque
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_addJointTorque
(JNIEnv *, jobject, jlong linkId, jint dofIndex, jfloat torque) {
    btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);

    int dof = (int) dofIndex;
    btAssert(dof >= 0);
    btAssert(dof < pLink->m_dofCount);
    btScalar amount = (btScalar) torque;

    pLink->m_jointTorque[dof] += amount;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    addTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_addTorque
(JNIEnv *env, jobject, jlong linkId, jobject torqueVector) {
    btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);

    NULL_CHECK(torqueVector, "The torque vector does not exist.",);
    btVector3 torque;
    jmeBulletUtil::convert(env, torqueVector, &torque);

    pLink->m_appliedTorque += torque;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getAppliedForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getAppliedForce
(JNIEnv *env, jobject, jlong linkId, jobject storeVector) {
    btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    const btVector3 *pForce = &pLink->m_appliedForce;
    jmeBulletUtil::convert(env, pForce, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getAppliedTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getAppliedTorque
(JNIEnv *env, jobject, jlong linkId, jobject storeVector) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    const btVector3 * const pTorque = &pLink->m_appliedTorque;
    jmeBulletUtil::convert(env, pTorque, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getAxisBottom
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getAxisBottom
(JNIEnv *env, jobject, jlong linkId, jint dofIndex, jobject storeVector) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    int dof = (int) dofIndex;
    const btVector3 * const pAxisBottom = &pLink->getAxisBottom(dof);
    jmeBulletUtil::convert(env, pAxisBottom, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getAxisTop
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getAxisTop
(JNIEnv *env, jobject, jlong linkId, jint dofIndex, jobject storeVector) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    int dof = (int) dofIndex;
    const btVector3 * const pAxisTop = &pLink->getAxisTop(dof);
    jmeBulletUtil::convert(env, pAxisTop, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getCollider
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_MultiBodyLink_getCollider
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const btMultiBodyLinkCollider *
            pCollider = pMultiBody->getLinkCollider((int) linkIndex);
    return reinterpret_cast<jlong> (pCollider);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getConstraintForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getConstraintForce
(JNIEnv *env, jobject, jlong linkId, jobject storeVector) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    const btVector3 *pForce = &pLink->m_appliedConstraintForce;
    jmeBulletUtil::convert(env, pForce, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getConstraintTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getConstraintTorque
(JNIEnv *env, jobject, jlong linkId, jobject storeVector) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    const btVector3 *pTorque = &pLink->m_appliedConstraintTorque;
    jmeBulletUtil::convert(env, pTorque, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getDofCount
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodyLink_getDofCount
(JNIEnv *, jobject, jlong linkId) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.", 0);

    const int dofCount = pLink->m_dofCount;
    return (jint) dofCount;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getDVector
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getDVector
(JNIEnv *env, jobject, jlong linkId, jobject storeVector) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    const btVector3 * const pDVector = &pLink->m_dVector;
    jmeBulletUtil::convert(env, pDVector, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getEVector
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getEVector
(JNIEnv *env, jobject, jlong linkId, jobject storeVector) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    const btVector3 * const pEVector = &pLink->m_eVector;
    jmeBulletUtil::convert(env, pEVector, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getFlags
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodyLink_getFlags
(JNIEnv *, jobject, jlong linkId) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.", 0);

    const int flags = pLink->m_flags;
    return (jint) flags;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getInertiaLocal
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getInertiaLocal
(JNIEnv *env, jobject, jlong linkId, jobject storeVector) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    const btVector3 * const pInertiaLocal = &pLink->m_inertiaLocal;
    jmeBulletUtil::convert(env, pInertiaLocal, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getJointPos
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBodyLink_getJointPos
(JNIEnv *, jobject, jlong linkId, jint dofIndex) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.", 0);

    int dof = (int) dofIndex;
    btAssert(dof >= 0);
    btAssert(dof < pLink->m_dofCount);

    btScalar jointPos = pLink->m_jointPos[dof];
    return (jfloat) jointPos;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getJointTorque
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBodyLink_getJointTorque
(JNIEnv *, jobject, jlong linkId, jint dofIndex) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.", 0);

    int dof = (int) dofIndex;
    btAssert(dof >= 0);
    btAssert(dof < pLink->m_dofCount);

    btScalar jointTorque = pLink->m_jointTorque[dof];
    return (jfloat) jointTorque;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getJointType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodyLink_getJointType
(JNIEnv *, jobject, jlong linkId) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.", 0);

    int jointType = pLink->m_jointType;
    return (jint) jointType;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getJointVel
 * Signature: (JII)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBodyLink_getJointVel
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex, jint dofIndex) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const int dof = (int) dofIndex;
    btAssert(dof >= 0);
    btAssert(dof < pMultiBody->getLink(linkIndex).m_dofCount);

    const btScalar * const pVelocities
            = pMultiBody->getJointVelMultiDof(linkIndex);
    const btScalar result = pVelocities[dof];

    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getLinkId
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_MultiBodyLink_getLinkId
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);
    const btMultibodyLink& link = pMultiBody->getLink(linkIndex);

    return reinterpret_cast<jlong> (&link);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getMass
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBodyLink_getMass
(JNIEnv *, jobject, jlong linkId) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.", 0);

    btScalar mass = pLink->m_mass;
    return (jfloat) mass;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getParent2LinkRotation
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getParent2LinkRotation
(JNIEnv *env, jobject, jlong linkId, jobject storeQuaternion) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",)
    NULL_CHECK(storeQuaternion, "The storeQuaternion does not exist.",);

    const btQuaternion *pRotation = &pLink->m_cachedRotParentToThis;
    jmeBulletUtil::convert(env, pRotation, storeQuaternion);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getParentIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodyLink_getParentIndex
(JNIEnv *, jobject, jlong linkId) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.", 0);

    int parentIndex = pLink->m_parent;
    return (jint) parentIndex;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getPosVarCount
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodyLink_getPosVarCount
(JNIEnv *, jobject, jlong linkId) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.", 0);

    const int posVarCount = pLink->m_posVarCount;
    return (jint) posVarCount;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getQ0Parent2LinkRotation
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getQ0Parent2LinkRotation
(JNIEnv *env, jobject, jlong linkId, jobject storeQuaternion) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeQuaternion, "The store quaternion does not exist.",);

    const btQuaternion * const pRotation = &pLink->m_zeroRotParentToThis;
    jmeBulletUtil::convert(env, pRotation, storeQuaternion);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    getWorldTransform
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_getWorldTransform
(JNIEnv *env, jobject, jlong linkId, jobject storeTransform) {
    const btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);
    NULL_CHECK(storeTransform, "The storeTransform does not exist.",);

    const btTransform * const pTransform = &pLink->m_cachedWorldTransform;
    jmeBulletUtil::convert(env, pTransform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    localFrameToWorld
 * Signature: (JILcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_localFrameToWorld
(JNIEnv *env, jobject, jlong multiBodyId, jint linkIndex,
        jobject rotationMatrix) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    NULL_CHECK(rotationMatrix, "The rotation matrix does not exist.",)
    btMatrix3x3 local_frame;
    jmeBulletUtil::convert(env, rotationMatrix, &local_frame);

    const btMatrix3x3 world_frame
            = pMultiBody->localFrameToWorld(linkIndex, local_frame);

    jmeBulletUtil::convert(env, &world_frame, rotationMatrix);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    localPosToWorld
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_localPosToWorld
(JNIEnv *env, jobject, jlong multiBodyId, jint linkIndex,
        jobject locationVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    NULL_CHECK(locationVector, "The location vector does not exist.",)
    btVector3 local_pos;
    jmeBulletUtil::convert(env, locationVector, &local_pos);

    const btVector3 world_pos
            = pMultiBody->localPosToWorld(linkIndex, local_pos);

    jmeBulletUtil::convert(env, &world_pos, locationVector);
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    setCollider
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_setCollider
(JNIEnv *, jobject, jlong linkId, jlong colliderId) {
    btMultibodyLink * const
            pLink = reinterpret_cast<btMultibodyLink *> (linkId);
    NULL_CHECK(pLink, "The link does not exist.",);

    btMultiBodyLinkCollider * const
            pCollider = reinterpret_cast<btMultiBodyLinkCollider *> (colliderId);
    NULL_CHECK(pCollider, "The collider does not exist.",);

    pLink->m_collider = pCollider;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    setJointPos
 * Signature: (JIIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_setJointPos
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex, jint dofIndex,
        jfloat position) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);
    btMultibodyLink& link = pMultiBody->getLink(linkIndex);

    const int dof = (int) dofIndex;
    btAssert(dof >= 0);
    btAssert(dof < link.m_dofCount);

    link.m_jointPos[dof] = (btScalar) position;
    link.updateCacheMultiDof();
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    setJointVel
 * Signature: (JIIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_setJointVel
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex, jint dofIndex,
        jfloat velocity) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    const int dof = (int) dofIndex;
    btAssert(dof >= 0);
    btAssert(dof < pMultiBody->getLink(linkIndex).m_dofCount);

    btScalar * const pVelocities = pMultiBody->getJointVelMultiDof(linkIndex);
    pVelocities[dof] = (btScalar) velocity;
}

/*
 * Class:     com_jme3_bullet_MultiBodyLink
 * Method:    worldPosToLocal
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodyLink_worldPosToLocal
(JNIEnv *env, jobject, jlong multiBodyId, jint linkIndex,
        jobject locationVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    NULL_CHECK(locationVector, "The location vector does not exist.",)
    btVector3 world_pos;
    jmeBulletUtil::convert(env, locationVector, &world_pos);

    const btVector3 local_pos
            = pMultiBody->worldPosToLocal(linkIndex, world_pos);

    jmeBulletUtil::convert(env, &local_pos, locationVector);
}
