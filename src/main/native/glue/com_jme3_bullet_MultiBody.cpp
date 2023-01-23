/*
 * Copyright (c) 2020-2023 jMonkeyEngine
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
#include "com_jme3_bullet_MultiBody.h"
#include "btMultiBody.h"
#include "jmeBulletUtil.h"
#include "jmeUserInfo.h"

/*
 * Author: Stephen Gold
 */

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    addBaseForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_addBaseForce
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject forceVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    NULL_CHK(pEnv, forceVector, "The force vector does not exist.",)
    btVector3 force;
    jmeBulletUtil::convert(pEnv, forceVector, &force);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->addBaseForce(force);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    addBaseTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_addBaseTorque
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject torqueVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    NULL_CHK(pEnv, torqueVector, "The torque vector does not exist.",)
    btVector3 torque;
    jmeBulletUtil::convert(pEnv, torqueVector, &torque);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->addBaseTorque(torque);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    clearConstraintForces
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_clearConstraintForces
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    pMultiBody->clearConstraintForces();
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    clearForcesAndTorques
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_clearForcesAndTorques
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    pMultiBody->clearForcesAndTorques();
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    clearVelocities
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_clearVelocities
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    pMultiBody->clearVelocities();
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    create
 * Signature: (IFLcom/jme3/math/Vector3f;ZZ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_MultiBody_create
(JNIEnv *pEnv, jobject object, jint numLinks, jfloat baseMass,
        jobject inertiaVector, jboolean fixedBase, jboolean canSleep) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, inertiaVector, "The inertia vector does not exist.", 0)
    btVector3 inertia;
    jmeBulletUtil::convert(pEnv, inertiaVector, &inertia);
    EXCEPTION_CHK(pEnv, 0);

    btMultiBody * const
            pMultiBody = new btMultiBody(numLinks, baseMass, inertia, fixedBase,
            canSleep); //dance004

    jmeUserPointer const pUser = new jmeUserInfo(); //dance005
    pUser->m_javaRef = pEnv->NewWeakGlobalRef(object); //dance039
    EXCEPTION_CHK(pEnv, 0);
    pUser->m_group = 0x1;
    pUser->m_groups = 0x1;
    pUser->m_jmeSpace = NULL;
    pMultiBody->setUserPointer(pUser);

    return reinterpret_cast<jlong> (pMultiBody);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    finalizeMultiDof
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_finalizeMultiDof
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    pMultiBody->finalizeMultiDof();
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_finalizeNative
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);

    if (pMultiBody) {
        jmeUserPointer const
                pUser = (jmeUserPointer) pMultiBody->getUserPointer();
        if (pUser) {
            if (pUser->m_javaRef) {
                pEnv->DeleteWeakGlobalRef(pUser->m_javaRef); //dance039
                EXCEPTION_CHK(pEnv,);
            }
            delete pUser; //dance005
        }
        delete pMultiBody; //dance004
    }
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getAngularDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getAngularDamping
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    btScalar angularDamping = pMultiBody->getAngularDamping();
    return (jfloat) angularDamping;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseCollider
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_MultiBody_getBaseCollider
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    const btMultiBodyLinkCollider * pCollider = pMultiBody->getBaseCollider();
    return reinterpret_cast<jlong> (pCollider);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseForce
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& baseForce = pMultiBody->getBaseForce();
    jmeBulletUtil::convert(pEnv, &baseForce, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseInertia
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseInertia
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& baseInertia = pMultiBody->getBaseInertia();
    jmeBulletUtil::convert(pEnv, &baseInertia, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseMass
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getBaseMass
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    const btScalar baseMass = pMultiBody->getBaseMass();
    return (jfloat) baseMass;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseOmega
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseOmega
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& baseOmega = pMultiBody->getBaseOmega();
    jmeBulletUtil::convert(pEnv, &baseOmega, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBasePos
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBasePos
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& basePos = pMultiBody->getBasePos();
    jmeBulletUtil::convert(pEnv, &basePos, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseTorque
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& baseTorque = pMultiBody->getBaseTorque();
    jmeBulletUtil::convert(pEnv, &baseTorque, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseVel
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseVel
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& baseVel = pMultiBody->getBaseVel();
    jmeBulletUtil::convert(pEnv, &baseVel, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseWorldTransform
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseWorldTransform
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject storeTransform) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);
    NULL_CHK(pEnv, storeTransform, "The storeTransform does not exist.",);

    const btTransform& baseWorldTransform = pMultiBody->getBaseWorldTransform();
    jmeBulletUtil::convert(pEnv, &baseWorldTransform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getCanSleep
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_getCanSleep
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", JNI_FALSE);

    bool canSleep = pMultiBody->getCanSleep();
    return (jboolean) canSleep;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getCanWakeup
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_getCanWakeup
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", JNI_FALSE);

    bool canWakeup = pMultiBody->getCanWakeup();
    return (jboolean) canWakeup;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getCollideWithGroups
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBody_getCollideWithGroups
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    jmeUserPointer const pUser = (jmeUserPointer) pMultiBody->getUserPointer();
    jint groups = pUser->m_groups;
    return groups;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getCollisionGroup
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBody_getCollisionGroup
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    jmeUserPointer const pUser = (jmeUserPointer) pMultiBody->getUserPointer();
    jint group = pUser->m_group;
    return group;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getLinearDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getLinearDamping
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    btScalar linearDamping = pMultiBody->getLinearDamping();
    return (jfloat) linearDamping;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getMaxAppliedImpulse
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getMaxAppliedImpulse
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    btScalar maxAppliedImpulse = pMultiBody->getMaxAppliedImpulse();
    return (jfloat) maxAppliedImpulse;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getMaxCoordinateVelocity
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getMaxCoordinateVelocity
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    btScalar maxCoordinateVelocity = pMultiBody->getMaxCoordinateVelocity();
    return (jfloat) maxCoordinateVelocity;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getNumDofs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBody_getNumDofs
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    int numDofs = pMultiBody->getNumDofs();
    return (jint) numDofs;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getNumLinks
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBody_getNumLinks
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    int numLinks = pMultiBody->getNumLinks();
    return (jint) numLinks;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getNumPosVars
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBody_getNumPosVars
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    int numPosVars = pMultiBody->getNumPosVars();
    return (jint) numPosVars;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getSpace
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_MultiBody_getSpace
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", 0);

    jmeUserPointer const pUser = (jmeUserPointer) pMultiBody->getUserPointer();
    jmeCollisionSpace *pSpace = pUser->m_jmeSpace;
    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getUseGyroTerm
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_getUseGyroTerm
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", JNI_FALSE);

    bool useGyroTerm = pMultiBody->getUseGyroTerm();
    return (jboolean) useGyroTerm;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getWorldToBaseRot
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getWorldToBaseRot
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject storeQuaternion) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);
    NULL_CHK(pEnv, storeQuaternion, "The storeQuaternion does not exist.",);

    btQuaternion worldToBaseRot = pMultiBody->getWorldToBaseRot();
    jmeBulletUtil::convert(pEnv, &worldToBaseRot, storeQuaternion);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    hasFixedBase
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_hasFixedBase
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", JNI_FALSE);

    bool hasFixedBase = pMultiBody->hasFixedBase();
    return (jboolean) hasFixedBase;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    isUsingGlobalVelocities
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_isUsingGlobalVelocities
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", JNI_FALSE);

    bool isUsingGlobalVelocities = pMultiBody->isUsingGlobalVelocities();
    return (jboolean) isUsingGlobalVelocities;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    isUsingRK4Integration
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_isUsingRK4Integration
(JNIEnv *pEnv, jclass, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.", JNI_FALSE);

    bool isUsingRK4Integration = pMultiBody->isUsingRK4Integration();
    return (jboolean) isUsingRK4Integration;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBaseCollider
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBaseCollider
(JNIEnv *pEnv, jclass, jlong multiBodyId, jlong colliderId) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    btMultiBodyLinkCollider * const
            pCollider = reinterpret_cast<btMultiBodyLinkCollider *> (colliderId);
    NULL_CHK(pEnv, pCollider, "The collider does not exist.",);

    pMultiBody->setBaseCollider(pCollider);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBaseOmega
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBaseOmega
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject angularVelocityVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    NULL_CHK(pEnv, angularVelocityVector,
            "The angular velocity vector does not exist.",);
    btVector3 omega;
    jmeBulletUtil::convert(pEnv, angularVelocityVector, &omega);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setBaseOmega(omega);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBasePos
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBasePos
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject positionVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    NULL_CHK(pEnv, positionVector, "The position vector does not exist.",);
    btVector3 pos;
    jmeBulletUtil::convert(pEnv, positionVector, &pos);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setBasePos(pos);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBaseVel
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBaseVel
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject velocityVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",);
    btVector3 vel;
    jmeBulletUtil::convert(pEnv, velocityVector, &vel);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setBaseVel(vel);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBaseWorldTransform
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBaseWorldTransform
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject transform) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    NULL_CHK(pEnv, transform, "The transform does not exist.",);
    btTransform tr;
    btVector3 scale;
    jmeBulletUtil::convert(pEnv, transform, &tr, &scale);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setBaseWorldTransform(tr);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setCollideWithGroups
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setCollideWithGroups
(JNIEnv *pEnv, jclass, jlong multiBodyId, jint groups) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    jmeUserPointer const pUser = (jmeUserPointer) pMultiBody->getUserPointer();
    pUser->m_groups = groups;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setCollisionGroup
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setCollisionGroup
(JNIEnv *pEnv, jclass, jlong multiBodyId, jint group) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    jmeUserPointer const pUser = (jmeUserPointer) pMultiBody->getUserPointer();
    pUser->m_group = group;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setupFixed
 * Signature: (JIFLcom/jme3/math/Vector3f;ILcom/jme3/math/Quaternion;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setupFixed
(JNIEnv *pEnv, jclass, jlong multiBodyId, jint linkIndex, jfloat mass,
        jobject inertiaVector, jint parentLinkIndex,
        jobject parent2LinkQuaternion, jobject parent2PivotVector,
        jobject pivot2LinkVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    const int i = (int) linkIndex;
    ASSERT_CHK(pEnv, i >= 0,);

    btScalar m = (btScalar) mass;
    ASSERT_CHK(pEnv, mass > 0,);

    NULL_CHK(pEnv, inertiaVector, "The inertia vector does not exist.",);
    btVector3 inertia;
    jmeBulletUtil::convert(pEnv, inertiaVector, &inertia);
    EXCEPTION_CHK(pEnv,);

    int parent = (int) parentLinkIndex;
    ASSERT_CHK(pEnv, parent >= -1,);

    NULL_CHK(pEnv, parent2LinkQuaternion,
            "The parent2Link quaternion does not exist.",);
    btQuaternion rotParentToThis;
    jmeBulletUtil::convert(pEnv, parent2LinkQuaternion, &rotParentToThis);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, parent2PivotVector, "The parent2pivot vector does not exist.",);
    btVector3 parentComToThisPivotOffset;
    jmeBulletUtil::convert(pEnv, parent2PivotVector, &parentComToThisPivotOffset);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, pivot2LinkVector, "The pivot2link vector does not exist.",);
    btVector3 thisPivotToThisComOffset;
    jmeBulletUtil::convert(pEnv, pivot2LinkVector, &thisPivotToThisComOffset);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setupFixed(i, m, inertia, parent, rotParentToThis,
            parentComToThisPivotOffset, thisPivotToThisComOffset);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setupPlanar
 * Signature: (JIFLcom/jme3/math/Vector3f;ILcom/jme3/math/Quaternion;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setupPlanar
(JNIEnv *pEnv, jclass, jlong multiBodyId, jint linkIndex, jfloat mass,
        jobject inertiaVector, jint parentLinkIndex,
        jobject parent2LinkQuaternion, jobject axisVector,
        jobject parent2LinkVector, jboolean disableParentCollision) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    const int i = (int) linkIndex;
    ASSERT_CHK(pEnv, i >= 0,);

    btScalar m = (btScalar) mass;
    ASSERT_CHK(pEnv, mass > 0,);

    NULL_CHK(pEnv, inertiaVector, "The inertia vector does not exist.",);
    btVector3 inertia;
    jmeBulletUtil::convert(pEnv, inertiaVector, &inertia);
    EXCEPTION_CHK(pEnv,);

    int parent = (int) parentLinkIndex;
    ASSERT_CHK(pEnv, parent >= -1,);

    NULL_CHK(pEnv, parent2LinkQuaternion,
            "The parent2Link quaternion does not exist.",);
    btQuaternion rotParentToThis;
    jmeBulletUtil::convert(pEnv, parent2LinkQuaternion, &rotParentToThis);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, axisVector, "The axis vector does not exist.",);
    btVector3 rotationAxis;
    jmeBulletUtil::convert(pEnv, axisVector, &rotationAxis);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, parent2LinkVector, "The parent2link vector does not exist.",);
    btVector3 parentComToThisComOffset;
    jmeBulletUtil::convert(pEnv, parent2LinkVector, &parentComToThisComOffset);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setupPlanar(i, m, inertia, parent, rotParentToThis,
            rotationAxis, parentComToThisComOffset,
            (bool)disableParentCollision);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setupPrismatic
 * Signature: (JIFLcom/jme3/math/Vector3f;ILcom/jme3/math/Quaternion;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setupPrismatic
(JNIEnv *pEnv, jclass, jlong multiBodyId, jint linkIndex, jfloat mass,
        jobject inertiaVector, jint parentLinkIndex,
        jobject parent2LinkQuaternion, jobject axisVector,
        jobject parent2PivotVector, jobject pivot2LinkVector,
        jboolean disableParentCollision) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    const int i = (int) linkIndex;
    ASSERT_CHK(pEnv, i >= 0,);

    btScalar m = (btScalar) mass;
    ASSERT_CHK(pEnv, mass > 0,);

    NULL_CHK(pEnv, inertiaVector, "The inertia vector does not exist.",);
    btVector3 inertia;
    jmeBulletUtil::convert(pEnv, inertiaVector, &inertia);
    EXCEPTION_CHK(pEnv,);

    int parent = (int) parentLinkIndex;
    ASSERT_CHK(pEnv, parent >= -1,);

    NULL_CHK(pEnv, parent2LinkQuaternion,
            "The parent2Link quaternion does not exist.",);
    btQuaternion rotParentToThis;
    jmeBulletUtil::convert(pEnv, parent2LinkQuaternion, &rotParentToThis);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, axisVector, "The axis vector does not exist.",);
    btVector3 jointAxis;
    jmeBulletUtil::convert(pEnv, axisVector, &jointAxis);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, parent2PivotVector, "The parent2pivot vector does not exist.",);
    btVector3 parentComToThisPivotOffset;
    jmeBulletUtil::convert(pEnv, parent2PivotVector, &parentComToThisPivotOffset);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, pivot2LinkVector, "The pivot2link vector does not exist.",);
    btVector3 thisPivotToThisComOffset;
    jmeBulletUtil::convert(pEnv, pivot2LinkVector, &thisPivotToThisComOffset);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setupPrismatic(i, m, inertia, parent, rotParentToThis,
            jointAxis, parentComToThisPivotOffset, thisPivotToThisComOffset,
            (bool)disableParentCollision);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setupRevolute
 * Signature: (JIFLcom/jme3/math/Vector3f;ILcom/jme3/math/Quaternion;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setupRevolute
(JNIEnv *pEnv, jclass, jlong multiBodyId, jint linkIndex, jfloat mass,
        jobject inertiaVector, jint parentLinkIndex,
        jobject parent2LinkQuaternion, jobject axisVector,
        jobject parent2PivotVector, jobject pivot2LinkVector,
        jboolean disableParentCollision) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    const int i = (int) linkIndex;
    ASSERT_CHK(pEnv, i >= 0,);

    btScalar m = (btScalar) mass;
    ASSERT_CHK(pEnv, mass > 0,);

    NULL_CHK(pEnv, inertiaVector, "The inertia vector does not exist.",);
    btVector3 inertia;
    jmeBulletUtil::convert(pEnv, inertiaVector, &inertia);
    EXCEPTION_CHK(pEnv,);

    int parent = (int) parentLinkIndex;
    ASSERT_CHK(pEnv, parent >= -1,);

    NULL_CHK(pEnv, parent2LinkQuaternion,
            "The parent2Link quaternion does not exist.",);
    btQuaternion rotParentToThis;
    jmeBulletUtil::convert(pEnv, parent2LinkQuaternion, &rotParentToThis);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, axisVector, "The axis vector does not exist.",);
    btVector3 jointAxis;
    jmeBulletUtil::convert(pEnv, axisVector, &jointAxis);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, parent2PivotVector, "The parent2pivot vector does not exist.",);
    btVector3 parentComToThisPivotOffset;
    jmeBulletUtil::convert(pEnv, parent2PivotVector, &parentComToThisPivotOffset);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, pivot2LinkVector, "The pivot2link vector does not exist.",);
    btVector3 thisPivotToThisComOffset;
    jmeBulletUtil::convert(pEnv, pivot2LinkVector, &thisPivotToThisComOffset);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setupRevolute(i, m, inertia, parent, rotParentToThis,
            jointAxis, parentComToThisPivotOffset, thisPivotToThisComOffset,
            (bool)disableParentCollision);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setupSpherical
 * Signature: (JIFLcom/jme3/math/Vector3f;ILcom/jme3/math/Quaternion;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setupSpherical
(JNIEnv *pEnv, jclass, jlong multiBodyId, jint linkIndex, jfloat mass,
        jobject inertiaVector, jint parentLinkIndex,
        jobject parent2LinkQuaternion, jobject parent2PivotVector,
        jobject pivot2LinkVector, jboolean disableParentCollision) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    const int i = (int) linkIndex;
    ASSERT_CHK(pEnv, i >= 0,);

    btScalar m = (btScalar) mass;
    ASSERT_CHK(pEnv, mass > 0,);

    NULL_CHK(pEnv, inertiaVector, "The inertia vector does not exist.",);
    btVector3 inertia;
    jmeBulletUtil::convert(pEnv, inertiaVector, &inertia);
    EXCEPTION_CHK(pEnv,);

    int parent = (int) parentLinkIndex;
    ASSERT_CHK(pEnv, parent >= -1,);

    NULL_CHK(pEnv, parent2LinkQuaternion,
            "The parent2Link quaternion does not exist.",);
    btQuaternion rotParentToThis;
    jmeBulletUtil::convert(pEnv, parent2LinkQuaternion, &rotParentToThis);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, parent2PivotVector, "The parent2pivot vector does not exist.",);
    btVector3 parentComToThisPivotOffset;
    jmeBulletUtil::convert(pEnv, parent2PivotVector, &parentComToThisPivotOffset);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, pivot2LinkVector, "The pivot2link vector does not exist.",);
    btVector3 thisPivotToThisComOffset;
    jmeBulletUtil::convert(pEnv, pivot2LinkVector, &thisPivotToThisComOffset);
    EXCEPTION_CHK(pEnv,);

    pMultiBody->setupSpherical(i, m, inertia, parent, rotParentToThis,
            parentComToThisPivotOffset, thisPivotToThisComOffset,
            (bool)disableParentCollision);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setWorldToBaseRot
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setWorldToBaseRot
(JNIEnv *pEnv, jclass, jlong multiBodyId, jobject quaternion) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    NULL_CHK(pEnv, quaternion, "The quaternion does not exist.",);
    btQuaternion rot;
    jmeBulletUtil::convert(pEnv, quaternion, &rot);

    pMultiBody->setWorldToBaseRot(rot);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    useGlobalVelocities
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_useGlobalVelocities
(JNIEnv *pEnv, jclass, jlong multiBodyId, jboolean use) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    pMultiBody->useGlobalVelocities(use);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    useRK4Integration
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_useRK4Integration
(JNIEnv *pEnv, jclass, jlong multiBodyId, jboolean use) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",);

    pMultiBody->useRK4Integration(use);
}
