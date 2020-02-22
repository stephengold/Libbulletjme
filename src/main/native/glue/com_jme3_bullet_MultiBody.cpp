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
#include "com_jme3_bullet_MultiBody.h"
#include "btMultiBody.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    addBaseForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_addBaseForce
(JNIEnv *env, jobject, jlong multiBodyId, jobject forceVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    NULL_CHECK(forceVector, "The force vector does not exist.",)
    btVector3 force;
    jmeBulletUtil::convert(env, forceVector, &force);

    pMultiBody->addBaseForce(force);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    addBaseTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_addBaseTorque
(JNIEnv *env, jobject, jlong multiBodyId, jobject torqueVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    NULL_CHECK(torqueVector, "The torque vector does not exist.",)
    btVector3 torque;
    jmeBulletUtil::convert(env, torqueVector, &torque);

    pMultiBody->addBaseTorque(torque);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    addLinkForce
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_addLinkForce
(JNIEnv *env, jobject, jlong multiBodyId, jint linkIndex, jobject forceVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    NULL_CHECK(forceVector, "The force vector does not exist.",)
    btVector3 force;
    jmeBulletUtil::convert(env, forceVector, &force);

    const int i = (int) linkIndex;
    btAssert(i >= 0);

    pMultiBody->addLinkForce(i, force);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    addLinkTorque
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_addLinkTorque
(JNIEnv *env, jobject, jlong multiBodyId, jint linkIndex,
        jobject torqueVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    NULL_CHECK(torqueVector, "The torque vector does not exist.",)
    btVector3 torque;
    jmeBulletUtil::convert(env, torqueVector, &torque);

    const int i = (int) linkIndex;
    btAssert(i >= 0);

    pMultiBody->addLinkTorque(i, torque);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    canSleep
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_canSleep
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", JNI_FALSE);

    const bool canSleep = pMultiBody->getCanSleep();
    return (jboolean) canSleep;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    clearConstraintForces
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_clearConstraintForces
(JNIEnv *, jobject, jlong multiBodyId) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    pMultiBody->clearConstraintForces();
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    clearForcesAndTorques
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_clearForcesAndTorques
(JNIEnv *, jobject, jlong multiBodyId) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    pMultiBody->clearForcesAndTorques();
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    clearVelocities
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_clearVelocities
(JNIEnv *, jobject, jlong multiBodyId) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    pMultiBody->clearVelocities();
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    create
 * Signature: (IFLcom/jme3/math/Vector3f;ZZ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_MultiBody_create
(JNIEnv *env, jobject, jint numLinks, jfloat baseMass, jobject inertiaVector,
        jboolean fixedBase, jboolean canSleep) {
    jmeClasses::initJavaClasses(env);

    NULL_CHECK(inertiaVector, "The intertia vector does not exist.", 0)
    btVector3 inertia;
    jmeBulletUtil::convert(env, inertiaVector, &inertia);

    btMultiBody * const
            pMultiBody = new btMultiBody(numLinks, baseMass, inertia, fixedBase,
            canSleep);

    return reinterpret_cast<jlong> (pMultiBody);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_finalizeNative
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);

    if (pMultiBody) {
        delete pMultiBody;
    }
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getAngularMomentum
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getAngularMomentum
(JNIEnv *env, jobject, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);
    NULL_CHECK(storeVector, "The storeVector does not exist.",);

    const btVector3& angularMomentum = pMultiBody->getAngularMomentum();
    jmeBulletUtil::convert(env, &angularMomentum, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseInertia
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseInertia
(JNIEnv *env, jobject, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);
    NULL_CHECK(storeVector, "The storeVector does not exist.",);

    const btVector3& baseInertia = pMultiBody->getBaseInertia();
    jmeBulletUtil::convert(env, &baseInertia, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseMass
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getBaseMass
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0)

            const btScalar baseMass = pMultiBody->getBaseMass();
    return (jfloat) baseMass;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseOmega
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseOmega
(JNIEnv *env, jobject, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);
    NULL_CHECK(storeVector, "The storeVector does not exist.",);

    const btVector3& baseOmega = pMultiBody->getBaseOmega();
    jmeBulletUtil::convert(env, &baseOmega, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBasePos
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBasePos
(JNIEnv *env, jobject, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);
    NULL_CHECK(storeVector, "The storeVector does not exist.",);

    const btVector3& basePos = pMultiBody->getBasePos();
    jmeBulletUtil::convert(env, &basePos, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseVel
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseVel
(JNIEnv *env, jobject, jlong multiBodyId, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);
    NULL_CHECK(storeVector, "The storeVector does not exist.",);

    const btVector3& baseVel = pMultiBody->getBaseVel();
    jmeBulletUtil::convert(env, &baseVel, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getBaseWorldTransform
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getBaseWorldTransform
(JNIEnv *env, jobject, jlong multiBodyId, jobject storeTransform) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);
    NULL_CHECK(storeTransform, "The storeTransform does not exist.",);

    const btTransform& baseWorldTransform = pMultiBody->getBaseWorldTransform();
    jmeBulletUtil::convert(env, &baseWorldTransform, storeTransform);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getJointPos
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getJointPos
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const int i = (int) linkIndex;
    btAssert(i >= 0);

    const btScalar jointPos = pMultiBody->getJointPos(i);
    return (jfloat) jointPos;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getJointVel
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getJointVel
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const int i = (int) linkIndex;
    btAssert(i >= 0);

    const btScalar jointVel = pMultiBody->getJointVel(i);
    return (jfloat) jointVel;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getKineticEnergy
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getKineticEnergy
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const btScalar kineticEnergy = pMultiBody->getKineticEnergy();
    return (jfloat) kineticEnergy;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getLinkInertia
 * Signature: (JILcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getLinkInertia
(JNIEnv *env, jobject, jlong multiBodyId, jint linkIndex, jobject storeVector) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    const int i = (int) linkIndex;
    btAssert(i >= 0);

    NULL_CHECK(storeVector, "The storeVector does not exist.",);

    const btVector3& linkInertia = pMultiBody->getLinkInertia(i);
    jmeBulletUtil::convert(env, &linkInertia, storeVector);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getLinkMass
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_MultiBody_getLinkMass
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const int i = (int) linkIndex;
    btAssert(i >= 0);

    const btScalar linkMass = pMultiBody->getLinkMass(i);
    return (jfloat) linkMass;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getNumDofs
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBody_getNumDofs
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const int numDofs = pMultiBody->getNumDofs();
    return (jint) numDofs;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getNumLinks
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBody_getNumLinks
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const int numLinks = pMultiBody->getNumLinks();
    return (jint) numLinks;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getParent
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBody_getParent
(JNIEnv *, jobject, jlong multiBodyId, jint childLinkIndex) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", 0);

    const int i = (int) childLinkIndex;
    btAssert(i >= 0);

    const int parent = pMultiBody->getParent(i);
    return (jint) parent;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    getWorldToBaseRot
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_getWorldToBaseRot
(JNIEnv *env, jobject, jlong multiBodyId, jobject storeQuaternion) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);
    NULL_CHECK(storeQuaternion, "The storeQuaternion does not exist.",);

    btQuaternion worldToBaseRot = pMultiBody->getWorldToBaseRot();
    jmeBulletUtil::convert(env, &worldToBaseRot, storeQuaternion);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    hasFixedBase
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_hasFixedBase
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", JNI_FALSE);

    const bool hasFixedBase = pMultiBody->hasFixedBase();
    return (jboolean) hasFixedBase;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    isUsingGlobalVelocities
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_isUsingGlobalVelocities
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", JNI_FALSE);

    const bool isUsingGlobalVelocities = pMultiBody->isUsingGlobalVelocities();
    return (jboolean) isUsingGlobalVelocities;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    isUsingRK4Integration
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_MultiBody_isUsingRK4Integration
(JNIEnv *, jobject, jlong multiBodyId) {
    const btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.", JNI_FALSE);

    const bool isUsingRK4Integration = pMultiBody->isUsingRK4Integration();
    return (jboolean) isUsingRK4Integration;
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBaseOmega
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBaseOmega
(JNIEnv *env, jobject, jlong multiBodyId, jobject angularVelocityVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    NULL_CHECK(angularVelocityVector,
            "The angular velocity vector does not exist.",);
    btVector3 omega;
    jmeBulletUtil::convert(env, angularVelocityVector, &omega);

    pMultiBody->setBaseOmega(omega);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBasePos
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBasePos
(JNIEnv *env, jobject, jlong multiBodyId, jobject positionVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    NULL_CHECK(positionVector, "The position vector does not exist.",);
    btVector3 pos;
    jmeBulletUtil::convert(env, positionVector, &pos);

    pMultiBody->setBasePos(pos);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBaseVel
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBaseVel
(JNIEnv *env, jobject, jlong multiBodyId, jobject velocityVector) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
    btVector3 vel;
    jmeBulletUtil::convert(env, velocityVector, &vel);

    pMultiBody->setBaseVel(vel);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setBaseWorldTransform
 * Signature: (JLcom/jme3/math/Transform;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setBaseWorldTransform
(JNIEnv *env, jobject, jlong multiBodyId, jobject transform) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    NULL_CHECK(transform, "The transform does not exist.",);
    btTransform tr;
    btVector3 scale;
    jmeBulletUtil::convert(env, transform, &tr, &scale);

    pMultiBody->setBaseWorldTransform(tr);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setJointPos
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setJointPos
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex, jfloat position) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    const int i = (int) linkIndex;
    btAssert(i >= 0);
    btScalar pos = (btScalar) position;

    pMultiBody->setJointPos(i, pos);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setJointVel
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setJointVel
(JNIEnv *, jobject, jlong multiBodyId, jint linkIndex, jfloat velocity) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    const int i = (int) linkIndex;
    btAssert(i >= 0);
    btScalar vel = (btScalar) velocity;

    pMultiBody->setJointVel(i, vel);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    setWorldToBaseRot
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_setWorldToBaseRot
(JNIEnv *env, jobject, jlong multiBodyId, jobject quaternion) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    NULL_CHECK(quaternion, "The quaternion does not exist.",);
    btQuaternion rot;
    jmeBulletUtil::convert(env, quaternion, &rot);

    pMultiBody->setWorldToBaseRot(rot);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    useGlobalVelocities
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_useGlobalVelocities
(JNIEnv *, jobject, jlong multiBodyId, jboolean use) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    pMultiBody->useGlobalVelocities(use);
}

/*
 * Class:     com_jme3_bullet_MultiBody
 * Method:    useRK4Integration
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBody_useRK4Integration
(JNIEnv *, jobject, jlong multiBodyId, jboolean use) {
    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",);

    pMultiBody->useRK4Integration(use);
}
