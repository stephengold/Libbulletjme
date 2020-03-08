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
#include "com_jme3_bullet_MultiBodySpace.h"
#include "jmeMultiBodySpace.h"
#include "jmeBulletUtil.h"
#include "btMultiBody.h"
#include "btMultiBodyDynamicsWorld.h"
#include "btMultiBodyLinkCollider.h"

/*
 * Author: Stephen Gold
 */

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    addMultiBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodySpace_addMultiBody
(JNIEnv *, jobject, jlong spaceId, jlong multiBodyId) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD);

    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    jmeUserPointer *pUser = (jmeUserPointer *) pMultiBody->getUserPointer();
    pUser->space = pSpace;
    pWorld->addMultiBody(pMultiBody);
    /*
     * If there's a base collider, add it to the world.
     */
    btMultiBodyLinkCollider *pCollider = pMultiBody->getBaseCollider();
    if (pCollider && pCollider->getCollisionShape()) {
        btAssert(pCollider->getInternalType()
                & btCollisionObject::CO_FEATHERSTONE_LINK);
        pUser = (jmeUserPointer *) pCollider->getUserPointer();
        pUser->space = pSpace;
        int cfGroup, cfMask;
        if (pMultiBody->hasFixedBase()) {
            cfGroup = btBroadphaseProxy::StaticFilter;
            cfMask = ~btBroadphaseProxy::StaticFilter;
        } else {
            cfGroup = btBroadphaseProxy::DefaultFilter;
            cfMask = btBroadphaseProxy::AllFilter;
        }
        pWorld->addCollisionObject((btCollisionObject *) pCollider, cfGroup,
                cfMask);
    }
    /*
     * Add any link colliders to the world.
     */
    for (int linkI = 0; linkI < pMultiBody->getNumLinks(); ++linkI) {
        pCollider = pMultiBody->getLink(linkI).m_collider;
        if (pCollider && pCollider->getCollisionShape()) {
            btAssert(pCollider->getInternalType()
                    & btCollisionObject::CO_FEATHERSTONE_LINK);
            pUser = (jmeUserPointer *) pCollider->getUserPointer();
            pUser->space = pSpace;
            pWorld->addCollisionObject((btCollisionObject *) pCollider,
                    btBroadphaseProxy::DefaultFilter,
                    btBroadphaseProxy::AllFilter);
        }
    }
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    addMultiBodyConstraint
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodySpace_addMultiBodyConstraint
(JNIEnv *, jobject, jlong spaceId, jlong constraintId) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD);

    btMultiBodyConstraint * const
            pConstraint = reinterpret_cast<btMultiBodyConstraint *> (constraintId);
    NULL_CHECK(pConstraint, "The constraint does not exist.",)

    pWorld->addMultiBodyConstraint(pConstraint);
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    createMultiBodySpace
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;I)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_MultiBodySpace_createMultiBodySpace
(JNIEnv *env, jobject object, jobject minVector, jobject maxVector,
        jint broadphaseType) {
    jmeClasses::initJavaClasses(env);

    NULL_CHECK(minVector, "The min vector does not exist.", 0)
    btVector3 min;
    jmeBulletUtil::convert(env, minVector, &min);

    NULL_CHECK(maxVector, "The max vector does not exist.", 0)
    btVector3 max;
    jmeBulletUtil::convert(env, maxVector, &max);

    jmeMultiBodySpace * const pSpace = new jmeMultiBodySpace(env, object);
    pSpace->createMultiBodySpace(min, max, (int) broadphaseType);

    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    getNumMultibodies
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodySpace_getNumMultibodies
(JNIEnv *, jobject, jlong spaceId) {
    const jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.", 0)
            const btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD);

    int numMultiBodies = pWorld->getNumMultibodies();
    return (jint) numMultiBodies;
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    getNumMultiBodyConstraints
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodySpace_getNumMultiBodyConstraints
(JNIEnv *, jobject, jlong spaceId) {
    const jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.", 0);
    const btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD);

    int numConstraints = pWorld->getNumMultiBodyConstraints();
    return (jint) numConstraints;
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    removeMultiBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodySpace_removeMultiBody
(JNIEnv *, jobject, jlong spaceId, jlong multiBodyId) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD);

    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHECK(pMultiBody, "The multibody does not exist.",)

    jmeUserPointer *pUser = (jmeUserPointer *) pMultiBody->getUserPointer();
    pUser->space = NULL;
    pWorld->removeMultiBody(pMultiBody);
    /*
     * If there's a base collider, remove it from the world.
     */
    btMultiBodyLinkCollider *pCollider = pMultiBody->getBaseCollider();
    if (pCollider && pCollider->getCollisionShape()) {
        btAssert(pCollider->getInternalType()
                & btCollisionObject::CO_FEATHERSTONE_LINK);
        pUser = (jmeUserPointer *) pCollider->getUserPointer();
        pUser->space = NULL;
        pWorld->removeCollisionObject((btCollisionObject *) pCollider);
    }
    /*
     * Remove any link colliders from the world.
     */
    for (int linkI = 0; linkI < pMultiBody->getNumLinks(); ++linkI) {
        pCollider = pMultiBody->getLink(linkI).m_collider;
        if (pCollider && pCollider->getCollisionShape()) {
            btAssert(pCollider->getInternalType()
                    & btCollisionObject::CO_FEATHERSTONE_LINK);
            pUser = (jmeUserPointer *) pCollider->getUserPointer();
            pUser->space = NULL;
            pWorld->removeCollisionObject((btCollisionObject *) pCollider);
        }
    }

    pWorld->removeMultiBody(pMultiBody);
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    removeMultiBodyConstraint
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodySpace_removeMultiBodyConstraint
(JNIEnv *, jobject, jlong spaceId, jlong constraintId) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD);

    btMultiBodyConstraint * const
            pConstraint = reinterpret_cast<btMultiBodyConstraint *> (constraintId);
    NULL_CHECK(pConstraint, "The constraint does not exist.",)

    pWorld->removeMultiBodyConstraint(pConstraint);
}
