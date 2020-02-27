/*
 * Copyright (c) 2009-2015 jMonkeyEngine
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
#include "com_jme3_bullet_PhysicsSoftSpace.h"
#include "jmePhysicsSoftSpace.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

/*
 * Class:     com_jme3_bullet_PhysicsSoftSpace
 * Method:    addSoftBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_addSoftBody
(JNIEnv *, jobject, jlong spaceId, jlong softBodyId) {
    jmePhysicsSoftSpace * const
            pSpace = reinterpret_cast<jmePhysicsSoftSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)
    btSoftRigidDynamicsWorld * const pWorld = pSpace->getSoftDynamicsWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_SOFT_RIGID_DYNAMICS_WORLD);

    btSoftBody * const
            pSoftBody = reinterpret_cast<btSoftBody *> (softBodyId);
    NULL_CHECK(pSoftBody, "The collision object does not exist.",)
    btAssert(pSoftBody->getInternalType()
            & btCollisionObject::CO_SOFT_BODY);

    jmeUserPointer * const
            pUser = (jmeUserPointer *) pSoftBody->getUserPointer();
    pUser->space = pSpace;

    pWorld->addSoftBody(pSoftBody);
}

/*
 * Class:     com_jme3_bullet_PhysicsSoftSpace
 * Method:    createPhysicsSoftSpace
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;IZ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_createPhysicsSoftSpace
(JNIEnv *env, jobject object, jobject minVector, jobject maxVector,
        jint broadphase, jboolean) {
    jmeClasses::initJavaClasses(env);

    NULL_CHECK(minVector, "The min vector does not exist.", 0)
    btVector3 min;
    jmeBulletUtil::convert(env, minVector, &min);

    NULL_CHECK(maxVector, "The max vector does not exist.", 0)
    btVector3 max;
    jmeBulletUtil::convert(env, maxVector, &max);

    jmePhysicsSoftSpace * const pSpace = new jmePhysicsSoftSpace(env, object);
    pSpace->createPhysicsSoftSpace(min, max, (int) broadphase);

    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_PhysicsSoftSpace
 * Method:    getNumSoftBodies
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_getNumSoftBodies
(JNIEnv *, jobject, jlong spaceId) {
    const jmePhysicsSoftSpace * const
            pSpace = reinterpret_cast<jmePhysicsSoftSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.", 0)
            const btSoftRigidDynamicsWorld * const
            pWorld = pSpace->getSoftDynamicsWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_SOFT_RIGID_DYNAMICS_WORLD);

    const btSoftBodyArray& softBodies = pWorld->getSoftBodyArray();
    int sz = softBodies.size();
    return (jint) sz;
}

/*
 * Class:     com_jme3_bullet_PhysicsSoftSpace
 * Method:    getWorldInfo
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_getWorldInfo
(JNIEnv *, jobject, jlong spaceId) {
    jmePhysicsSoftSpace * const
            pSpace = reinterpret_cast<jmePhysicsSoftSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.", 0)
    btSoftRigidDynamicsWorld * const pWorld = pSpace->getSoftDynamicsWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_SOFT_RIGID_DYNAMICS_WORLD);

    btSoftBodyWorldInfo *pWorldInfo = &(pWorld->getWorldInfo());
    return reinterpret_cast<jlong> (pWorldInfo);
}

/*
 * Class:     com_jme3_bullet_PhysicsSoftSpace
 * Method:    removeSoftBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_removeSoftBody
(JNIEnv *, jobject, jlong spaceId, jlong softBodyId) {
    jmePhysicsSoftSpace * const
            pSpace = reinterpret_cast<jmePhysicsSoftSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)
    btSoftRigidDynamicsWorld * const pWorld = pSpace->getSoftDynamicsWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_SOFT_RIGID_DYNAMICS_WORLD);

    btSoftBody * const pSoftBody = reinterpret_cast<btSoftBody *> (softBodyId);
    NULL_CHECK(pSoftBody, "The collision object does not exist.",)
    btAssert(pSoftBody->getInternalType() & btCollisionObject::CO_SOFT_BODY);

    jmeUserPointer * const
            pUser = (jmeUserPointer *) pSoftBody->getUserPointer();
    pUser->space = NULL;

    pWorld->removeSoftBody(pSoftBody);
}
