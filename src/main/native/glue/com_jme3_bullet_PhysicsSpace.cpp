/*
 * Copyright (c) 2009-2012 jMonkeyEngine
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
 * Author: Normen Hansen
 */
#include "com_jme3_bullet_PhysicsSpace.h"
#include "jmePhysicsSpace.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addAction
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addAction
(JNIEnv *, jobject, jlong spaceId, jlong actionId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    btActionInterface * const
            pAction = reinterpret_cast<btActionInterface *> (actionId);
    NULL_CHECK(pAction, "The action object does not exist.",)

    pSpace->getDynamicsWorld()->addAction(pAction);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addCharacterObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addCharacterObject
(JNIEnv *, jobject, jlong spaceId, jlong pcoId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHECK(pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer * const
            pUser = (jmeUserPointer *) pCollisionObject->getUserPointer();
    pUser->space = pSpace;

    pSpace->getDynamicsWorld()->addCollisionObject(pCollisionObject,
            btBroadphaseProxy::CharacterFilter,
            btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addConstraintC
 * Signature: (JJZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addConstraintC
(JNIEnv *, jobject, jlong spaceId, jlong constraintId, jboolean collision) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHECK(pConstraint, "The btTypedConstraint does not exist.",)

    pSpace->getDynamicsWorld()->addConstraint(pConstraint, collision);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addRigidBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addRigidBody
(JNIEnv *, jobject, jlong spaceId, jlong rigidBodyId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHECK(pBody, "The collision object does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    jmeUserPointer * const pUser = (jmeUserPointer *) pBody->getUserPointer();
    pUser->space = pSpace;

    pSpace->getDynamicsWorld()->addRigidBody(pBody);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    createPhysicsSpace
 * Signature: (FFFFFFI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_createPhysicsSpace
(JNIEnv *env, jobject object, jfloat minX, jfloat minY, jfloat minZ,
        jfloat maxX, jfloat maxY, jfloat maxZ, jint broadphase) {
    jmeClasses::initJavaClasses(env);

    jmePhysicsSpace * const pSpace = new jmePhysicsSpace(env, object);
    btVector3 min(minX, minY, minZ);
    btVector3 max(maxX, maxY, maxZ);
    pSpace->createPhysicsSpace(min, max, (int) broadphase);

    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_getGravity
(JNIEnv *env, jobject, jlong spaceId, jobject storeVector) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",);
    NULL_CHECK(storeVector, "The store vector does not exist.",);

    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    const btVector3& gravity = pWorld->getGravity();
    jmeBulletUtil::convert(env, &gravity, storeVector);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getNumConstraints
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getNumConstraints
(JNIEnv *, jobject, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.", 0);

    int count = pSpace->getDynamicsWorld()->getNumConstraints();
    return count;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getWorldType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getWorldType
(JNIEnv *, jobject, jlong spaceId) {
    const jmePhysicsSpace * const pSpace
            = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.", 0);

    btDynamicsWorldType type = pSpace->getDynamicsWorld()->getWorldType();
    return (jint) type;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeAction
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeAction
(JNIEnv *, jobject, jlong spaceId, jlong actionId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    btActionInterface * const
            pAction = reinterpret_cast<btActionInterface *> (actionId);
    NULL_CHECK(pAction, "The action object does not exist.",)

    pSpace->getDynamicsWorld()->removeAction(pAction);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeCharacterObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeCharacterObject
(JNIEnv *, jobject, jlong spaceId, jlong pcoId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHECK(pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer * const
            pUser = (jmeUserPointer *) pCollisionObject->getUserPointer();
    pUser->space = NULL;

    pSpace->getDynamicsWorld()->removeCollisionObject(pCollisionObject);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeConstraint
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeConstraint
(JNIEnv *, jobject, jlong spaceId, jlong constraintId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHECK(pConstraint, "The constraint does not exist.",)

    pSpace->getDynamicsWorld()->removeConstraint(pConstraint);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeRigidBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeRigidBody
(JNIEnv *, jobject, jlong spaceId, jlong rigidBodyId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHECK(pBody, "The collision object does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    jmeUserPointer * const pUser = (jmeUserPointer *) pBody->getUserPointer();
    pUser->space = NULL;

    pSpace->getDynamicsWorld()->removeRigidBody(pBody);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setGravity
(JNIEnv *env, jobject, jlong spaceId, jobject gravityVector) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)
    NULL_CHECK(gravityVector, "The gravity vector does not exist.",)

    btVector3 gravity;
    jmeBulletUtil::convert(env, gravityVector, &gravity);

    pSpace->getDynamicsWorld()->setGravity(gravity);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setSolverNumIterations
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setSolverNumIterations
(JNIEnv *, jobject, jlong spaceId, jint value) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    pSpace->getDynamicsWorld()->getSolverInfo().m_numIterations = value;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    stepSimulation
 * Signature: (JFIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_stepSimulation
(JNIEnv *, jobject, jlong spaceId, jfloat tpf, jint maxSteps,
        jfloat accuracy) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHECK(pSpace, "The physics space does not exist.",)

    pSpace->stepSimulation(tpf, maxSteps, accuracy);
}
