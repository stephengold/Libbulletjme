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
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "com_jme3_bullet_PhysicsSpace.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "jmeUserInfo.h"

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getWorldType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getWorldType
(JNIEnv *pEnv, jobject, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);

    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    btDynamicsWorldType type = pWorld->getWorldType();
    return jint(type);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addAction
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addAction
(JNIEnv *pEnv, jobject, jlong spaceId, jlong actionId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btActionInterface * const
            pAction = reinterpret_cast<btActionInterface *> (actionId);
    NULL_CHK(pEnv, pAction, "The action object does not exist.",)

    pSpace->getDynamicsWorld()->addAction(pAction);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addCharacterObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addCharacterObject
(JNIEnv *pEnv, jobject, jlong spaceId, jlong pcoId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    pUser->m_jmeSpace = pSpace;

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
(JNIEnv *pEnv, jobject, jlong spaceId, jlong constraintId, jboolean collision) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",)
    btAssert(pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE);
    btAssert(pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE);

    btDynamicsWorld *pWorld = pSpace->getDynamicsWorld();
    pWorld->addConstraint(pConstraint, collision);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    addRigidBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_addRigidBody
(JNIEnv *pEnv, jobject, jlong spaceId, jlong rigidBodyId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHK(pEnv, pBody, "The collision object does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    jmeUserPointer const pUser = (jmeUserPointer) pBody->getUserPointer();
    pUser->m_jmeSpace = pSpace;

    pSpace->getDynamicsWorld()->addRigidBody(pBody);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    createPhysicsSpace
 * Signature: (FFFFFFI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_createPhysicsSpace
(JNIEnv *pEnv, jobject object, jfloat minX, jfloat minY, jfloat minZ,
        jfloat maxX, jfloat maxY, jfloat maxZ, jint broadphase) {
    jmeClasses::initJavaClasses(pEnv);

    jmePhysicsSpace * const pSpace = new jmePhysicsSpace(pEnv, object);
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
(JNIEnv *pEnv, jobject, jlong spaceId, jobject storeVector) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    const btVector3& gravity = pWorld->getGravity();
    jmeBulletUtil::convert(pEnv, &gravity, storeVector);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getNumConstraints
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_PhysicsSpace_getNumConstraints
(JNIEnv *pEnv, jobject, jlong spaceId) {
    const jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);

    int count = pSpace->getDynamicsWorld()->getNumConstraints();
    return (jint) count;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    getSolverInfo
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSpace_getSolverInfo
(JNIEnv *pEnv, jobject, jlong spaceId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);

    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    btContactSolverInfo *pInfo = &pWorld->getSolverInfo();
    return reinterpret_cast<jlong> (pInfo);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeAction
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeAction
(JNIEnv *pEnv, jobject, jlong spaceId, jlong actionId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btActionInterface * const
            pAction = reinterpret_cast<btActionInterface *> (actionId);
    NULL_CHK(pEnv, pAction, "The action object does not exist.",)

    pSpace->getDynamicsWorld()->removeAction(pAction);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeCharacterObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeCharacterObject
(JNIEnv *pEnv, jobject, jlong spaceId, jlong pcoId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    pUser->m_jmeSpace = NULL;

    pSpace->getDynamicsWorld()->removeCollisionObject(pCollisionObject);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeConstraint
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeConstraint
(JNIEnv *pEnv, jobject, jlong spaceId, jlong constraintId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The constraint does not exist.",)
    btAssert(pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE);
    btAssert(pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE);

    pSpace->getDynamicsWorld()->removeConstraint(pConstraint);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    removeRigidBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_removeRigidBody
(JNIEnv *pEnv, jobject, jlong spaceId, jlong rigidBodyId) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (rigidBodyId);
    NULL_CHK(pEnv, pBody, "The collision object does not exist.",)
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    jmeUserPointer const pUser = (jmeUserPointer) pBody->getUserPointer();
    pUser->m_jmeSpace = NULL;

    pSpace->getDynamicsWorld()->removeRigidBody(pBody);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setGravity
(JNIEnv *pEnv, jobject, jlong spaceId, jobject gravityVector) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    NULL_CHK(pEnv, gravityVector, "The gravity vector does not exist.",)

    btVector3 gravity;
    jmeBulletUtil::convert(pEnv, gravityVector, &gravity);

    pSpace->getDynamicsWorld()->setGravity(gravity);
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    setSolverType
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_setSolverType
(JNIEnv *pEnv, jobject, jlong spaceId, jint solverType) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    btAssert(pWorld != NULL);
    btAssert(pWorld->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD);

    btConstraintSolver *pConstraintSolver;
    btMLCPSolverInterface *pMLCP;
    switch (solverType) {
        case 0: // SI
            pConstraintSolver = new btSequentialImpulseConstraintSolver();
            break;
        case 1: // Dantzig
            pMLCP = new btDantzigSolver();
            pConstraintSolver = new btMLCPSolver(pMLCP);
            break;
        case 2: // Lemke
            pMLCP = new btLemkeSolver();
            pConstraintSolver = new btMLCPSolver(pMLCP);
            break;
        case 3: // PGS
            pMLCP = new btSolveProjectedGaussSeidel();
            pConstraintSolver = new btMLCPSolver(pMLCP);
            break;
        case 4: // NNCG
            pConstraintSolver = new btNNCGConstraintSolver();
            break;
        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The solver type is out of range.");
    }

    btConstraintSolver *pOldSolver = pWorld->getConstraintSolver();
    pWorld->setConstraintSolver(pConstraintSolver);
    delete pOldSolver;
}

/*
 * Class:     com_jme3_bullet_PhysicsSpace
 * Method:    stepSimulation
 * Signature: (JFIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSpace_stepSimulation
(JNIEnv *pEnv, jobject, jlong spaceId, jfloat tpf, jint maxSteps,
        jfloat accuracy) {
    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    pSpace->stepSimulation(tpf, maxSteps, accuracy);
}
