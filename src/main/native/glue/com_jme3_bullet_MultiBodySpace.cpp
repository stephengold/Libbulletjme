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
#include "btMultiBody.h"
#include "btMultiBodyDynamicsWorld.h"
#include "btMultiBodyLinkCollider.h"
#include "btMultiBodyMLCPConstraintSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "com_jme3_bullet_MultiBodySpace.h"
#include "jmeBulletUtil.h"
#include "jmeMultiBodySpace.h"
#include "jmeUserInfo.h"
/*
 * Author: Stephen Gold
 */

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    addMultiBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodySpace_addMultiBody
(JNIEnv *pEnv, jclass, jlong spaceId, jlong multiBodyId) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    NULL_CHK(pEnv, pWorld, "The dynamics world does not exist.",);

    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    jmeUserPointer pUser = (jmeUserPointer) pMultiBody->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    ASSERT_CHK(pEnv, pUser->m_jmeSpace == NULL,);
    pUser->m_jmeSpace = pSpace;

    pWorld->addMultiBody(pMultiBody);
    /*
     * If there's a base collider, add it to the world.
     */
    btMultiBodyLinkCollider *pCollider = pMultiBody->getBaseCollider();
    if (pCollider && pCollider->getCollisionShape()) {
        ASSERT_CHK(pEnv, pCollider->getInternalType()
                & btCollisionObject::CO_FEATHERSTONE_LINK,);
        pUser = (jmeUserPointer) pCollider->getUserPointer();
        pUser->m_jmeSpace = pSpace;
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
            ASSERT_CHK(pEnv, pCollider->getInternalType()
                    & btCollisionObject::CO_FEATHERSTONE_LINK,);
            pUser = (jmeUserPointer) pCollider->getUserPointer();
            pUser->m_jmeSpace = pSpace;
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
(JNIEnv *pEnv, jclass, jlong spaceId, jlong constraintId) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    NULL_CHK(pEnv, pWorld, "The dynamics world does not exist.",);

    btMultiBodyConstraint * const
            pConstraint = reinterpret_cast<btMultiBodyConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The constraint does not exist.",)

    pWorld->addMultiBodyConstraint(pConstraint);
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    createMultiBodySpace
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;I)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_MultiBodySpace_createMultiBodySpace
(JNIEnv *pEnv, jobject object, jobject minVector, jobject maxVector,
        jint broadphaseType) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, minVector, "The min vector does not exist.", 0)
    btVector3 min;
    jmeBulletUtil::convert(pEnv, minVector, &min);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, maxVector, "The max vector does not exist.", 0)
    btVector3 max;
    jmeBulletUtil::convert(pEnv, maxVector, &max);
    EXCEPTION_CHK(pEnv, 0);

    jmeMultiBodySpace * const
            pSpace = new jmeMultiBodySpace(pEnv, object); //dance003
    pSpace->createMultiBodySpace(min, max, broadphaseType);

    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    getNumMultibodies
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodySpace_getNumMultibodies
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);
    const btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    NULL_CHK(pEnv, pWorld, "The dynamics world does not exist.", 0);

    int numMultiBodies = pWorld->getNumMultibodies();
    return (jint) numMultiBodies;
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    getNumMultiBodyConstraints
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_MultiBodySpace_getNumMultiBodyConstraints
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.", 0);
    const btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    NULL_CHK(pEnv, pWorld, "The dynamics world does not exist.", 0);

    int numConstraints = pWorld->getNumMultiBodyConstraints();
    return (jint) numConstraints;
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    removeMultiBody
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodySpace_removeMultiBody
(JNIEnv *pEnv, jclass, jlong spaceId, jlong multiBodyId) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    NULL_CHK(pEnv, pWorld, "The dynamics world does not exist.",);

    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The multibody does not exist.",)

    jmeUserPointer pUser = (jmeUserPointer) pMultiBody->getUserPointer();
    NULL_CHK(pEnv, pUser, "The user object does not exist.",)
    ASSERT_CHK(pEnv, pUser->m_jmeSpace == pSpace,);
    pUser->m_jmeSpace = NULL;

    pWorld->removeMultiBody(pMultiBody);
    /*
     * If there's a base collider, remove it from the world.
     */
    btMultiBodyLinkCollider *pCollider = pMultiBody->getBaseCollider();
    if (pCollider && pCollider->getCollisionShape()) {
        ASSERT_CHK(pEnv, pCollider->getInternalType()
                & btCollisionObject::CO_FEATHERSTONE_LINK,);
        pUser = (jmeUserPointer) pCollider->getUserPointer();
        pUser->m_jmeSpace = NULL;
        pWorld->removeCollisionObject((btCollisionObject *) pCollider);
    }
    /*
     * Remove any link colliders from the world.
     */
    for (int linkI = 0; linkI < pMultiBody->getNumLinks(); ++linkI) {
        pCollider = pMultiBody->getLink(linkI).m_collider;
        if (pCollider && pCollider->getCollisionShape()) {
            ASSERT_CHK(pEnv, pCollider->getInternalType()
                    & btCollisionObject::CO_FEATHERSTONE_LINK,);
            pUser = (jmeUserPointer) pCollider->getUserPointer();
            pUser->m_jmeSpace = NULL;
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
(JNIEnv *pEnv, jclass, jlong spaceId, jlong constraintId) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    NULL_CHK(pEnv, pWorld, "The dynamics world does not exist.",);

    btMultiBodyConstraint * const
            pConstraint = reinterpret_cast<btMultiBodyConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The constraint does not exist.",)

    pWorld->removeMultiBodyConstraint(pConstraint);
}

/*
 * Class:     com_jme3_bullet_MultiBodySpace
 * Method:    setSolverType
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_MultiBodySpace_setSolverType
(JNIEnv *pEnv, jclass, jlong spaceId, jint solverType) {
    jmeMultiBodySpace * const
            pSpace = reinterpret_cast<jmeMultiBodySpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)
    btMultiBodyDynamicsWorld * const pWorld = pSpace->getMultiBodyWorld();
    NULL_CHK(pEnv, pWorld, "The dynamics world does not exist.",);

    btMultiBodyConstraintSolver *pConstraintSolver;
    btMLCPSolverInterface *pMLCP;
    switch (solverType) {
        case 0: // SI
            pConstraintSolver = new btMultiBodyConstraintSolver(); // TODO leak
            break;
        case 1: // Dantzig
            pMLCP = new btDantzigSolver(); // TODO leak
            pConstraintSolver = new btMultiBodyMLCPConstraintSolver(pMLCP); // TODO leak
            break;
        case 2: // Lemke
            pMLCP = new btLemkeSolver(); // TODO leak
            pConstraintSolver = new btMultiBodyMLCPConstraintSolver(pMLCP); // TODO leak
            break;
        case 3: // PGS
            pMLCP = new btSolveProjectedGaussSeidel(); // TODO leak
            pConstraintSolver = new btMultiBodyMLCPConstraintSolver(pMLCP); // TODO leak
            break;
        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The solver type is out of range.");
            return;
    }

    pWorld->setMultiBodyConstraintSolver(pConstraintSolver);
    // TODO delete the old solver
}
