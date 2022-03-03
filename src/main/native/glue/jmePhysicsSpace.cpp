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
#if BT_THREADSAFE
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#endif
#include "jmePhysicsSpace.h"
#include "jmeBulletUtil.h"
#include "jmeUserInfo.h"

/*
 * Author: Normen Hansen
 */

#if BT_THREADSAFE

void jmePhysicsSpace::createMultiThreadedSpace(const btVector3& min,
        const btVector3& max, int broadphaseType, int numSolvers) {
    // Create the pair cache for broadphase collision detection.
    btBroadphaseInterface * const
            pBroadphase = createBroadphase(min, max, broadphaseType);

    // Use the default collision dispatcher plus GImpact.
    btCollisionConfiguration * const
            pCollisionConfiguration = new btDefaultCollisionConfiguration(); //dance010
    btCollisionDispatcher * const
            pDispatcher = new btCollisionDispatcher(pCollisionConfiguration); //dance008
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // For now, use sequential-impulse solvers.
    btConstraintSolver * const
            pConstraintSolver = new btSequentialImpulseConstraintSolver(); //dance006

    btConstraintSolverPoolMt * const
            pSolverPool = new btConstraintSolverPoolMt(numSolvers); // TODO leak

    // Create the multithreaded dynamics world.
    m_collisionWorld = new btDiscreteDynamicsWorldMt(pDispatcher, pBroadphase,
            pSolverPool, pConstraintSolver, pCollisionConfiguration); //dance007

    modify(); // Apply the standard modifications.
}

#else // BT_THREADSAFE

void jmePhysicsSpace::createPhysicsSpace(const btVector3& min,
        const btVector3& max, int broadphaseId) {
    // Create the pair cache for broadphase collision detection.
    btBroadphaseInterface * const
            pBroadphase = createBroadphase(min, max, broadphaseId);

    // Use the default collision dispatcher plus GImpact.
    btCollisionConfiguration * const
            pCollisionConfiguration = new btDefaultCollisionConfiguration(); //dance010
    btCollisionDispatcher * const
            pDispatcher = new btCollisionDispatcher(pCollisionConfiguration); //dance008
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // For now, use a sequential-impulse solver.
    btConstraintSolver * const
            pConstraintSolver = new btSequentialImpulseConstraintSolver(); //dance006

    // Create the discrete dynamics world.
    m_collisionWorld = new btDiscreteDynamicsWorld(pDispatcher, pBroadphase,
            pConstraintSolver, pCollisionConfiguration); //dance007

    modify(); // Apply the standard modifications.
}

#endif // BT_THREADSAFE

void jmePhysicsSpace::contactEndedCallback(btPersistentManifold * const &pm) {
    btAssert(pm->getObjectType() == BT_PERSISTENT_MANIFOLD_TYPE);
    BT_PROFILE("contactEndedCallback");

    const btCollisionObject * const pBody0 = pm->getBody0();
    if (pBody0 == NULL) {
        printf("null body in contactEndedCallback\n");
        fflush(stdout);
        return;
    }

    jmeUserPointer const pUser0 = (jmeUserPointer) pBody0->getUserPointer();
    if (pUser0 == NULL) {
        printf("null userPointer in contactEndedCallback\n");
        fflush(stdout);
        return;
    }

    jmePhysicsSpace * const pSpace = (jmePhysicsSpace *) pUser0->m_jmeSpace;
    if (pSpace == NULL) {
        printf("null jmePhysicsSpace in contactEndedCallback\n");
        fflush(stdout);
        return;
    }

#if BT_THREADSAFE
    pSpace->m_mutex.lock();
#endif

    JNIEnv * const pEnv = pSpace->getEnv();
    jobject javaPhysicsSpace
            = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
    if (javaPhysicsSpace == NULL) {
        printf("null javaPhysicsSpace in contactEndedCallback\n");
        fflush(stdout);
#if BT_THREADSAFE
        pSpace->m_mutex.unlock();
#endif
        return;
    }

    jlong manifoldId = reinterpret_cast<jlong> (pm);
    pEnv->CallVoidMethod(javaPhysicsSpace,
            jmeClasses::PhysicsSpace_onContactEnded, manifoldId);
    if (pEnv->ExceptionCheck()) {
        printf("exception in contactEndedCallback CallVoidMethod\n");
        fflush(stdout);
#if BT_THREADSAFE
        pSpace->m_mutex.unlock();
#endif
        return;
    }

    pEnv->DeleteLocalRef(javaPhysicsSpace);
    if (pEnv->ExceptionCheck()) {
        printf("exception in contactEndedCallback DeleteLocalRef\n");
        fflush(stdout);
    }

#if BT_THREADSAFE
     pSpace->m_mutex.unlock();
#endif
}

bool jmePhysicsSpace::contactProcessedCallback(btManifoldPoint& contactPoint,
        void* pBody0, void* pBody1) {
    BT_PROFILE("contactProcessedCallback");
    //printf("contactProcessedCallback %x %x\n", pBody0, pBody1);

    if (pBody0 == NULL || pBody1 == NULL) {
        printf("null body in contactProcessedCallback\n");
        fflush(stdout);
        return true;
    }
    const btCollisionObject *pco0 = (btCollisionObject *) pBody0;
    jmeUserPointer const pUser0 = (jmeUserPointer) pco0->getUserPointer();
    const btCollisionObject *pco1 = (btCollisionObject *) pBody1;
    jmeUserPointer const pUser1 = (jmeUserPointer) pco1->getUserPointer();
    if (pUser0 == NULL || pUser1 == NULL) {
        printf("null userPointer in contactProcessedCallback\n");
        fflush(stdout);
        return true;
    }

    jmePhysicsSpace * const pSpace = (jmePhysicsSpace *) pUser0->m_jmeSpace;
    if (pSpace == NULL) {
        printf("null jmePhysicsSpace in contactProcessedCallback\n");
        fflush(stdout);
        return true;
    }

#if BT_THREADSAFE
    pSpace->m_mutex.lock();
#endif
    JNIEnv * const pEnv = pSpace->getEnv();
    jobject javaPhysicsSpace = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
    if (javaPhysicsSpace == NULL) {
        printf("null javaPhysicsSpace in contactProcessedCallback\n");
        fflush(stdout);
#if BT_THREADSAFE
        pSpace->m_mutex.unlock();
#endif
        return true;
    }

    jobject javaCollisionObject0 = pEnv->NewLocalRef(pUser0->m_javaRef);
    jobject javaCollisionObject1 = pEnv->NewLocalRef(pUser1->m_javaRef);
    jlong manifoldPointId = reinterpret_cast<jlong> (&contactPoint);
    pEnv->CallVoidMethod(javaPhysicsSpace,
            jmeClasses::PhysicsSpace_onContactProcessed, javaCollisionObject0,
            javaCollisionObject1, manifoldPointId);
    if (pEnv->ExceptionCheck()) {
        printf("exception in contactProcessedCallback CallVoidMethod\n");
        fflush(stdout);
#if BT_THREADSAFE
        pSpace->m_mutex.unlock();
#endif
        return true;
    }

    pEnv->DeleteLocalRef(javaPhysicsSpace);
    pEnv->DeleteLocalRef(javaCollisionObject0);
    pEnv->DeleteLocalRef(javaCollisionObject1);
    if (pEnv->ExceptionCheck()) {
        printf("exception in contactProcessedCallback DeleteLocalRef\n");
        fflush(stdout);
    }
#if BT_THREADSAFE
    pSpace->m_mutex.unlock();
#endif

    return true;
}

void jmePhysicsSpace::contactStartedCallback(btPersistentManifold * const &pm) {
    btAssert(pm->getObjectType() == BT_PERSISTENT_MANIFOLD_TYPE);
    BT_PROFILE("contactStartedCallback");

    const btCollisionObject * const pBody0 = pm->getBody0();
    if (pBody0 == NULL) {
        printf("null body in contactStartedCallback\n");
        fflush(stdout);
        return;
    }

    jmeUserPointer const pUser0 = (jmeUserPointer) pBody0->getUserPointer();
    if (pUser0 == NULL) {
        printf("null userPointer in contactStartedCallback\n");
        fflush(stdout);
        return;
    }

    jmePhysicsSpace * const pSpace = (jmePhysicsSpace *) pUser0->m_jmeSpace;
    if (pSpace == NULL) {
        printf("null jmePhysicsSpace in contactStartedCallback\n");
        fflush(stdout);
        return;
    }

#if BT_THREADSAFE
    pSpace->m_mutex.lock();
#endif

    JNIEnv * const pEnv = pSpace->getEnv();
    jobject javaPhysicsSpace
            = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
    if (javaPhysicsSpace == NULL) {
        printf("null javaPhysicsSpace in contactStartedCallback\n");
        fflush(stdout);
#if BT_THREADSAFE
        pSpace->m_mutex.unlock();
#endif
        return;
    }

    jlong manifoldId = reinterpret_cast<jlong> (pm);
    pEnv->CallVoidMethod(javaPhysicsSpace,
        jmeClasses::PhysicsSpace_onContactStarted, manifoldId);
    if (pEnv->ExceptionCheck()) {
        printf("exception in contactStartedCallback CallVoidMethod\n");
        fflush(stdout);
#if BT_THREADSAFE
        pSpace->m_mutex.unlock();
#endif
        return;
    }

    pEnv->DeleteLocalRef(javaPhysicsSpace);
    if (pEnv->ExceptionCheck()) {
        printf("exception in contactStartedCallback DeleteLocalRef\n");
        fflush(stdout);
    }

#if BT_THREADSAFE
    pSpace->m_mutex.unlock();
#endif
}

/*
 * Apply some JME-standard modifications to a newly-created dynamics world.
 */
void jmePhysicsSpace::modify() {
    btDynamicsWorld *pWorld = getDynamicsWorld();

    pWorld->setGravity(btVector3(0, -9.81f, 0));
    pWorld->setInternalTickCallback(&jmePhysicsSpace::preTickCallback,
            static_cast<void *> (this), true);
    pWorld->setInternalTickCallback(&jmePhysicsSpace::postTickCallback,
            static_cast<void *> (this));
    pWorld->setWorldUserInfo(this);
}

void jmePhysicsSpace::postTickCallback(btDynamicsWorld *pWorld,
        btScalar timeStep) {
    BT_PROFILE("postTickCallback");

    jmePhysicsSpace * const
            pSpace = (jmePhysicsSpace *) pWorld->getWorldUserInfo();
    JNIEnv * const pEnv = pSpace->getEnv();
    jobject javaPhysicsSpace = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
    if (javaPhysicsSpace != NULL) {
        pEnv->CallVoidMethod(javaPhysicsSpace, jmeClasses::PhysicsSpace_postTick,
                timeStep);
        pEnv->DeleteLocalRef(javaPhysicsSpace);
        if (pEnv->ExceptionCheck()) {
            pEnv->Throw(pEnv->ExceptionOccurred());
            return;
        }
    }
}

void jmePhysicsSpace::preTickCallback(btDynamicsWorld *pWorld,
        btScalar timeStep) {
    BT_PROFILE("preTickCallback");

    jmePhysicsSpace * const
            pSpace = (jmePhysicsSpace *) pWorld->getWorldUserInfo();
    JNIEnv * const pEnv = pSpace->getEnv();
    jobject javaPhysicsSpace = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
    if (javaPhysicsSpace != NULL) {
        pEnv->CallVoidMethod(javaPhysicsSpace, jmeClasses::PhysicsSpace_preTick,
                timeStep);
        pEnv->DeleteLocalRef(javaPhysicsSpace);
        if (pEnv->ExceptionCheck()) {
            pEnv->Throw(pEnv->ExceptionOccurred());
            return;
        }
    }
}

void jmePhysicsSpace::stepSimulation(jfloat timeInterval, jint maxSteps,
        jfloat accuracy, jboolean enableContactEndedCallback,
        jboolean enableContactProcessedCallback,
        jboolean enableContactStartedCallback) {

    if ((bool) enableContactEndedCallback) {
        gContactEndedCallback = &contactEndedCallback;
    } else {
        gContactEndedCallback = NULL;
    }

    if ((bool) enableContactProcessedCallback) {
        gContactProcessedCallback = &contactProcessedCallback;
    } else {
        gContactProcessedCallback = NULL;
    }

    if ((bool) enableContactStartedCallback) {
        gContactStartedCallback = &contactStartedCallback;
    } else {
        gContactStartedCallback = NULL;
    }

    btDynamicsWorld * const pWorld = getDynamicsWorld();
    pWorld->stepSimulation((btScalar) timeInterval, (int) maxSteps,
            (btScalar) accuracy);
}

jmePhysicsSpace::~jmePhysicsSpace() {
    btDynamicsWorld * const pWorld = getDynamicsWorld();

    btConstraintSolver *pConstraintSolver = pWorld->getConstraintSolver();
    if (pConstraintSolver) {
        delete pConstraintSolver; //dance006
    }
}
