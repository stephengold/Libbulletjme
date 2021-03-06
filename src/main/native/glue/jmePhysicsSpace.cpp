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
#include "jmePhysicsSpace.h"
#include "jmeBulletUtil.h"
#include "jmeUserInfo.h"

/*
 * Author: Normen Hansen
 */
void jmePhysicsSpace::createPhysicsSpace(const btVector3& min,
        const btVector3& max, int broadphaseId) {
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

    modify(); // Make the standard modifications.
}

bool jmePhysicsSpace::contactProcessedCallback(btManifoldPoint& contactPoint,
        void* pBody0, void* pBody1) {
    //printf("contactProcessedCallback %x %x\n", pBody0, pBody1);
    const btCollisionObject *pco0 = (btCollisionObject *) pBody0;
    jmeUserPointer const pUser0 = (jmeUserPointer) pco0->getUserPointer();
    const btCollisionObject *pco1 = (btCollisionObject *) pBody1;
    jmeUserPointer const pUser1 = (jmeUserPointer) pco1->getUserPointer();
    if (pUser0 == NULL || pUser1 == NULL) {
        printf("null userPointer in contactProcessedCallback\n");
        return true;
    }

    jmePhysicsSpace * const pSpace = (jmePhysicsSpace *) pUser0->m_jmeSpace;
    if (pSpace == NULL) {
        printf("null dynamicsWorld in contactProcessedCallback\n");
        return true;
    }

    JNIEnv * const pEnv = pSpace->getEnv();
    jobject javaPhysicsSpace = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
    if (javaPhysicsSpace == NULL) {
        printf("null javaPhysicsSpace in contactProcessedCallback\n");
        return true;
    }

    jobject javaCollisionObject0 = pEnv->NewLocalRef(pUser0->m_javaRef);
    jobject javaCollisionObject1 = pEnv->NewLocalRef(pUser1->m_javaRef);
    jlong manifoldPointId = reinterpret_cast<jlong> (&contactPoint);
    pEnv->CallVoidMethod(javaPhysicsSpace,
            jmeClasses::PhysicsSpace_addContactProcessed, javaCollisionObject0,
            javaCollisionObject1, manifoldPointId);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return true;
    }

    pEnv->DeleteLocalRef(javaPhysicsSpace);
    pEnv->DeleteLocalRef(javaCollisionObject0);
    pEnv->DeleteLocalRef(javaCollisionObject1);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return true;
    }

    return true;
}

void jmePhysicsSpace::contactStartedCallback(btPersistentManifold * const &pm) {
    const btCollisionObject *pco0 = pm->getBody0();
    const btCollisionObject *pco1 = pm->getBody1();
    //printf("contactStartedCallback %x %x\n", pco0, pco1);
    jmeUserPointer const pUser0 = (jmeUserPointer) pco0->getUserPointer();
    jmeUserPointer const pUser1 = (jmeUserPointer) pco1->getUserPointer();
    if (pUser0 != NULL && pUser1 != NULL) {
        jmePhysicsSpace * const pSpace = (jmePhysicsSpace *) pUser0->m_jmeSpace;
        if (pSpace != NULL) {
            JNIEnv * const pEnv = pSpace->getEnv();
            jobject javaPhysicsSpace
                    = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
            if (javaPhysicsSpace != NULL) {
                jobject javaCollisionObject0
                        = pEnv->NewLocalRef(pUser0->m_javaRef);
                jobject javaCollisionObject1
                        = pEnv->NewLocalRef(pUser1->m_javaRef);
                for (int i = 0; i < pm->getNumContacts(); ++i) {
                    const btManifoldPoint& cp = pm->getContactPoint(i);
                    jlong manifoldPointId = reinterpret_cast<jlong> (&cp);
                    pEnv->CallVoidMethod(javaPhysicsSpace,
                            jmeClasses::PhysicsSpace_addCollisionEvent,
                            javaCollisionObject0, javaCollisionObject1,
                            manifoldPointId);
                    if (pEnv->ExceptionCheck()) {
                        pEnv->Throw(pEnv->ExceptionOccurred());
                        return;
                    }
                }
                pEnv->DeleteLocalRef(javaPhysicsSpace);
                pEnv->DeleteLocalRef(javaCollisionObject0);
                pEnv->DeleteLocalRef(javaCollisionObject1);
                if (pEnv->ExceptionCheck()) {
                    pEnv->Throw(pEnv->ExceptionOccurred());
                    return;
                }
            } else {
                printf("null javaPhysicsSpace in contactStartedCallback\n");
            }
        } else {
            printf("null dynamicsWorld in contactStartedCallback\n");
        }
    } else {
        printf("null userPointer in contactStartedCallback\n");
    }
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
    /*
     * Ensure that both global callbacks are configured.
     */
    btAssert(gContactProcessedCallback == NULL
            || gContactProcessedCallback == &contactProcessedCallback);
    gContactProcessedCallback = &contactProcessedCallback;

    btAssert(gContactStartedCallback == NULL
            || gContactStartedCallback == &contactStartedCallback);
    gContactStartedCallback = &contactStartedCallback;
}

void jmePhysicsSpace::postTickCallback(btDynamicsWorld *pWorld,
        btScalar timeStep) {
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
        jfloat accuracy) {
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
