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
            pCollisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * const
            pDispatcher = new btCollisionDispatcher(pCollisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // For now, use a sequential-impulse solver.
    btConstraintSolver * const
            pConstraintSolver = new btSequentialImpulseConstraintSolver();

    // Create the discrete dynamics world.
    m_collisionWorld = new btDiscreteDynamicsWorld(pDispatcher, pBroadphase,
            pConstraintSolver, pCollisionConfiguration);

    modify(); // Make the standard modifications.
}

void jmePhysicsSpace::contactStartedCallback(btPersistentManifold * const &pm) {
    const btCollisionObject *pco0 = pm->getBody0();
    const btCollisionObject *pco1 = pm->getBody1();
    //printf("contactProcessedCallback %x %x\n", co0, co1);
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
                for (int i = 0; i < pm->getNumContacts(); i++) {
                    pEnv->CallVoidMethod(javaPhysicsSpace,
                            jmeClasses::PhysicsSpace_addCollisionEvent,
                            javaCollisionObject0, javaCollisionObject1,
                            (jlong) & pm->getContactPoint(i));
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
                printf("null javaPhysicsSpace in contactProcessedCallback\n");
            }
        } else {
            printf("null dynamicsWorld in contactProcessedCallback\n");
        }
    } else {
        printf("null userPointer in contactProcessedCallback\n");
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