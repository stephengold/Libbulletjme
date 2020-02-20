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

/*
 * Author: Normen Hansen
 */
void jmePhysicsSpace::createPhysicsSpace(jfloat minX, jfloat minY, jfloat minZ,
        jfloat maxX, jfloat maxY, jfloat maxZ, jint broadphaseId) {
    const btVector3 min = btVector3(minX, minY, minZ);
    const btVector3 max = btVector3(maxX, maxY, maxZ);

    btBroadphaseInterface * const pBroadphase
            = createBroadphase(min, max, broadphaseId);

    // Use the default collision dispatcher plus GImpact.
    btCollisionConfiguration * const pCollisionConfiguration
            = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * const pDispatcher
            = new btCollisionDispatcher(pCollisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // Use the default constraint solver.
    btConstraintSolver * const pConstraintSolver
            = new btSequentialImpulseConstraintSolver();

    // Create the discrete dynamics world.
    btDiscreteDynamicsWorld * const pWorld = new btDiscreteDynamicsWorld(
            pDispatcher, pBroadphase, pConstraintSolver,
            pCollisionConfiguration);
    m_collisionWorld = pWorld;

    // Do btDynamicsWorld modifications.
    pWorld->setGravity(btVector3(0, -9.81f, 0));
    pWorld->setInternalTickCallback(&jmePhysicsSpace::preTickCallback, static_cast<void *> (this), true);
    pWorld->setInternalTickCallback(&jmePhysicsSpace::postTickCallback, static_cast<void *> (this));
    pWorld->setWorldUserInfo(this);

    btAssert(gContactStartedCallback == NULL
            || gContactStartedCallback == &contactStartedCallback);
    gContactStartedCallback = &contactStartedCallback;
}

void jmePhysicsSpace::contactStartedCallback(btPersistentManifold * const &pm) {
    const btCollisionObject *pco0 = pm->getBody0();
    const btCollisionObject *pco1 = pm->getBody1();
    //printf("contactProcessedCallback %x %x\n", co0, co1);
    jmeUserPointer *pUser0 = (jmeUserPointer *) pco0->getUserPointer();
    jmeUserPointer *pUser1 = (jmeUserPointer *) pco1->getUserPointer();
    if (pUser0 != NULL && pUser1 != NULL) {
        jmePhysicsSpace *pSpace = (jmePhysicsSpace *) pUser0->space;
        if (pSpace != NULL) {
            JNIEnv *env = pSpace->getEnv();
            jobject javaPhysicsSpace
                    = env->NewLocalRef(pSpace->getJavaPhysicsSpace());
            if (javaPhysicsSpace != NULL) {
                jobject javaCollisionObject0
                        = env->NewLocalRef(pUser0->javaCollisionObject);
                jobject javaCollisionObject1
                        = env->NewLocalRef(pUser1->javaCollisionObject);
                for (int i = 0; i < pm->getNumContacts(); i++) {
                    env->CallVoidMethod(javaPhysicsSpace,
                            jmeClasses::PhysicsSpace_addCollisionEvent,
                            javaCollisionObject0, javaCollisionObject1,
                            (jlong) & pm->getContactPoint(i));
                    if (env->ExceptionCheck()) {
                        env->Throw(env->ExceptionOccurred());
                        return;
                    }
                }
                env->DeleteLocalRef(javaPhysicsSpace);
                env->DeleteLocalRef(javaCollisionObject0);
                env->DeleteLocalRef(javaCollisionObject1);
                if (env->ExceptionCheck()) {
                    env->Throw(env->ExceptionOccurred());
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

void jmePhysicsSpace::postTickCallback(btDynamicsWorld *pWorld,
        btScalar timeStep) {
    jmePhysicsSpace *pSpace = (jmePhysicsSpace *) pWorld->getWorldUserInfo();
    JNIEnv *env = pSpace->getEnv();
    jobject javaPhysicsSpace = env->NewLocalRef(pSpace->getJavaPhysicsSpace());
    if (javaPhysicsSpace != NULL) {
        env->CallVoidMethod(javaPhysicsSpace, jmeClasses::PhysicsSpace_postTick,
                timeStep);
        env->DeleteLocalRef(javaPhysicsSpace);
        if (env->ExceptionCheck()) {
            env->Throw(env->ExceptionOccurred());
            return;
        }
    }
}

void jmePhysicsSpace::preTickCallback(btDynamicsWorld *pWorld,
        btScalar timeStep) {
    jmePhysicsSpace *pSpace = (jmePhysicsSpace *) pWorld->getWorldUserInfo();
    JNIEnv *env = pSpace->getEnv();
    jobject javaPhysicsSpace = env->NewLocalRef(pSpace->getJavaPhysicsSpace());
    if (javaPhysicsSpace != NULL) {
        env->CallVoidMethod(javaPhysicsSpace, jmeClasses::PhysicsSpace_preTick,
                timeStep);
        env->DeleteLocalRef(javaPhysicsSpace);
        if (env->ExceptionCheck()) {
            env->Throw(env->ExceptionOccurred());
            return;
        }
    }
}

void jmePhysicsSpace::stepSimulation(jfloat timeInterval, jint maxSteps,
        jfloat accuracy) {
    btDynamicsWorld *pWorld = getDynamicsWorld();
    pWorld->stepSimulation(timeInterval, maxSteps, accuracy);
}