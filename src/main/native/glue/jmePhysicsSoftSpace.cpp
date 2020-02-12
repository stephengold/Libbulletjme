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
#include "jmePhysicsSoftSpace.h"
#include "jmeBulletUtil.h"

/*
 * Author: dokthar
 */
jmePhysicsSoftSpace::jmePhysicsSoftSpace(JNIEnv *env, jobject javaSpace)
: jmePhysicsSpace(env, javaSpace) {
};

void jmePhysicsSoftSpace::createPhysicsSoftSpace(jobject min_vec,
        jobject max_vec, jint broadphaseId) {
    btVector3 min;
    jmeBulletUtil::convert(this->getEnv(), min_vec, &min);
    btVector3 max;
    jmeBulletUtil::convert(this->getEnv(), max_vec, &max);

    btBroadphaseInterface *pBroadphase;
    switch (broadphaseId) {
        case 0:
            pBroadphase = new btSimpleBroadphase();
            break;
        case 1:
            pBroadphase = new btAxisSweep3(min, max);
            break;
        case 2:
            pBroadphase = new bt32BitAxisSweep3(min, max);
            break;
        case 3:
            pBroadphase = new btDbvtBroadphase();
            break;
        default:
            env->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The broadphase type is out of range.");
    }

    // Register some soft-body collision algorithms on top of the default
    // collision dispatcher configuration.
    btCollisionConfiguration *pCollisionConfiguration
            = new btSoftBodyRigidBodyCollisionConfiguration();
    btCollisionDispatcher *pDispatcher
            = new btCollisionDispatcher(pCollisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // Use the default constraint solver.
    btConstraintSolver *pConstraintSolver
            = new btSequentialImpulseConstraintSolver();

    //create dynamics world
    btSoftBodySolver *pSoftSolver = 0; //use default
    btSoftRigidDynamicsWorld *pWorld = new btSoftRigidDynamicsWorld(pDispatcher,
            pBroadphase, pConstraintSolver, pCollisionConfiguration,
            pSoftSolver);
    dynamicsWorld = pWorld;
    dynamicsWorld->setWorldUserInfo(this);

    pBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));
    // do SoftBodyWorldInfo modifications
    btSoftBodyWorldInfo softBodyWorldInfo = pWorld->getWorldInfo();
    softBodyWorldInfo.m_gravity.setValue(0, -9.81f, 0);
    softBodyWorldInfo.m_sparsesdf.Initialize();
    softBodyWorldInfo.m_broadphase = pBroadphase;
    softBodyWorldInfo.m_dispatcher = pDispatcher;

    struct jmeFilterCallback : public btOverlapFilterCallback {
        // return true when pairs need collision

        virtual bool needBroadphaseCollision(btBroadphaseProxy *pProxy0, btBroadphaseProxy *pProxy1) const {
            bool collides = (pProxy0->m_collisionFilterGroup & pProxy1->m_collisionFilterMask) != 0;
            collides = collides || (pProxy1->m_collisionFilterGroup & pProxy0->m_collisionFilterMask);
            if (collides) {
                btCollisionObject *pco0 = (btCollisionObject *) pProxy0->m_clientObject;
                btCollisionObject *pco1 = (btCollisionObject *) pProxy1->m_clientObject;
                jmeUserPointer *pUser0 = (jmeUserPointer *) pco0->getUserPointer();
                jmeUserPointer *pUser1 = (jmeUserPointer *) pco1->getUserPointer();
                if (pUser0 != NULL && pUser1 != NULL) {
                    collides = (pUser0->group & pUser1->groups) != 0
                            || (pUser1->group & pUser0->groups) != 0;

                    if (collides) {
                        jmePhysicsSpace *pSpace = (jmePhysicsSpace *) pUser0->space;
                        JNIEnv *env = pSpace->getEnv();
                        jobject javaPhysicsSpace = env->NewLocalRef(pSpace->getJavaPhysicsSpace());
                        jobject javaCollisionObject0 = env->NewLocalRef(pUser0->javaCollisionObject);
                        jobject javaCollisionObject1 = env->NewLocalRef(pUser1->javaCollisionObject);

                        jboolean notifyResult = env->CallBooleanMethod(
                                javaPhysicsSpace,
                                jmeClasses::PhysicsSpace_notifyCollisionGroupListeners,
                                javaCollisionObject0, javaCollisionObject1);

                        env->DeleteLocalRef(javaPhysicsSpace);
                        env->DeleteLocalRef(javaCollisionObject0);
                        env->DeleteLocalRef(javaCollisionObject1);

                        if (env->ExceptionCheck()) {
                            env->Throw(env->ExceptionOccurred());
                            return collides;
                        }

                        collides = (bool) notifyResult;
                    }

                    //add some additional logic here that modifies 'collides'
                    return collides;
                }
                return false;
            }
            return collides;
        }
    };
    dynamicsWorld->getPairCache()->setOverlapFilterCallback(new jmeFilterCallback());
    dynamicsWorld->setInternalTickCallback(&jmePhysicsSpace::preTickCallback, static_cast<void *> (this), true);
    dynamicsWorld->setInternalTickCallback(&jmePhysicsSpace::postTickCallback, static_cast<void *> (this));

    btAssert(gContactStartedCallback == NULL
            || gContactStartedCallback == &jmePhysicsSpace::contactStartedCallback);
    gContactStartedCallback = &jmePhysicsSpace::contactStartedCallback;
}

btSoftRigidDynamicsWorld * jmePhysicsSoftSpace::getSoftDynamicsWorld() {
    return (btSoftRigidDynamicsWorld *) dynamicsWorld;
}
