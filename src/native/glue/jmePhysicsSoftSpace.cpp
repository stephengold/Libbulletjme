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
jmePhysicsSoftSpace::jmePhysicsSoftSpace(JNIEnv* env, jobject javaSpace)
: jmePhysicsSpace(env, javaSpace) {
};

// Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;IZ)V

void jmePhysicsSoftSpace::createPhysicsSoftSpace(jobject min_vec,
        jobject max_vec, jint broadphaseId,
        jboolean threading /*unused*/) {
    btVector3 min;
    jmeBulletUtil::convert(this->getEnv(), min_vec, &min);
    btVector3 max;
    jmeBulletUtil::convert(this->getEnv(), max_vec, &max);

    btBroadphaseInterface* broadphase;

    switch (broadphaseId) {
        case 0:
            broadphase = new btSimpleBroadphase();
            break;
        case 1:
            broadphase = new btAxisSweep3(min, max);
            break;
        case 2:
            broadphase = new bt32BitAxisSweep3(min, max);
            break;
        case 3:
            broadphase = new btDbvtBroadphase();
            break;
    }

    // Register some soft-body collision algorithms on top of the default
    // collision dispatcher configuration.
    btCollisionConfiguration* collisionConfiguration
            = new btSoftBodyRigidBodyCollisionConfiguration();
    btCollisionDispatcher* dispatcher
            = new btCollisionDispatcher(collisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

    // Use the default constraint solver.
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

    //create dynamics world
    btSoftBodySolver* softBodySolver = 0; //use default
    btSoftRigidDynamicsWorld* world
            = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver,
            collisionConfiguration, softBodySolver);
    dynamicsWorld = world;
    dynamicsWorld->setWorldUserInfo(this);

    broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));
    // do SoftBodyWorldInfo modifications
    btSoftBodyWorldInfo softBodyWorldInfo = world->getWorldInfo();
    softBodyWorldInfo.m_gravity.setValue(0, -9.81f, 0);
    softBodyWorldInfo.m_sparsesdf.Initialize();
    softBodyWorldInfo.m_broadphase = broadphase;
    softBodyWorldInfo.m_dispatcher = dispatcher;

    struct jmeFilterCallback : public btOverlapFilterCallback {
        // return true when pairs need collision

        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy * proxy1) const {
            bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
            collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
            if (collides) {
                btCollisionObject* co0 = (btCollisionObject*) proxy0->m_clientObject;
                btCollisionObject* co1 = (btCollisionObject*) proxy1->m_clientObject;
                jmeUserPointer *up0 = (jmeUserPointer*) co0 -> getUserPointer();
                jmeUserPointer *up1 = (jmeUserPointer*) co1 -> getUserPointer();
                if (up0 != NULL && up1 != NULL) {
                    collides = (up0->group & up1->groups) != 0 || (up1->group & up0->groups) != 0;

                    if (collides) {
                        jmePhysicsSpace *dynamicsWorld = (jmePhysicsSpace *) up0->space;
                        JNIEnv* env = dynamicsWorld->getEnv();
                        jobject javaPhysicsSpace = env->NewLocalRef(dynamicsWorld->getJavaPhysicsSpace());
                        jobject javaCollisionObject0 = env->NewLocalRef(up0->javaCollisionObject);
                        jobject javaCollisionObject1 = env->NewLocalRef(up1->javaCollisionObject);

                        jboolean notifyResult
                                = env->CallBooleanMethod(javaPhysicsSpace, jmeClasses::PhysicsSpace_notifyCollisionGroupListeners, javaCollisionObject0, javaCollisionObject1);

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
    if (gContactStartedCallback == NULL) {
        gContactStartedCallback = &jmePhysicsSpace::contactStartedCallback;
    }
}

btSoftRigidDynamicsWorld* jmePhysicsSoftSpace::getSoftDynamicsWorld() {
    return (btSoftRigidDynamicsWorld*) dynamicsWorld;
}
