/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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
#include <stdio.h>

/*
 * Author: dokthar
 */
jmePhysicsSoftSpace::jmePhysicsSoftSpace(JNIEnv* env, jobject javaSpace)
: jmePhysicsSpace(env, javaSpace) {
};

// Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;IZ)V

void jmePhysicsSoftSpace::createPhysicsSoftSpace(jobject min_vec, jobject max_vec, jint broadphaseId, jboolean threading) {
    // collision configuration contains default setup for memory, collision setup
    btDefaultCollisionConstructionInfo cci;

    btVector3 min = btVector3();
    btVector3 max = btVector3();
    jmeBulletUtil::convert(this->getEnv(), min_vec, &min);
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
            //TODO: 32bit!
            broadphase = new btAxisSweep3(min, max);
            break;
        case 3:
            broadphase = new btDbvtBroadphase();
            break;
        case 4:
            //            broadphase = new btGpu3DGridBroadphase(
            //                    min, max,
            //                    20, 20, 20,
            //                    10000, 1000, 25);
            break;
    }

    btSoftBodyRigidBodyCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    // Use the default collision dispatcher.
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    // Use the default constraint solver.
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    //create dynamics world
    btSoftBodySolver* softBodySolver = 0; //use default
    btSoftRigidDynamicsWorld* world = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration, softBodySolver);
    //btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
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
            //            bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
            //            collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
            bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
            collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
            if (collides) {
                btCollisionObject* co0 = (btCollisionObject*) proxy0->m_clientObject;
                btCollisionObject* co1 = (btCollisionObject*) proxy1->m_clientObject;
                jmeUserPointer *up0 = (jmeUserPointer*) co0 -> getUserPointer();
                jmeUserPointer *up1 = (jmeUserPointer*) co1 -> getUserPointer();
                if (up0 != NULL && up1 != NULL) {
                    collides = (up0->group & up1->groups) != 0;
                    collides = collides && (up1->group & up0->groups);

                    //add some additional logic here that modified 'collides'
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
    if (gContactProcessedCallback == NULL) {
        gContactProcessedCallback = &jmePhysicsSpace::contactProcessedCallback;
    }
}

btSoftRigidDynamicsWorld* jmePhysicsSoftSpace::getSoftDynamicsWorld() {
    return (btSoftRigidDynamicsWorld*) dynamicsWorld;
}
