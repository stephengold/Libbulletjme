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
#ifndef JME_PHYSICS_SPACE_H
#define JME_PHYSICS_SPACE_H

/*
 * Author: Normen Hansen
 */
#include "jmeCollisionSpace.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

class jmePhysicsSpace : public jmeCollisionSpace {
protected:
    void
    modify();

public:
    /*
     * constructor:
     */
    jmePhysicsSpace(JNIEnv *pEnv, jobject javaSpace)
    : jmeCollisionSpace(pEnv, javaSpace) {
    }

    virtual ~jmePhysicsSpace();

    static void
    contactEndedCallback(btPersistentManifold * const &);

    static bool
    contactProcessedCallback(btManifoldPoint&, void* pBody0, void* pBody1);

    static void
    contactStartedCallback(btPersistentManifold * const &);

    /*
     * configuration:
     */
    void
#if BT_THREADSAFE
    createMultiThreadedSpace(const btVector3& min, const btVector3& max,
            int broadphaseType, int numSolvers,
            const btDefaultCollisionConstructionInfo *);
#else
    createPhysicsSpace(const btVector3& min, const btVector3& max,
            int broadphaseType, const btDefaultCollisionConstructionInfo *);
#endif // BT_THREADSAFE

    /*
     * getters:
     */
    const btDiscreteDynamicsWorld *
    getDynamicsWorld() const {
        return (btDiscreteDynamicsWorld *) m_pCollisionWorld;
    }

    btDiscreteDynamicsWorld *
    getDynamicsWorld() {
        return (btDiscreteDynamicsWorld *) m_pCollisionWorld;
    }

    static void
    postTickCallback(btDynamicsWorld *, btScalar);

    static void
    preTickCallback(btDynamicsWorld *, btScalar);

    void
    stepSimulation(jfloat timeInterval, jint maxSteps, jfloat accuracy,
            jboolean enableContactEndedCallback,
            jboolean enableContactProcessedCallback,
            jboolean enableContactStartedCallback);
};

#endif // JME_PHYSICS_SPACE_H
