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

/*
 * Author: dokthar
 */
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "jmeClasses.h"
#include "jmePhysicsSoftSpace.h"

void jmePhysicsSoftSpace::createPhysicsSoftSpace(const btVector3& min,
        const btVector3& max, int broadphaseType,
        const btDefaultCollisionConstructionInfo *pInfo) {
    // Create the pair cache for broadphase collision detection.
    btBroadphaseInterface * const
            pBroadphase = createBroadphase(min, max, broadphaseType);

    // Register some soft-body collision algorithms on top of the default
    // collision dispatcher plus GImpact.
    btCollisionConfiguration * const pCollisionConfiguration
            = new btSoftBodyRigidBodyCollisionConfiguration(*pInfo); //dance010
    btCollisionDispatcher * const
            pDispatcher = new btCollisionDispatcher(pCollisionConfiguration); //dance008
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // Use the default constraint solver.
    btConstraintSolver * const
            pConstraintSolver = new btSequentialImpulseConstraintSolver(); //dance006

    // Create the soft-rigid dynamics world.
    btSoftBodySolver * const pSoftSolver = 0; //use default
    btSoftRigidDynamicsWorld * const
            pWorld = new btSoftRigidDynamicsWorld(pDispatcher, pBroadphase,
            pConstraintSolver, pCollisionConfiguration, pSoftSolver); //dance007
    m_pCollisionWorld = pWorld;

    // Do btSoftBodyWorldInfo modifications.
    btSoftBodyWorldInfo softBodyWorldInfo = pWorld->getWorldInfo();
    softBodyWorldInfo.m_gravity.setValue(0, -9.81f, 0);
    softBodyWorldInfo.m_sparsesdf.Initialize();
    softBodyWorldInfo.m_broadphase = pBroadphase;
    softBodyWorldInfo.m_dispatcher = pDispatcher;

    modify(); // Make the standard btDynamicsWorld modifications.
}
