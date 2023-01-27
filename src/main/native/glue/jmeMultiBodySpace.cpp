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
#include "jmeMultiBodySpace.h"
#include "jmeClasses.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"

/*
 * Author: Stephen Gold
 */
void jmeMultiBodySpace::
createMultiBodySpace(const btVector3& min, const btVector3& max,
        jint broadphaseType) {
    // Create the pair cache for broadphase collision detection.
    btBroadphaseInterface * const
            pBroadphase = createBroadphase(min, max, broadphaseType);

    // Use the default collision dispatcher plus GImpact.
    btCollisionConfiguration * const
            pCollisionConfiguration = new btDefaultCollisionConfiguration(); //dance010
    btCollisionDispatcher * const
            pDispatcher = new btCollisionDispatcher(pCollisionConfiguration); //dance008
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // For now, use a sequential-impulse solver.
    btMultiBodyConstraintSolver * const
            pConstraintSolver = new btMultiBodyConstraintSolver(); //dance006

    // Create the multibody dynamics world.
    m_pCollisionWorld = new btMultiBodyDynamicsWorld(pDispatcher, pBroadphase,
            pConstraintSolver, pCollisionConfiguration); //dance007

    modify(); // Make the standard modifications.
}
