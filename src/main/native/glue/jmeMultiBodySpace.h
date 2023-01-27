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
#ifndef JME_MULTI_BODY_SPACE_H
#define JME_MULTI_BODY_SPACE_H

#include "jmePhysicsSpace.h"
#include "btMultiBodyDynamicsWorld.h"

/*
 * Author: Stephen Gold
 */

/*
 * A jmePhysicsSpace with support for multibodies.
 */
class jmeMultiBodySpace : public jmePhysicsSpace {
public:
    /*
     * constructor:
     */
    jmeMultiBodySpace(JNIEnv *pEnv, jobject javaSpace)
    : jmePhysicsSpace(pEnv, javaSpace) {
    }
    /*
     * configuration:
     */
    void
    createMultiBodySpace(const btVector3& min, const btVector3& max,
            jint broadphaseType);
    /*
     * getters:
     */
    const btMultiBodyDynamicsWorld *
    getMultiBodyWorld() const {
        return (btMultiBodyDynamicsWorld *) m_pCollisionWorld;
    }
    btMultiBodyDynamicsWorld *
    getMultiBodyWorld() {
        return (btMultiBodyDynamicsWorld *) m_pCollisionWorld;
    }
};

#endif // JME_MULTI_BODY_SPACE_H