/*
 * Copyright (c) 2009-2020 jMonkeyEngine
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
#ifndef _Included_jmeCollisionSpace
#define _Included_jmeCollisionSpace

/*
 * Author: Normen Hansen
 */
#include <jni.h>
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "LinearMath/btThreads.h"

struct jmeFilterCallback : public btOverlapFilterCallback {
    bool needBroadphaseCollision(btBroadphaseProxy*, btBroadphaseProxy*) const;
};

class jmeCollisionSpace {
protected:
    btCollisionWorld * m_pCollisionWorld;
    /*
     * interface pointer most recently attached by this space:
     */
    JNIEnv *m_pAttachEnv;
    /*
     * interface pointer for the thread that created this space:
     */
    JNIEnv *m_pCreateEnv;
    jweak m_javaSpace;
    /*
     * exclusive access to the JavaVM and JNIEnv during parallel for loops:
     */
    btSpinMutex m_mutex;

    void attachThread();
    btBroadphaseInterface * createBroadphase(const btVector3 & min,
            const btVector3 & max, int broadphaseType);

public:
    /*
     * constructor:
     */
    jmeCollisionSpace(JNIEnv *, jobject javaSpace);

    virtual ~jmeCollisionSpace();

    void
    createCollisionSpace(const btVector3& min, const btVector3& max,
            int broadphaseType, const btDefaultCollisionConstructionInfo *);

    const btCollisionWorld *
    getCollisionWorld() const {
        return m_pCollisionWorld;
    }

    btCollisionWorld *
    getCollisionWorld() {
        return m_pCollisionWorld;
    }

    const JNIEnv *
    getAttachEnv() const {
        return m_pAttachEnv;
    }

    const JNIEnv *
    getCreateEnv() const {
        return m_pCreateEnv;
    }

    JNIEnv *
    getEnvAndAttach() {
        attachThread();
        return m_pAttachEnv;
    }

    jobject
    getJavaPhysicsSpace() {
        return m_javaSpace;
    }
};

#endif
