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
#include "jmeCollisionSpace.h"
#include "jmeBulletUtil.h"
#include "jmeUserInfo.h"

/*
 * Author: Normen Hansen
 */

/*
 * Test whether the specified pair of proxies needs collision.
 */
bool jmeFilterCallback::needBroadphaseCollision(btBroadphaseProxy *pProxy0,
        btBroadphaseProxy *pProxy1) const {
    bool collides = (pProxy0->m_collisionFilterGroup & pProxy1->m_collisionFilterMask) != 0
            || (pProxy1->m_collisionFilterGroup & pProxy0->m_collisionFilterMask) != 0;
    if (collides) {
        btCollisionObject * const pco0 = (btCollisionObject *) pProxy0->m_clientObject;
        btCollisionObject * const pco1 = (btCollisionObject *) pProxy1->m_clientObject;
        jmeUserPointer const pUser0 = (jmeUserPointer) pco0->getUserPointer();
        jmeUserPointer const pUser1 = (jmeUserPointer) pco1->getUserPointer();
        if (pUser0 != NULL && pUser1 != NULL) {
            collides = (pUser0->m_group & pUser1->m_groups) != 0
                    || (pUser1->m_group & pUser0->m_groups) != 0;

            if (collides) {
                jmeCollisionSpace * const pSpace = pUser0->m_jmeSpace;
                JNIEnv * const pEnv = pSpace->getEnv();
                jobject javaPhysicsSpace = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
                jobject javaCollisionObject0 = pEnv->NewLocalRef(pUser0->m_javaRef);
                jobject javaCollisionObject1 = pEnv->NewLocalRef(pUser1->m_javaRef);

                const jboolean notifyResult = pEnv->CallBooleanMethod(
                        javaPhysicsSpace,
                        jmeClasses::CollisionSpace_notifyCollisionGroupListeners,
                        javaCollisionObject0, javaCollisionObject1);

                pEnv->DeleteLocalRef(javaPhysicsSpace);
                pEnv->DeleteLocalRef(javaCollisionObject0);
                pEnv->DeleteLocalRef(javaCollisionObject1);

                if (pEnv->ExceptionCheck()) {
                    pEnv->Throw(pEnv->ExceptionOccurred());
                    return 0;
                }

                collides = (bool) notifyResult;
            }
        }
    }
    return collides;
}

jmeCollisionSpace::jmeCollisionSpace(JNIEnv *pEnv, jobject javaSpace) {
    this->pEnv = pEnv;

    m_javaSpace = pEnv->NewWeakGlobalRef(javaSpace);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    pEnv->GetJavaVM(&vm);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

void jmeCollisionSpace::attachThread() {
#ifdef ANDROID
    vm->AttachCurrentThread((JNIEnv **) & pEnv, NULL);
#elif defined (JNI_VERSION_1_2)
    vm->AttachCurrentThread((void **) &pEnv, NULL);
#else
    vm->AttachCurrentThread(&pEnv, NULL);
#endif
}

btBroadphaseInterface * jmeCollisionSpace::createBroadphase(
        const btVector3 & min, const btVector3 & max, int broadphaseId) {
    btBroadphaseInterface * pBroadphase;
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
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The broadphase type is out of range.");
    }

    btOverlappingPairCache * const
            pPairCache = pBroadphase->getOverlappingPairCache();
    pPairCache->setInternalGhostPairCallback(new btGhostPairCallback());
    pPairCache->setOverlapFilterCallback(new jmeFilterCallback());

    return pBroadphase;
}

void jmeCollisionSpace::createCollisionSpace(const btVector3& min,
        const btVector3& max, int broadphaseId) {
    btBroadphaseInterface * const
            pBroadphase = createBroadphase(min, max, broadphaseId);

    // Use the default collision dispatcher plus GImpact.
    btCollisionConfiguration * const
            pCollisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * const
            pDispatcher = new btCollisionDispatcher(pCollisionConfiguration);
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // Create the collision world.
    m_collisionWorld = new btCollisionWorld(pDispatcher, pBroadphase,
            pCollisionConfiguration);
}
