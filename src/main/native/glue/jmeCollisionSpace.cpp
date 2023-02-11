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
#include "jmeClasses.h"
#include "jmeUserInfo.h"

/*
 * Author: Normen Hansen
 */

/*
 * During the broadphase, test whether the specified pair of proxies
 * needs collision detection.
 */
bool jmeFilterCallback::needBroadphaseCollision(btBroadphaseProxy *pProxy0,
        btBroadphaseProxy *pProxy1) const {
    /*
     * Test the Bullet collision-filter groups.
     */
    if ((pProxy0->m_collisionFilterGroup & pProxy1->m_collisionFilterMask) == 0x0
            && (pProxy1->m_collisionFilterGroup & pProxy0->m_collisionFilterMask) == 0x0) {
        return false;
    }
    /*
     * Test the ignore lists.
     */
    btCollisionObject * const pco0 = (btCollisionObject *) pProxy0->m_clientObject;
    btCollisionObject * const pco1 = (btCollisionObject *) pProxy1->m_clientObject;
    if (!pco0->checkCollideWith(pco1)) {
        return false;
    } else if (!pco1->checkCollideWith(pco0)) {
        return false;
    }
    /*
     * Test the Minie collision-filter groups.
     */
    jmeUserPointer const pUser0 = (jmeUserPointer) pco0->getUserPointer();
    jmeUserPointer const pUser1 = (jmeUserPointer) pco1->getUserPointer();
    if (pUser0 == NULL || pUser1 == NULL) { // TODO is this necessary?
        return true;
    } else if ((pUser0->m_group & pUser1->m_groups) == 0x0
            && (pUser1->m_group & pUser0->m_groups) == 0x0) {
        return false;
    }
    /*
     * As a final test, invoke the applicable collision-group listeners, if any.
     */
    jmeCollisionSpace * const pSpace = pUser0->m_jmeSpace;
    JNIEnv * const pEnv = pSpace->getEnvAndAttach();
    jobject javaPhysicsSpace = pEnv->NewLocalRef(pSpace->getJavaPhysicsSpace());
    EXCEPTION_CHK(pEnv, false);
    jobject javaCollisionObject0 = pEnv->NewLocalRef(pUser0->m_javaRef); // TODO is this necessary?
    EXCEPTION_CHK(pEnv, false);
    jobject javaCollisionObject1 = pEnv->NewLocalRef(pUser1->m_javaRef);
    EXCEPTION_CHK(pEnv, false);

    const jboolean result = pEnv->CallBooleanMethod(javaPhysicsSpace,
            jmeClasses::CollisionSpace_notifyCollisionGroupListeners,
            javaCollisionObject0, javaCollisionObject1);
    EXCEPTION_CHK(pEnv, false);

    pEnv->DeleteLocalRef(javaPhysicsSpace);
    EXCEPTION_CHK(pEnv, false);
    pEnv->DeleteLocalRef(javaCollisionObject0);
    EXCEPTION_CHK(pEnv, false);
    pEnv->DeleteLocalRef(javaCollisionObject1);

    return (bool) result;
}

jmeCollisionSpace::jmeCollisionSpace(JNIEnv *pEnv, jobject javaSpace) {
    this->m_pCreateEnv = pEnv;
    attachThread();

    m_javaSpace = pEnv->NewWeakGlobalRef(javaSpace);
    EXCEPTION_CHK(pEnv,);
}

void jmeCollisionSpace::attachThread() {
#ifdef ANDROID
    // doesn't match the Invocation API spec
    jint retCode = jmeClasses::vm->AttachCurrentThread(&m_pAttachEnv, NULL);
#else
    jint retCode = jmeClasses::vm->AttachCurrentThread((void **)&m_pAttachEnv, NULL);
#endif
    btAssert(retCode == JNI_OK);
}

btBroadphaseInterface * jmeCollisionSpace::createBroadphase(
        const btVector3 & min, const btVector3 & max, int broadphaseId) {
    btBroadphaseInterface * pBroadphase;
    switch (broadphaseId) {
        case 0:
            pBroadphase = new btSimpleBroadphase(); //dance009
            break;
        case 1:
            pBroadphase = new btAxisSweep3(min, max); //dance009
            break;
        case 2:
            pBroadphase = new bt32BitAxisSweep3(min, max); //dance009
            break;
        case 3:
            pBroadphase = new btDbvtBroadphase(); //dance009
            break;
        default:
            m_pCreateEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The broadphase type is out of range.");
            return 0;
    }

    btOverlappingPairCache * const
            pPairCache = pBroadphase->getOverlappingPairCache();
    pPairCache->setInternalGhostPairCallback(new btGhostPairCallback()); //dance036
    pPairCache->setOverlapFilterCallback(new jmeFilterCallback()); //dance011

    return pBroadphase;
}

void jmeCollisionSpace::createCollisionSpace(const btVector3& min,
        const btVector3& max, int broadphaseId) {
    btBroadphaseInterface * const
            pBroadphase = createBroadphase(min, max, broadphaseId);

    // Use the default collision dispatcher plus GImpact.
    btCollisionConfiguration * const
            pCollisionConfiguration = new btDefaultCollisionConfiguration(); //dance010
    btCollisionDispatcher * const
            pDispatcher = new btCollisionDispatcher(pCollisionConfiguration); //dance008
    btGImpactCollisionAlgorithm::registerAlgorithm(pDispatcher);

    // Create the collision world.
    m_pCollisionWorld = new btCollisionWorld(pDispatcher, pBroadphase,
            pCollisionConfiguration); //dance007
}

jmeCollisionSpace::~jmeCollisionSpace() {
    int numCollisionObjects = m_pCollisionWorld->getNumCollisionObjects();
    if (numCollisionObjects > 0) {
        /*
         * To avoid JME issue #1351, remove all collision objects.
         */
        btCollisionObjectArray&
                objects = m_pCollisionWorld->getCollisionObjectArray();
        for (int i = numCollisionObjects - 1; i >= 0; --i) {
            btCollisionObject *pObject = objects[i];
            m_pCollisionWorld->removeCollisionObject(pObject);

            jmeUserPointer const
                    pUser = (jmeUserPointer) pObject->getUserPointer();
            if (pUser != NULL) {
                delete pUser; //dance013
                pObject->setUserPointer(NULL);
            }
        }
    }
    btAssert(m_pCollisionWorld->getNumCollisionObjects() == 0);

    btBroadphaseInterface *pBroadphase = m_pCollisionWorld->getBroadphase();
    if (pBroadphase) {
        btOverlappingPairCache * const
                pPairCache = pBroadphase->getOverlappingPairCache();
        if (pPairCache) {
            btOverlappingPairCallback * const
                    pIGPCallback = pPairCache->getInternalGhostPairCallback();
            if (pIGPCallback) {
                delete pIGPCallback; //dance036
            }

            btOverlapFilterCallback * const
                    pOFCallback = pPairCache->getOverlapFilterCallback();
            if (pOFCallback) {
                delete pOFCallback; //dance011
            }
        }

        delete pBroadphase; //dance009
    }

    btCollisionDispatcher *pDispatcher =
            (btCollisionDispatcher *) m_pCollisionWorld->getDispatcher();
    if (pDispatcher) {
        btCollisionConfiguration *
                pCollisionConfiguration = pDispatcher->getCollisionConfiguration();
        if (pCollisionConfiguration) {
            delete pCollisionConfiguration; //dance010
        }
        delete pDispatcher; //dance008
    }

    delete m_pCollisionWorld; //dance007
}
