/*
 * Copyright (c) 2009-2021 jMonkeyEngine
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
 * Author: Normen Hansen
 */
#include "com_jme3_bullet_CollisionSpace.h"
#include "jmeCollisionSpace.h"
#include "jmeBulletUtil.h"
#include "jmeUserInfo.h"

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    addCollisionObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_addCollisionObject
(JNIEnv *pEnv, jclass, jlong spaceId, jlong pcoId) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.",)
    btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.",);

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",);
    const int internalType = pCollisionObject->getInternalType();
    ASSERT_CHK(pEnv, internalType > 0,);
    ASSERT_CHK(pEnv, internalType <= btCollisionObject::CO_FEATHERSTONE_LINK,);

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    pUser->m_jmeSpace = pSpace;

    pWorld->addCollisionObject(pCollisionObject);
}

/*
 * Callback used in contact tests.
 */
struct JmeContactResultCallback
: public btCollisionWorld::ContactResultCallback {
    jint m_invocationCount;
    JNIEnv *m_pEnv;
    jobject m_listener;

    JmeContactResultCallback(JNIEnv *pEnv, jobject listener)
    : m_invocationCount(0), m_pEnv(pEnv), m_listener(listener) {
    }

    btScalar addSingleResult(btManifoldPoint& manifoldPoint,
            const btCollisionObjectWrapper* pWrap0, int part0, int tri0,
            const btCollisionObjectWrapper* pWrap1, int part1, int tri1) {

        if (manifoldPoint.getDistance() > 0) { // no penetration, so ignore it
            return (btScalar) 1;
        }

        const btCollisionObject * const pColObj0 = pWrap0->m_collisionObject;
        jmeUserPointer pUser0 = (jmeUserPointer) pColObj0->getUserPointer();

        const btCollisionObject * const pColObj1 = pWrap1->m_collisionObject;
        jmeUserPointer pUser1 = (jmeUserPointer) pColObj1->getUserPointer();

        bool collides = (pUser0->m_group & pUser1->m_groups) != 0x0
                || (pUser1->m_group & pUser0->m_groups) != 0x0;
        if (!collides) {
            return (btScalar) 1;
        }

        ++m_invocationCount;
        if (m_listener == NULL) {
            return (btScalar) 1;
        }

        jobject const pcoA = pUser0->m_javaRef;
        jobject const pcoB = pUser1->m_javaRef;
        jlong const manifoldId = reinterpret_cast<jlong> (&manifoldPoint);
        jobject const eventObject = m_pEnv->NewObject(
                jmeClasses::PhysicsCollisionEvent_Class,
                jmeClasses::PhysicsCollisionEvent_init, pcoA, pcoB, manifoldId);
        EXCEPTION_CHK(m_pEnv, btScalar(1));

        m_pEnv->CallVoidMethod(m_listener,
                jmeClasses::PhysicsCollisionListener_method, eventObject);

        return (btScalar) 1;
    }
};

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    contactTest
 * Signature: (JJLcom/jme3/bullet/collision/PhysicsCollisionListener;)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_CollisionSpace_contactTest
(JNIEnv *pEnv, jclass, jlong spaceId, jlong pcoId, jobject listener) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.", 0);
    btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.", 0);

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.", 0);
    const int internalType = pCollisionObject->getInternalType();
    ASSERT_CHK(pEnv, internalType > 0, 0);
    ASSERT_CHK(pEnv, internalType <= btCollisionObject::CO_FEATHERSTONE_LINK, 0);

    JmeContactResultCallback callback(pEnv, listener);
    pWorld->contactTest(pCollisionObject, callback);

    jint result = callback.m_invocationCount;
    return result;
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    createCollisionSpace
 * Signature: (FFFFFFI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_CollisionSpace_createCollisionSpace
(JNIEnv *pEnv, jobject object, jfloat minX, jfloat minY, jfloat minZ,
        jfloat maxX, jfloat maxY, jfloat maxZ, jint broadphase) {
    jmeClasses::initJavaClasses(pEnv);

    jmeCollisionSpace * const
            pSpace = new jmeCollisionSpace(pEnv, object); //dance003
    btVector3 min(minX, minY, minZ);
    btVector3 max(maxX, maxY, maxZ);
    pSpace->createCollisionSpace(min, max, (int) broadphase);

    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_finalizeNative
(JNIEnv *, jclass, jlong spaceId) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    if (pSpace != NULL) {
        delete pSpace; //dance003
    }
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    getDeterministicOverlappingPairs
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_CollisionSpace_getDeterministicOverlappingPairs
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.", JNI_FALSE);
    const btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.", JNI_FALSE);

    const btDispatcherInfo& dispatchInfo = pWorld->getDispatchInfo();
    bool result = dispatchInfo.m_deterministicOverlappingPairs;

    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    getJniEnvId
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_CollisionSpace_getJniEnvId
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.", 0);

    const JNIEnv * result = pSpace->getCreateEnv();
    return reinterpret_cast<jlong> (result);
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    getNumCollisionObjects
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_CollisionSpace_getNumCollisionObjects
(JNIEnv *pEnv, jclass, jlong spaceId) {
    const jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.", 0);
    const btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.", 0);

    int count = pWorld->getNumCollisionObjects();
    return (jint) count;
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    hasClosest
 * Signature: (JII)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_CollisionSpace_hasClosest
(JNIEnv *pEnv, jclass, jlong spaceId, jint shape0Type, jint shape1Type) {
    const jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.", JNI_FALSE);
    const btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.", JNI_FALSE);
    const btCollisionDispatcher * const pDispatcher
            = (const btCollisionDispatcher *) pWorld->getDispatcher();
    NULL_CHK(pEnv, pDispatcher, "The dispatcher does not exist.", JNI_FALSE);
    ASSERT_CHK(pEnv, shape0Type >= 0, JNI_FALSE);
    ASSERT_CHK(pEnv, shape0Type < MAX_BROADPHASE_COLLISION_TYPES, JNI_FALSE);
    ASSERT_CHK(pEnv, shape1Type >= 0, JNI_FALSE);
    ASSERT_CHK(pEnv, shape1Type < MAX_BROADPHASE_COLLISION_TYPES, JNI_FALSE);

    bool result = pDispatcher->hasClosestFunction(shape0Type, shape1Type);
    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    hasContact
 * Signature: (JII)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_CollisionSpace_hasContact
(JNIEnv *pEnv, jclass, jlong spaceId, jint shape0Type, jint shape1Type) {
    const jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.", JNI_FALSE);
    const btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.", JNI_FALSE);
    const btCollisionDispatcher * const pDispatcher
            = (const btCollisionDispatcher *) pWorld->getDispatcher();
    NULL_CHK(pEnv, pDispatcher, "The dispatcher does not exist.", JNI_FALSE);
    ASSERT_CHK(pEnv, shape0Type >= 0, JNI_FALSE);
    ASSERT_CHK(pEnv, shape0Type < MAX_BROADPHASE_COLLISION_TYPES, JNI_FALSE);
    ASSERT_CHK(pEnv, shape1Type >= 0, JNI_FALSE);
    ASSERT_CHK(pEnv, shape1Type < MAX_BROADPHASE_COLLISION_TYPES, JNI_FALSE);

    bool result = pDispatcher->hasContactFunction(shape0Type, shape1Type);
    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    isForceUpdateAllAabbs
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_CollisionSpace_isForceUpdateAllAabbs
(JNIEnv *pEnv, jclass, jlong spaceId) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.", false);
    btCollisionWorld *pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.", false)

    return pWorld->getForceUpdateAllAabbs();
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    pairTest
 * Signature: (JJJLcom/jme3/bullet/collision/PhysicsCollisionListener;)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_CollisionSpace_pairTest
(JNIEnv *pEnv, jclass, jlong spaceId, jlong aId, jlong bId, jobject listener) {
jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.", 0);
    btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.", 0);

    btCollisionObject * const
            pObjectA = reinterpret_cast<btCollisionObject *> (aId);
    NULL_CHK(pEnv, pObjectA, "Collision object A does not exist.", 0);
    const int aType = pObjectA->getInternalType();
    ASSERT_CHK(pEnv, aType > 0, 0);
    ASSERT_CHK(pEnv, aType <= btCollisionObject::CO_FEATHERSTONE_LINK, 0);

    btCollisionObject * const
            pObjectB = reinterpret_cast<btCollisionObject *> (bId);
    NULL_CHK(pEnv, pObjectB, "Collision object B does not exist.", 0);
    const int bType = pObjectB->getInternalType();
    ASSERT_CHK(pEnv, bType > 0, 0);
    ASSERT_CHK(pEnv, bType <= btCollisionObject::CO_FEATHERSTONE_LINK, 0);

    JmeContactResultCallback callback(pEnv, listener);
    pWorld->contactPairTest(pObjectA, pObjectB, callback);

    jint result = callback.m_invocationCount;
    return result;
}

/*
 * Callback used in raycasts.
 */
struct JmeRayResultCallback : public btCollisionWorld::RayResultCallback {
    btVector3 m_rayFromWorld;
    btVector3 m_rayToWorld;
    JNIEnv *m_pEnv;
    jobject m_resultList;

    JmeRayResultCallback(JNIEnv *pEnv, const btVector3& rayFromWorld,
            const btVector3& rayToWorld, jobject resultList, int flags)
    : m_rayFromWorld(rayFromWorld), m_rayToWorld(rayToWorld), m_pEnv(pEnv),
    m_resultList(resultList) {
        m_flags = flags;
    }

    btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult,
            bool normalInWorldSpace) {
        btVector3 m_hitNormalWorld;
        if (normalInWorldSpace) {
            m_hitNormalWorld = rayResult.m_hitNormalLocal;
        } else {
            m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()
                    * rayResult.m_hitNormalLocal;
        }
        /*
         * If the shape of the hit collision object is compound or concave,
         *  LocalShapeInfo indicates where it was hit.
         */
        int partIndex = -1;
        int triangleIndex = -1;
        btCollisionWorld::LocalShapeInfo *pLsi = rayResult.m_localShapeInfo;
        if (pLsi != NULL) {
            partIndex = pLsi->m_shapePart;
            triangleIndex = pLsi->m_triangleIndex;
        }

        jmeBulletUtil::addRayTestResult(m_pEnv, m_resultList,
                &m_hitNormalWorld, rayResult.m_hitFraction,
                rayResult.m_collisionObject, partIndex, triangleIndex);

        return (btScalar) 1;
    }
};

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    rayTestNative
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;JLjava/util/List;I)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_rayTestNative
(JNIEnv *pEnv, jclass, jobject from, jobject to, jlong spaceId,
        jobject resultList, jint flags) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.",);
    btCollisionWorld *pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.",)

    NULL_CHK(pEnv, to, "The to vector does not exist.",);
    btVector3 native_to;
    jmeBulletUtil::convert(pEnv, to, &native_to);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, from, "The from vector does not exist.",);
    btVector3 native_from;
    jmeBulletUtil::convert(pEnv, from, &native_from);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, resultList, "The result list does not exist.",);

    JmeRayResultCallback
    resultCallback(pEnv, native_from, native_to, resultList, flags);

    pWorld->rayTest(native_from, native_to, resultCallback);
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    rayTestNativeDp
 * Signature: (Lcom/simsilica/mathd/Vec3d;Lcom/simsilica/mathd/Vec3d;JLjava/util/List;I)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_rayTestNativeDp
(JNIEnv *pEnv, jclass, jobject from, jobject to, jlong spaceId,
        jobject resultList, jint flags) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.",);
    btCollisionWorld *pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.",)

    NULL_CHK(pEnv, to, "The to vector does not exist.",);
    btVector3 native_to;
    jmeBulletUtil::convertDp(pEnv, to, &native_to);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, from, "The from vector does not exist.",);
    btVector3 native_from;
    jmeBulletUtil::convertDp(pEnv, from, &native_from);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, resultList, "The result list does not exist.",);

    JmeRayResultCallback
    resultCallback(pEnv, native_from, native_to, resultList, flags);

    pWorld->rayTest(native_from, native_to, resultCallback);
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    removeCollisionObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_removeCollisionObject
(JNIEnv *pEnv, jclass, jlong spaceId, jlong pcoId) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.",)
    btCollisionWorld *pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.",);

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The collision object does not exist.",);
    const int internalType = pCollisionObject->getInternalType();
    ASSERT_CHK(pEnv, internalType > 0,);
    ASSERT_CHK(pEnv, internalType <= btCollisionObject::CO_FEATHERSTONE_LINK,);

    pWorld->removeCollisionObject(pCollisionObject);

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    pUser->m_jmeSpace = NULL;
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    setDeterministicOverlappingPairs
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_setDeterministicOverlappingPairs
(JNIEnv *pEnv, jclass, jlong spaceId, jboolean desiredSetting) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.",);
    btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.",);

    btDispatcherInfo& dispatchInfo = pWorld->getDispatchInfo();
    dispatchInfo.m_deterministicOverlappingPairs = (bool)desiredSetting;
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    setForceUpdateAllAabbs
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_setForceUpdateAllAabbs
(JNIEnv *pEnv, jclass, jlong spaceId, jboolean forceUpdate) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.",);
    btCollisionWorld *pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.",)

    pWorld->setForceUpdateAllAabbs(forceUpdate);
}

/*
 * Callback used in (convex) sweep tests.
 */
struct JmeConvexResultCallback : public btCollisionWorld::ConvexResultCallback {
    btTransform m_convexFromWorld;
    btTransform m_convexToWorld;
    JNIEnv *m_pEnv;
    jobject m_resultList;

    JmeConvexResultCallback(JNIEnv *pEnv, const btTransform& convexFromWorld,
            const btTransform & convexToWorld, jobject resultList)
    : m_convexFromWorld(convexFromWorld), m_convexToWorld(convexToWorld),
    m_pEnv(pEnv), m_resultList(resultList) {
    }

    btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult,
            bool normalInWorldSpace) {
        btVector3 m_hitNormalWorld;
        if (normalInWorldSpace) {
            m_hitNormalWorld = convexResult.m_hitNormalLocal;
        } else {
            m_hitNormalWorld
                    = convexResult.m_hitCollisionObject->getWorldTransform().getBasis()
                    * convexResult.m_hitNormalLocal;
        }
        /*
         * If the shape of the hit collision object is compound or concave,
         *  LocalShapeInfo indicates where it was hit.
         */
        int partIndex = -1;
        int triangleIndex = -1;
        btCollisionWorld::LocalShapeInfo *pLsi
                = convexResult.m_localShapeInfo;
        if (pLsi != NULL) {
            partIndex = pLsi->m_shapePart;
            triangleIndex = pLsi->m_triangleIndex;
        }

        jmeBulletUtil::addSweepTestResult(m_pEnv, m_resultList,
                &m_hitNormalWorld, convexResult.m_hitFraction,
                convexResult.m_hitCollisionObject, partIndex,
                triangleIndex);

        return (btScalar) 1;
    }
};

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    sweepTestNative
 * Signature: (JLcom/jme3/math/Transform;Lcom/jme3/math/Transform;JLjava/util/List;F)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_sweepTestNative
(JNIEnv *pEnv, jclass, jlong shapeId, jobject from, jobject to, jlong spaceId,
        jobject resultList, jfloat allowedCcdPenetration) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The collision space does not exist.",)
    btCollisionWorld *pWorld = pSpace->getCollisionWorld();
    NULL_CHK(pEnv, pWorld, "The collision world does not exist.",);

    btCollisionShape * const
            pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The shape does not exist.",);
    if (!pShape->isConvex()) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "The btCollisionShape isn't convex.");
        return;
    }
    const btConvexShape * const
            pConvexShape = reinterpret_cast<btConvexShape *> (shapeId);

    NULL_CHK(pEnv, resultList, "The result list does not exist.",);

    btVector3 scale; // scales are ignored
    btTransform native_to;
    jmeBulletUtil::convert(pEnv, to, &native_to, &scale);
    EXCEPTION_CHK(pEnv,);

    btTransform native_from;
    jmeBulletUtil::convert(pEnv, from, &native_from, &scale);
    EXCEPTION_CHK(pEnv,);

    JmeConvexResultCallback
    resultCallback(pEnv, native_from, native_to, resultList);

    btScalar allowed_penetration = (btScalar) allowedCcdPenetration;
    pWorld->convexSweepTest(pConvexShape, native_from, native_to,
            resultCallback, allowed_penetration);
}
