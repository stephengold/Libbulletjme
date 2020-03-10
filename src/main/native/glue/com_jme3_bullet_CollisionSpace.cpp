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

/*
 * Author: Normen Hansen
 */
#include "com_jme3_bullet_CollisionSpace.h"
#include "jmeCollisionSpace.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    addCollisionObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_addCollisionObject
(JNIEnv *env, jobject, jlong spaceId, jlong pcoId) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHECK(pSpace, "The collision space does not exist.",)

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHECK(pCollisionObject, "The collision object does not exist.",)

    jmeUserPointer * const
            pUser = (jmeUserPointer *) pCollisionObject->getUserPointer();
    pUser->space = pSpace;

    pSpace->getCollisionWorld()->addCollisionObject(pCollisionObject);
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    createCollisionSpace
 * Signature: (FFFFFFI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_CollisionSpace_createCollisionSpace
(JNIEnv *env, jobject object, jfloat minX, jfloat minY, jfloat minZ,
        jfloat maxX, jfloat maxY, jfloat maxZ, jint broadphase) {
    jmeClasses::initJavaClasses(env);

    jmeCollisionSpace * const pSpace = new jmeCollisionSpace(env, object);
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
(JNIEnv *, jobject, jlong spaceId) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    if (pSpace != NULL) {
        delete pSpace;
    }
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    getNumCollisionObjects
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_CollisionSpace_getNumCollisionObjects
(JNIEnv *env, jobject, jlong spaceId) {
    const jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHECK(pSpace, "The collision space does not exist.", 0);

    int count = pSpace->getCollisionWorld()->getNumCollisionObjects();
    return (jint) count;
}

struct AllRayResultCallback : public btCollisionWorld::RayResultCallback {
    JNIEnv *m_pEnv;
    btVector3 m_rayFromWorld;
    btVector3 m_rayToWorld;
    jobject m_resultlist;

    AllRayResultCallback(const btVector3& rayFromWorld,
            const btVector3& rayToWorld, jobject resultlist)
    : m_rayFromWorld(rayFromWorld),
    m_rayToWorld(rayToWorld),
    m_resultlist(resultlist) {
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

        jmeBulletUtil::addRayTestResult(m_pEnv, m_resultlist,
                &m_hitNormalWorld, rayResult.m_hitFraction,
                rayResult.m_collisionObject, partIndex, triangleIndex);

        return 1;
    }
};

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    rayTest_1native
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;JLjava/util/List;I)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_rayTest_1native
(JNIEnv *env, jobject, jobject from, jobject to, jlong spaceId,
        jobject resultlist, jint flags) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHECK(pSpace, "The collision space does not exist.",);

    NULL_CHECK(to, "The to vector does not exist.",);
    btVector3 native_to;
    jmeBulletUtil::convert(env, to, &native_to);

    NULL_CHECK(from, "The from vector does not exist.",);
    btVector3 native_from;
    jmeBulletUtil::convert(env, from, &native_from);

    AllRayResultCallback resultCallback(native_from, native_to, resultlist);
    resultCallback.m_pEnv = env;
    resultCallback.m_flags = flags;

    pSpace->getCollisionWorld()->rayTest(native_from, native_to,
            resultCallback);
}

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    removeCollisionObject
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_removeCollisionObject
(JNIEnv *env, jobject, jlong spaceId, jlong pcoId) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHECK(pSpace, "The collision space does not exist.",)

    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHECK(pCollisionObject, "The collision object does not exist.",)

    pSpace->getCollisionWorld()->removeCollisionObject(pCollisionObject);

    jmeUserPointer * const
            pUser = (jmeUserPointer *) pCollisionObject->getUserPointer();
    pUser->space = NULL;
}

struct AllConvexResultCallback : public btCollisionWorld::ConvexResultCallback {
    JNIEnv *m_pEnv;
    btTransform m_convexFromWorld;
    btTransform m_convexToWorld;
    jobject m_resultlist;

    AllConvexResultCallback(const btTransform& convexFromWorld,
            const btTransform & convexToWorld, jobject resultlist)
    : m_convexFromWorld(convexFromWorld), m_convexToWorld(convexToWorld),
    m_resultlist(resultlist) {
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

        jmeBulletUtil::addSweepTestResult(m_pEnv, m_resultlist,
                &m_hitNormalWorld, convexResult.m_hitFraction,
                convexResult.m_hitCollisionObject, partIndex,
                triangleIndex);

        return 1;
    }
};

/*
 * Class:     com_jme3_bullet_CollisionSpace
 * Method:    sweepTest_native
 * Signature: (JLcom/jme3/math/Transform;Lcom/jme3/math/Transform;JLjava/util/List;F)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionSpace_sweepTest_1native
(JNIEnv *env, jobject, jlong shapeId, jobject from, jobject to, jlong spaceId,
        jobject resultlist, jfloat allowedCcdPenetration) {
    jmeCollisionSpace * const
            pSpace = reinterpret_cast<jmeCollisionSpace *> (spaceId);
    NULL_CHECK(pSpace, "The collision space does not exist.",)

    btCollisionShape * const
            pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHECK(pShape, "The shape does not exist.",);
    if (!pShape->isConvex()) {
        env->ThrowNew(jmeClasses::IllegalArgumentException,
                "The btCollisionShape isn't convex.");
        return;
    }

    const btConvexShape * const pConvexShape
            = reinterpret_cast<btConvexShape *> (shapeId);

    btVector3 scale; // scales are ignored
    btTransform native_to;
    jmeBulletUtil::convert(env, to, &native_to, &scale);

    btTransform native_from;
    jmeBulletUtil::convert(env, from, &native_from, &scale);

    AllConvexResultCallback resultCallback(native_from, native_to,
            resultlist);
    resultCallback.m_pEnv = env;

    btScalar allowed_penetration = (btScalar) allowedCcdPenetration;
    pSpace->getCollisionWorld()->convexSweepTest(pConvexShape, native_from,
            native_to, resultCallback, allowed_penetration);
}
