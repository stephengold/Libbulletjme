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

/*
 * Author: Normen Hansen
 */
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "com_jme3_bullet_objects_PhysicsGhostObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "jmeUserInfo.h"

class jmeGhostOverlapCallback : public btOverlapCallback {
    JNIEnv * m_env;
    jobject m_object;
    btCollisionObject * m_ghost;
public:

    jmeGhostOverlapCallback(JNIEnv *pEnv, jobject object,
            btCollisionObject *pGhost)
    : m_env(pEnv),
    m_object(object),
    m_ghost(pGhost) {
    }

    virtual ~jmeGhostOverlapCallback() {
    }

    virtual bool processOverlap(btBroadphasePair& pair) {
        btCollisionObject *pOther;
        if (pair.m_pProxy1->m_clientObject == m_ghost) {
            pOther = (btCollisionObject *) pair.m_pProxy0->m_clientObject;
        } else {
            pOther = (btCollisionObject *) pair.m_pProxy1->m_clientObject;
        }
        jmeUserPointer const pUser = (jmeUserInfo *) pOther->getUserPointer();
        jobject javaCollisionObject1 = m_env->NewLocalRef(pUser->m_javaRef);
        EXCEPTION_CHK(m_env, false);
        m_env->CallVoidMethod(m_object,
                jmeClasses::PhysicsGhostObject_addOverlappingObject,
                javaCollisionObject1);
        EXCEPTION_CHK(m_env, false);
        m_env->DeleteLocalRef(javaCollisionObject1);

        return false;
    }
};

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    createGhostObject
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_createGhostObject
(JNIEnv *pEnv, jclass) {
    jmeClasses::initJavaClasses(pEnv);

    btPairCachingGhostObject * const
            pGhost = new btPairCachingGhostObject(); //dance014
    return reinterpret_cast<jlong> (pGhost);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    getOverlappingCount
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_getOverlappingCount
(JNIEnv *pEnv, jclass, jlong ghostId) {
    const btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.", 0)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT, 0);

    return pGhost->getNumOverlappingObjects();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    getOverlappingObjects
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_getOverlappingObjects
(JNIEnv *pEnv, jobject object, jlong ghostId) {
    btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT,);

    btHashedOverlappingPairCache * const
            pCache = pGhost->getOverlappingPairCache();
    jmeGhostOverlapCallback callback(pEnv, object, pGhost);
    pCache->processAllOverlappingPairs(&callback, NULL);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    setGhostFlags
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_setGhostFlags
(JNIEnv *pEnv, jclass, jlong ghostId) {
    btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT,);

    pGhost->setCollisionFlags(pGhost->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    setPhysicsLocation
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_setPhysicsLocation
(JNIEnv *pEnv, jclass, jlong ghostId, jobject locationVector) {
    btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT,);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)

    jmeBulletUtil::convert(pEnv, locationVector,
            &pGhost->getWorldTransform().getOrigin());
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    setPhysicsLocationDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_setPhysicsLocationDp
  (JNIEnv *pEnv, jclass, jlong ghostId, jobject locationVector) {
    btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT,);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)

    jmeBulletUtil::convertDp(
            pEnv, locationVector, &pGhost->getWorldTransform().getOrigin());
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    setPhysicsRotation
 * Signature: (JLcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_setPhysicsRotation__JLcom_jme3_math_Matrix3f_2
(JNIEnv *pEnv, jclass, jlong ghostId, jobject matrix) {
    btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT,);

    NULL_CHK(pEnv, matrix, "The rotation matrix does not exist.",)

    jmeBulletUtil::convert(pEnv, matrix,
            &pGhost->getWorldTransform().getBasis());
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    setPhysicsRotation
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_setPhysicsRotation__JLcom_jme3_math_Quaternion_2
(JNIEnv *pEnv, jclass, jlong ghostId, jobject quaternion) {
    btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT,);

    NULL_CHK(pEnv, quaternion, "The quaternion does not exist.",)

    jmeBulletUtil::convertQuat(pEnv, quaternion,
            &pGhost->getWorldTransform().getBasis());
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    setPhysicsRotationDp
 * Signature: (JLcom/simsilica/mathd/Matrix3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_setPhysicsRotationDp__JLcom_simsilica_mathd_Matrix3d_2
(JNIEnv *pEnv, jclass, jlong ghostId, jobject matrix) {
    btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT,);

    NULL_CHK(pEnv, matrix, "The matrix does not exist.",)

    btMatrix3x3& out = pGhost->getWorldTransform().getBasis();
    jmeBulletUtil::convertDp(pEnv, matrix, &out);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsGhostObject
 * Method:    setPhysicsRotationDp
 * Signature: (JLcom/simsilica/mathd/Quatd;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsGhostObject_setPhysicsRotationDp__JLcom_simsilica_mathd_Quatd_2
(JNIEnv *pEnv, jclass, jlong ghostId, jobject quaternion) {
    btPairCachingGhostObject * const
            pGhost = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
    ASSERT_CHK(pEnv, pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT,);

    NULL_CHK(pEnv, quaternion, "The quaternion does not exist.",)

    btMatrix3x3& out = pGhost->getWorldTransform().getBasis();
    jmeBulletUtil::convertQuatDp(pEnv, quaternion, &out);
}
