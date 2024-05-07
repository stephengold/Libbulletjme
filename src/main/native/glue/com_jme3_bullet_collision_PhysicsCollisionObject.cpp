/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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
#include "com_jme3_bullet_collision_PhysicsCollisionObject.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "jmeUserInfo.h"

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    findInstance
 * Signature: (J)Lcom/jme3/bullet/collision/PhysicsCollisionObject;
 */
JNIEXPORT jobject JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_findInstance
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);
    const int internalType = pCollisionObject->getInternalType();
    ASSERT_CHK(pEnv, internalType > 0, 0);
    ASSERT_CHK(pEnv, internalType <= btCollisionObject::CO_FEATHERSTONE_LINK,
            0);

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    NULL_CHK(pEnv, pUser, "The jmeUserInfo does not exist.", 0);

    jobject result = pUser->m_javaRef;
    NULL_CHK(pEnv, result, "The PhysicsCollisionObject does not exist.", 0);

    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    attachCollisionShape
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_attachCollisionShape
(JNIEnv *pEnv, jclass, jlong pcoId, jlong shapeId) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",);

    btCollisionShape * const
            pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.",)

    pCollisionObject->setCollisionShape(pShape);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_finalizeNative
(JNIEnv *pEnv, jclass, jlong pcoId) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",);

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    if (pUser != NULL) {
        /*
         * To avoid JME issue #1351, remove it from any collision space it's in.
         */
        jmeCollisionSpace * const pSpace = pUser->m_jmeSpace;
        if (pSpace != NULL) {
            btCollisionWorld * const pWorld = pSpace->getCollisionWorld();
            if (pWorld != NULL) {
                const btCollisionObjectArray&
                        objects = pWorld->getCollisionObjectArray();
                int find = objects.findLinearSearch(pCollisionObject);
                if (find >= 0 && find < objects.size()) {
                    pWorld->removeCollisionObject(pCollisionObject);
                }
            }
        }
        pEnv->DeleteWeakGlobalRef(pUser->m_javaRef); //dance041
        EXCEPTION_CHK(pEnv,);

        delete pUser; //dance013
    }

    delete pCollisionObject; //dance014
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getInternalType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getInternalType
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jint result = pCollisionObject->getInternalType();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setActivationState
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setActivationState
(JNIEnv *pEnv, jclass, jlong pcoId, jint state) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->forceActivationState(state);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setCollisionFlags
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCollisionFlags
(JNIEnv *pEnv, jclass, jlong pcoId, jint desiredFlags) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setCollisionFlags(desiredFlags);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    activate
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_activate
(JNIEnv *pEnv, jclass, jlong pcoId, jboolean forceFlag) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->activate(forceFlag);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getActivationState
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getActivationState
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    int state = pCollisionObject->getActivationState();
    return (jint) state;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getAnisotropicFriction
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getAnisotropicFriction
(JNIEnv *pEnv, jclass, jlong pcoId, jobject storeVector) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)
    NULL_CHK(pEnv, storeVector, "The storeVector does not exist.",);

    const btVector3&
            frictionComponents = pCollisionObject->getAnisotropicFriction();
    jmeBulletUtil::convert(pEnv, &frictionComponents, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getBasis
 * Signature: (JLcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getBasis
(JNIEnv *pEnv, jclass, jlong pcoId, jobject storeMatrix) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)
    NULL_CHK(pEnv, storeMatrix, "The storeMatrix does not exist.",);

    const btMatrix3x3& basis = pCollisionObject->getWorldTransform().getBasis();
    jmeBulletUtil::convert(pEnv, &basis, storeMatrix);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getBasisDp
 * Signature: (JLcom/simsilica/mathd/Matrix3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getBasisDp
(JNIEnv *pEnv, jclass, jlong pcoId, jobject storeMatrix) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)
    NULL_CHK(pEnv, storeMatrix, "The storeMatrix does not exist.",);

    const btMatrix3x3& basis = pCollisionObject->getWorldTransform().getBasis();
    jmeBulletUtil::convertDp(pEnv, &basis, storeMatrix);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getCcdMotionThreshold
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCcdMotionThreshold
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.", 0)

    return pCollisionObject->getCcdMotionThreshold();
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getCcdSweptSphereRadius
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCcdSweptSphereRadius
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    return pCollisionObject->getCcdSweptSphereRadius();
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getCollideWithGroups
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCollideWithGroups
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    jint result = pUser->m_groups;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getCollisionFlags
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCollisionFlags
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jint result = pCollisionObject->getCollisionFlags();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getCollisionGroup
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCollisionGroup
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    jint result = pUser->m_group;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getCollisionSpace
 * Signature: (J)Lcom/jme3/bullet/CollisionSpace;
 */
JNIEXPORT jobject JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCollisionSpace
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    NULL_CHK(pEnv, pUser, "The jmeUserInfo does not exist.", 0);

    jmeCollisionSpace *pSpace = pUser->m_jmeSpace;
    NULL_CHK(pEnv, pSpace, "The jmeCollisionSpace does not exist.", 0);

    jobject javaSpace = pSpace->getJavaPhysicsSpace();
    return javaSpace;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getContactDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getContactDamping
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jfloat result = pCollisionObject->getContactDamping();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getContactProcessingThreshold
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getContactProcessingThreshold
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jfloat result = pCollisionObject->getContactProcessingThreshold();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getContactStiffness
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getContactStiffness
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jfloat result = pCollisionObject->getContactStiffness();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getDeactivationTime
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getDeactivationTime
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jfloat result = pCollisionObject->getDeactivationTime();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getFriction
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getFriction
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jfloat result = pCollisionObject->getFriction();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getLocation
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getLocation
(JNIEnv *pEnv, jclass, jlong pcoId, jobject storeVector) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)
    NULL_CHK(pEnv, storeVector, "The storeVector does not exist.",);

    const btVector3&
            location = pCollisionObject->getWorldTransform().getOrigin();
    jmeBulletUtil::convert(pEnv, &location, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getLocationDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getLocationDp
(JNIEnv *pEnv, jclass, jlong pcoId, jobject storeVectorDp) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)
    NULL_CHK(pEnv, storeVectorDp, "The storeVector does not exist.",);

    const btVector3&
            location = pCollisionObject->getWorldTransform().getOrigin();
    jmeBulletUtil::convertDp(pEnv, &location, storeVectorDp);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getNumObjectsWithoutCollision
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getNumObjectsWithoutCollision
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    int result = pCollisionObject->getNumObjectsWithoutCollision();
    return jint(result);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getObjectWithoutCollision
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getObjectWithoutCollision
(JNIEnv *pEnv, jclass, jlong pcoId, jint listIndex) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);
    ASSERT_CHK(pEnv, listIndex >= 0, 0);
    ASSERT_CHK(pEnv,
            listIndex < pCollisionObject->getNumObjectsWithoutCollision(), 0);

    int index = int(listIndex);
    const btCollisionObject *
            result = pCollisionObject->getObjectWithoutCollision(index);
    const int internalType = result->getInternalType();
    ASSERT_CHK(pEnv, internalType > 0, 0);
    ASSERT_CHK(pEnv, internalType <= btCollisionObject::CO_FEATHERSTONE_LINK,
            0);

    return reinterpret_cast<jlong> (result);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getOrientation
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getOrientation
(JNIEnv *pEnv, jclass, jlong pcoId, jobject storeQuat) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)
    NULL_CHK(pEnv, storeQuat, "The storeQuat does not exist.",);

    const btMatrix3x3& basis = pCollisionObject->getWorldTransform().getBasis();
    jmeBulletUtil::convertQuat(pEnv, &basis, storeQuat);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getOrientationDp
 * Signature: (JLcom/simsilica/mathd/Quatd;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getOrientationDp
(JNIEnv *pEnv, jclass, jlong pcoId, jobject storeQuatDp) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)
    NULL_CHK(pEnv, storeQuatDp, "The storeQuatDp does not exist.",);

    const btMatrix3x3& basis = pCollisionObject->getWorldTransform().getBasis();

    btQuaternion btq;
    basis.getRotation(btq);

    jmeBulletUtil::convertDp(pEnv, &btq, storeQuatDp);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getProxyFilterGroup
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getProxyFilterGroup
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    const btBroadphaseProxy * const
            pProxy = pCollisionObject->getBroadphaseHandle();
    NULL_CHK(pEnv, pProxy, "The btBroadphaseProxy does not exist.", 0);

    int result = pProxy->m_collisionFilterGroup;
    return jint(result);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getProxyFilterMask
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getProxyFilterMask
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    const btBroadphaseProxy * const
            pProxy = pCollisionObject->getBroadphaseHandle();
    NULL_CHK(pEnv, pProxy, "The btBroadphaseProxy does not exist.", 0);

    int result = pProxy->m_collisionFilterMask;
    return jint(result);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getRestitution
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getRestitution
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.", 0)

    jfloat result = pCollisionObject->getRestitution();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getRollingFriction
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getRollingFriction
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jfloat result = pCollisionObject->getRollingFriction();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getSpaceId
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getSpaceId
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject,
            "The btCollisionObject does not exist.", 0);

    jmeUserPointer const
            pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    jmeCollisionSpace *pSpace = pUser->m_jmeSpace;
    return reinterpret_cast<jlong> (pSpace);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getSpinningFriction
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getSpinningFriction
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            0);

    jfloat result = pCollisionObject->getSpinningFriction();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getUserIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getUserIndex
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.", 0);

    jint result = pCollisionObject->getUserIndex();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getUserIndex2
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getUserIndex2
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.", 0);

    jint result = pCollisionObject->getUserIndex2();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    getUserIndex3
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getUserIndex3
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.", 0);

    jint result = pCollisionObject->getUserIndex3();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    hasAnisotropicFriction
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_hasAnisotropicFriction
(JNIEnv *pEnv, jclass, jlong pcoId, jint mode) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            JNI_FALSE);

    jboolean result = pCollisionObject->hasAnisotropicFriction(mode);
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    hasBroadphaseProxy
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_hasBroadphaseProxy
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            JNI_FALSE);

    const btBroadphaseProxy * const
            pProxy = pCollisionObject->getBroadphaseHandle();

    bool result = (pProxy != NULL);
    return jboolean(result);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    initUserPointer
 * Signature: (JII)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_initUserPointer
(JNIEnv *pEnv, jobject object, jlong pcoId, jint group, jint groups) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    jmeUserPointer pUser = (jmeUserPointer) pCollisionObject->getUserPointer();
    if (pUser != NULL) {
        delete pUser; //dance013
    }

    pUser = new jmeUserInfo(); //dance013
    pCollisionObject->setUserPointer(pUser);
    pUser->m_group = group;
    pUser->m_groups = groups;
    pUser->m_jmeSpace = NULL;
    pUser->m_javaRef = pEnv->NewWeakGlobalRef(object); //dance041
    // no check for exceptions!
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    isActive
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_isActive
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            JNI_FALSE)

    return pCollisionObject->isActive();
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    isInWorld
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_isInWorld
(JNIEnv *pEnv, jclass, jlong pcoId) {
    const btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",
            JNI_FALSE);

    jboolean inWorld = (pCollisionObject->getBroadphaseHandle() != 0);
    return inWorld;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setAnisotropicFriction
 * Signature: (JLcom/jme3/math/Vector3f;I)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setAnisotropicFriction
(JNIEnv *pEnv, jclass, jlong pcoId, jobject frictionVector, jint mode) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    NULL_CHK(pEnv, frictionVector, "The friction vector does not exist.",)
    btVector3 tempVector;
    jmeBulletUtil::convert(pEnv, frictionVector, &tempVector);
    EXCEPTION_CHK(pEnv,);

    pCollisionObject->setAnisotropicFriction(tempVector, (int) mode);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setCcdMotionThreshold
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCcdMotionThreshold
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat threshold) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setCcdMotionThreshold(threshold);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setCcdSweptSphereRadius
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCcdSweptSphereRadius
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat radius) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setCcdSweptSphereRadius(radius);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setCollideWithGroups
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCollideWithGroups
(JNIEnv *pEnv, jclass, jlong pcoId, jint groups) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    jmeUserPointer const
            pUser = (jmeUserInfo *) pCollisionObject->getUserPointer();
    pUser->m_groups = groups;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setCollisionGroup
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCollisionGroup
(JNIEnv *pEnv, jclass, jlong pcoId, jint group) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    jmeUserPointer const
            pUser = (jmeUserInfo *) pCollisionObject->getUserPointer();
    pUser->m_group = group;
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setContactProcessingThreshold
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setContactProcessingThreshold
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat thresholdDistance) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setContactProcessingThreshold(thresholdDistance);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setContactStiffnessAndDamping
 * Signature: (JFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setContactStiffnessAndDamping
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat stiffness, jfloat damping) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setContactStiffnessAndDamping((btScalar) stiffness,
            (btScalar) damping);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setDeactivationTime
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setDeactivationTime
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat time) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setDeactivationTime((btScalar) time);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setFriction
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setFriction
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat friction) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setFriction((btScalar) friction);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setIgnoreCollisionCheck
 * Signature: (JJZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setIgnoreCollisionCheck
(JNIEnv *pEnv, jclass, jlong pco1Id, jlong pco2Id, jboolean setting) {
    btCollisionObject * const
            pObject1 = reinterpret_cast<btCollisionObject *> (pco1Id);
    NULL_CHK(pEnv, pObject1, "The btCollisionObject #1 does not exist.",);
    const int internalType1 = pObject1->getInternalType();
    ASSERT_CHK(pEnv, internalType1 > 0,);
    ASSERT_CHK(pEnv, internalType1 <= btCollisionObject::CO_FEATHERSTONE_LINK,);

    btCollisionObject * const
            pObject2 = reinterpret_cast<btCollisionObject *> (pco2Id);
    NULL_CHK(pEnv, pObject2, "The btCollisionObject #2 does not exist.",);
    const int internalType2 = pObject2->getInternalType();
    ASSERT_CHK(pEnv, internalType2 > 0,);
    ASSERT_CHK(pEnv, internalType2 <= btCollisionObject::CO_FEATHERSTONE_LINK,);

    bool ignoreCollisionCheck = bool(setting);
    pObject1->setIgnoreCollisionCheck(pObject2, ignoreCollisionCheck);
    pObject2->setIgnoreCollisionCheck(pObject1, ignoreCollisionCheck);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setLocationAndBasis
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setLocationAndBasis
(JNIEnv *pEnv, jclass, jlong pcoId, jobject locationVector,
        jobject basisMatrix) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)
    NULL_CHK(pEnv, locationVector, "The locationVector does not exist.",)
    NULL_CHK(pEnv, basisMatrix, "The basisMatrix does not exist.",)

    btTransform transform;
    jmeBulletUtil::convert(pEnv, locationVector, &transform.getOrigin());
    EXCEPTION_CHK(pEnv,);
    jmeBulletUtil::convert(pEnv, basisMatrix, &transform.getBasis());
    EXCEPTION_CHK(pEnv,);

    pCollisionObject->setWorldTransform(transform);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setRestitution
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setRestitution
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat restitution) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setRestitution((btScalar) restitution);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setRollingFriction
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setRollingFriction
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat friction) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setRollingFriction((btScalar) friction);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setSpinningFriction
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setSpinningFriction
(JNIEnv *pEnv, jclass, jlong pcoId, jfloat friction) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setSpinningFriction((btScalar) friction);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setUserIndex
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setUserIndex
(JNIEnv *pEnv, jclass, jlong pcoId, jint newIndex) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setUserIndex(newIndex);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setUserIndex2
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setUserIndex2
(JNIEnv *pEnv, jclass, jlong pcoId, jint newIndex) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setUserIndex2(newIndex);
}

/*
 * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
 * Method:    setUserIndex3
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setUserIndex3
(JNIEnv *pEnv, jclass, jlong pcoId, jint newIndex) {
    btCollisionObject * const
            pCollisionObject = reinterpret_cast<btCollisionObject *> (pcoId);
    NULL_CHK(pEnv, pCollisionObject, "The btCollisionObject does not exist.",)

    pCollisionObject->setUserIndex3(newIndex);
}
