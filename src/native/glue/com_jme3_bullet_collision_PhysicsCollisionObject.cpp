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

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    activate
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_activate
    (JNIEnv * env, jobject object, jlong objectId, jboolean forceFlag) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->activate(forceFlag);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    attachCollisionShape
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_attachCollisionShape
    (JNIEnv * env, jobject object, jlong objectId, jlong shapeId) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        btCollisionShape* collisionShape = reinterpret_cast<btCollisionShape*> (shapeId);
        NULL_CHECK(collisionShape, "The btCollisionShape does not exist.",)

        collisionObject->setCollisionShape(collisionShape);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    finalizeNative
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_finalizeNative
    (JNIEnv * env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        if (collisionObject -> getUserPointer() != NULL) {
            jmeUserPointer *userPointer = (jmeUserPointer*) collisionObject->getUserPointer();
            delete userPointer;
        }
        delete collisionObject;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getAnisotropicFriction
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getAnisotropicFriction
    (JNIEnv *env, jobject object, jlong objectId, jobject storeVector) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)
        NULL_CHECK(storeVector, "The storeVector does not exist.",);

        const btVector3& frictionComponents
                = collisionObject->getAnisotropicFriction();
        jmeBulletUtil::convert(env, &frictionComponents, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getBasis
     * Signature: (JLcom/jme3/math/Matrix3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getBasis
    (JNIEnv *env, jobject object, jlong objectId, jobject storeMatrix) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)
        NULL_CHECK(storeMatrix, "The storeMatrix does not exist.",)

        btMatrix3x3& basis = collisionObject->getWorldTransform().getBasis();
        jmeBulletUtil::convert(env, &basis, storeMatrix);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getCcdMotionThreshold
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCcdMotionThreshold
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        return collisionObject->getCcdMotionThreshold();
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getCcdSweptSphereRadius
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCcdSweptSphereRadius
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        return collisionObject->getCcdSweptSphereRadius();
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getCollisionFlags
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getCollisionFlags
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jint result = collisionObject->getCollisionFlags();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getContactDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getContactDamping
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jfloat result = collisionObject->getContactDamping();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getContactProcessingThreshold
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getContactProcessingThreshold
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jfloat result = collisionObject->getContactProcessingThreshold();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getContactStiffness
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getContactStiffness
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jfloat result = collisionObject->getContactStiffness();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getDeactivationTime
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getDeactivationTime
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jfloat result = collisionObject->getDeactivationTime();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getFriction
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getFriction
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jfloat result = collisionObject->getFriction();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getInternalType
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getInternalType
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject *pCollisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(pCollisionObject, "The btCollisionObject does not exist.", 0)

        jint result = pCollisionObject->getInternalType();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getLocation
    (JNIEnv *env, jobject object, jlong objectId, jobject storeVector) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)
        NULL_CHECK(storeVector, "The storeVector does not exist.",);

        const btVector3& location = collisionObject->getWorldTransform().getOrigin();
        jmeBulletUtil::convert(env, &location, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getOrientation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getOrientation
    (JNIEnv *env, jobject object, jlong objectId, jobject storeQuat) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)
        NULL_CHECK(storeQuat, "The storeQuat does not exist.",)

        btMatrix3x3& basis = collisionObject->getWorldTransform().getBasis();
        jmeBulletUtil::convertQuat(env, &basis, storeQuat);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getRestitution
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getRestitution
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jfloat result = collisionObject->getRestitution();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getRollingFriction
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getRollingFriction
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jfloat result = collisionObject->getRollingFriction();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    getSpinningFriction
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_getSpinningFriction
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", 0)

        jfloat result = collisionObject->getSpinningFriction();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    hasAnisotropicFriction
     * Signature: (JI)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_hasAnisotropicFriction
    (JNIEnv *env, jobject object, jlong objectId, jint mode) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", false)

        jboolean result = collisionObject->hasAnisotropicFriction(mode);
        return result;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    initUserPointer
     * Signature: (JII)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_initUserPointer
    (JNIEnv *env, jobject object, jlong objectId, jint group, jint groups) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        jmeUserPointer *userPointer = (jmeUserPointer*) collisionObject->getUserPointer();
        if (userPointer != NULL) {
            //            delete userPointer;
        }
        userPointer = new jmeUserPointer();
        userPointer -> javaCollisionObject = env->NewWeakGlobalRef(object);
        userPointer -> group = group;
        userPointer -> groups = groups;
        userPointer -> space = NULL;
        collisionObject -> setUserPointer(userPointer);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    isActive
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_isActive
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.", false)

        return collisionObject->isActive();
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    isInWorld
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_isInWorld
    (JNIEnv *env, jobject object, jlong objectId) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",
                false);

        jboolean inWorld = (collisionObject->getBroadphaseHandle() != 0);

        return inWorld;
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setAnisotropicFriction
     * Signature: (JLcom/jme3/math/Vector3f;I)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setAnisotropicFriction
    (JNIEnv *env, jobject object, jlong objectId, jobject components, jint mode) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)
        NULL_CHECK(components, "The components vector does not exist.",)

        btVector3 tempVector;
        jmeBulletUtil::convert(env, components, &tempVector);
        collisionObject->setAnisotropicFriction(tempVector, mode);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setCcdMotionThreshold
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCcdMotionThreshold
    (JNIEnv *env, jobject object, jlong objectId, jfloat threshold) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setCcdMotionThreshold(threshold);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setCcdSweptSphereRadius
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCcdSweptSphereRadius
    (JNIEnv *env, jobject object, jlong objectId, jfloat radius) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setCcdSweptSphereRadius(radius);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setCollisionFlags
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCollisionFlags
    (JNIEnv *env, jobject object, jlong objectId, jint desiredFlags) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setCollisionFlags(desiredFlags);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setCollideWithGroups
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCollideWithGroups
    (JNIEnv *env, jobject object, jlong objectId, jint groups) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        jmeUserPointer *userPointer = (jmeUserPointer*) collisionObject->getUserPointer();
        if (userPointer != NULL) {
            userPointer -> groups = groups;
        }
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setCollisionGroup
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setCollisionGroup
    (JNIEnv *env, jobject object, jlong objectId, jint group) {
        btCollisionObject* collisionObject = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        jmeUserPointer *userPointer = (jmeUserPointer*) collisionObject->getUserPointer();
        if (userPointer != NULL) {
            userPointer -> group = group;
        }
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setContactProcessingThreshold
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setContactProcessingThreshold
    (JNIEnv *env, jobject object, jlong objectId, jfloat thresholdDistance) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setContactProcessingThreshold(thresholdDistance);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setContactStiffnessAndDamping
     * Signature: (JFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setContactStiffnessAndDamping
    (JNIEnv *env, jobject object, jlong objectId, jfloat stiffness, jfloat damping) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setContactStiffnessAndDamping(stiffness, damping);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setDeactivationTime
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setDeactivationTime
    (JNIEnv *env, jobject object, jlong objectId, jfloat time) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setDeactivationTime(time);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setFriction
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setFriction
    (JNIEnv *env, jobject object, jlong objectId, jfloat friction) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setFriction(friction);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setLocationAndBasis
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setLocationAndBasis
    (JNIEnv *env, jobject object, jlong objectId, jobject locationVector, jobject basisMatrix) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)
        NULL_CHECK(locationVector, "The locationVector does not exist.",)
        NULL_CHECK(basisMatrix, "The basisMatrix does not exist.",)

        btTransform transform;
        jmeBulletUtil::convert(env, locationVector, &transform.getOrigin());
        jmeBulletUtil::convert(env, basisMatrix, &transform.getBasis());

        collisionObject->setWorldTransform(transform);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setRestitution
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setRestitution
    (JNIEnv *env, jobject object, jlong objectId, jfloat restitution) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setRestitution(restitution);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setRollingFriction
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setRollingFriction
    (JNIEnv *env, jobject object, jlong objectId, jfloat friction) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setRollingFriction(friction);
    }

    /*
     * Class:     com_jme3_bullet_collision_PhysicsCollisionObject
     * Method:    setSpinningFriction
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_PhysicsCollisionObject_setSpinningFriction
    (JNIEnv *env, jobject object, jlong objectId, jfloat friction) {
        btCollisionObject* collisionObject
                = reinterpret_cast<btCollisionObject*> (objectId);
        NULL_CHECK(collisionObject, "The btCollisionObject does not exist.",)

        collisionObject->setSpinningFriction(friction);
    }

#ifdef __cplusplus
}
#endif
