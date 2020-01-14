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
#include "com_jme3_bullet_objects_PhysicsRigidBody.h"
#include "jmeBulletUtil.h"
#include "jmeMotionState.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyCentralForce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyCentralForce
    (JNIEnv *env, jobject object, jlong bodyId, jobject forceVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, forceVector, "The force vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, forceVector, &vec1);

        pBody->applyCentralForce(vec1);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyCentralImpulse
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyCentralImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jobject impulseVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, impulseVector, "The impulse vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, impulseVector, &vec);

        pBody->applyCentralImpulse(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyForce
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyForce
    (JNIEnv *env, jobject object, jlong bodyId, jobject forceVector,
            jobject locationVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, forceVector, "The force vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, forceVector, &vec1);

        NULL_CHECK(env, locationVector, "The location vector does not exist.",)
        btVector3 vec2;
        jmeBulletUtil::convert(env, locationVector, &vec2);

        pBody->applyForce(vec1, vec2);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyImpulse
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jobject impulseVector,
            jobject locationVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, impulseVector, "The impulse vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, impulseVector, &vec1);

        NULL_CHECK(env, locationVector, "The location vector does not exist.",)
        btVector3 vec2;
        jmeBulletUtil::convert(env, locationVector, &vec2);

        pBody->applyImpulse(vec1, vec2);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyTorque
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyTorque
    (JNIEnv *env, jobject object, jlong bodyId, jobject torqueVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, torqueVector, "The torque vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, torqueVector, &vec1);

        pBody->applyTorque(vec1);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyTorqueImpulse
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyTorqueImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jobject torqueImpulseVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, torqueImpulseVector,
                "The torque-impulse vector does not exist.",)
                btVector3 vec1;
        jmeBulletUtil::convert(env, torqueImpulseVector, &vec1);

        pBody->applyTorqueImpulse(vec1);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    clearForces
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_clearForces
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        pBody->clearForces();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    createRigidBody
     * Signature: (FJJ)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_createRigidBody
    (JNIEnv *env, jobject object, jfloat mass, jlong motionStateId,
            jlong shapeId) {
        jmeClasses::initJavaClasses(env);

        btMotionState *pMotionState
                = reinterpret_cast<btMotionState *> (motionStateId);
        NULL_CHECK(env, pMotionState, "The btMotionState does not exist.", 0)

        btCollisionShape *pShape
                = reinterpret_cast<btCollisionShape *> (shapeId);
        NULL_CHECK(env, pShape, "The btCollisionShape does not exist.", 0)

        btVector3 localInertia;
        int shapeType = pShape->getShapeType();
        if (shapeType == EMPTY_SHAPE_PROXYTYPE
                || shapeType == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
            localInertia.setZero(); // avoid a btAssert()
        } else {
            pShape->calculateLocalInertia(mass, localInertia);
        }
        btRigidBody *pBody
                = new btRigidBody(mass, pMotionState, pShape, localInertia);
        pBody->setUserPointer(NULL);

        return reinterpret_cast<jlong> (pBody);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getAngularDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularDamping
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", 0)

        return pBody->getAngularDamping();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getAngularFactor
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularFactor
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pBody->getAngularFactor(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getAngularSleepingThreshold
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularSleepingThreshold
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", 0)

        return pBody->getAngularSleepingThreshold();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getAngularVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pBody->getAngularVelocity(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getGravity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getGravity
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pBody->getGravity(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getInverseInertiaLocal
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getInverseInertiaLocal
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",);

        const btVector3& invInertia = pBody->getInvInertiaDiagLocal();

        NULL_CHECK(env, storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &invInertia, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getInverseInertiaWorld
     * Signature: (JLcom/jme3/math/Matrix3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getInverseInertiaWorld
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeMatrix) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",);

        const btMatrix3x3& invInertia = pBody->getInvInertiaTensorWorld();

        NULL_CHECK(env, storeMatrix, "The store matrix does not exist.",)
        jmeBulletUtil::convert(env, &invInertia, storeMatrix);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getLinearDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearDamping
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", 0)

        return pBody->getLinearDamping();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getLinearFactor
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearFactor
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pBody->getLinearFactor(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getLinearSleepingThreshold
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearSleepingThreshold
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", 0)

        return pBody->getLinearSleepingThreshold();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getLinearVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pBody->getLinearVelocity(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getPhysicsLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getPhysicsLocation
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &pBody->getWorldTransform().getOrigin(),
                storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getPhysicsRotation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getPhysicsRotation
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeQuaternion) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, storeQuaternion,
                "The store quaternion does not exist.",)
                jmeBulletUtil::convertQuat(env, &pBody->getWorldTransform().getBasis(),
                storeQuaternion);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getPhysicsRotationMatrix
     * Signature: (JLcom/jme3/math/Matrix3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getPhysicsRotationMatrix
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeMatrix) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)

        NULL_CHECK(env, storeMatrix, "The store matrix does not exist.",)
        jmeBulletUtil::convert(env, &pBody->getWorldTransform().getBasis(),
                storeMatrix);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getRestitution
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getRestitution
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", 0)

        return pBody->getRestitution();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getSquaredSpeed
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getSquaredSpeed
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", 0)

        btVector3 vec = pBody->getLinearVelocity();
        float squaredSpeed = vec.length2();
        return (jfloat) squaredSpeed;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    isActive
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_isActive
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", JNI_FALSE)

        return pBody->isActive();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    isInWorld
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_isInWorld
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", JNI_FALSE);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        return pBody->isInWorld();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setAngularDamping
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularDamping
    (JNIEnv *env, jobject object, jlong bodyId, jfloat value) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        pBody->setDamping(pBody->getLinearDamping(), value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setAngularFactor
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularFactor
    (JNIEnv *env, jobject object, jlong bodyId, jobject factorVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(env, factorVector, "The factor vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, factorVector, &vec);

        pBody->setAngularFactor(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setAngularSleepingThreshold
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularSleepingThreshold
    (JNIEnv *env, jobject object, jlong bodyId, jfloat value) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        pBody->setSleepingThresholds(pBody->getLinearSleepingThreshold(), value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setAngularVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(env, velocityVector, "The velocity vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, velocityVector, &vec);

        pBody->setAngularVelocity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setCollisionShape
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setCollisionShape
    (JNIEnv *env, jobject object, jlong bodyId, jlong shapeId) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        btCollisionShape *pShape
                = reinterpret_cast<btCollisionShape *> (shapeId);
        NULL_CHECK(env, pShape, "The btCollisionShape does not exist.",)

        pBody->setCollisionShape(pShape);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setDamping
     * Signature: (JFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setDamping
    (JNIEnv *env, jobject object, jlong bodyId, jfloat linear, jfloat angular) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        pBody->setDamping(linear, angular);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setGravity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setGravity
    (JNIEnv *env, jobject object, jlong bodyId, jobject gravityVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(env, gravityVector, "The gravity vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, gravityVector, &vec);

        pBody->setGravity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setInverseInertiaLocal
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setInverseInertiaLocal
    (JNIEnv *env, jobject object, jlong bodyId, jobject invInertiaVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(env, invInertiaVector,
                "The inverse-inertia vector does not exist.",)
                btVector3 vec;
        jmeBulletUtil::convert(env, invInertiaVector, &vec);

        pBody->setInvInertiaDiagLocal(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setKinematic
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setKinematic
    (JNIEnv *env, jobject object, jlong bodyId, jboolean value) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        int flags = pBody->getCollisionFlags();
        int bitmask = btCollisionObject::CF_KINEMATIC_OBJECT;
        if (value) {
            pBody->setCollisionFlags(flags | bitmask);
            pBody->setActivationState(DISABLE_DEACTIVATION);
        } else {
            pBody->setCollisionFlags(flags & ~bitmask);
            pBody->activate(true);
            pBody->forceActivationState(ACTIVE_TAG);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setLinearFactor
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearFactor
    (JNIEnv *env, jobject object, jlong bodyId, jobject factorVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(env, factorVector, "The factor vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, factorVector, &vec);

        pBody->setLinearFactor(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setLinearSleepingThreshold
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearSleepingThreshold
    (JNIEnv *env, jobject object, jlong bodyId, jfloat value) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        pBody->setSleepingThresholds(value,
                pBody->getAngularSleepingThreshold());
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setLinearVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",)
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        NULL_CHECK(env, velocityVector, "The velocity vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, velocityVector, &vec);

        pBody->setLinearVelocity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setPhysicsLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsLocation
    (JNIEnv *env, jobject object, jlong bodyId, jobject locationVector) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);
        jmeMotionState *pMotionState = (jmeMotionState *) pBody->getMotionState();

        NULL_CHECK(env, locationVector, "The location vector does not exist.",);
        pMotionState->setKinematicLocation(env, locationVector);

        pBody->setCenterOfMassTransform(pMotionState->worldTransform);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setPhysicsRotation
     * Signature: (JLcom/jme3/math/Matrix3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsRotation__JLcom_jme3_math_Matrix3f_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject rotationMatrix) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);
        jmeMotionState *pMotionState
                = (jmeMotionState *) pBody->getMotionState();

        NULL_CHECK(env, rotationMatrix, "The rotation matrix does not exist.",);

        pMotionState->setKinematicRotation(env, rotationMatrix);
        pBody->setCenterOfMassTransform(pMotionState->worldTransform);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setPhysicsRotation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsRotation__JLcom_jme3_math_Quaternion_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject rotationQuaternion) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);
        jmeMotionState *pMotionState
                = (jmeMotionState *) pBody->getMotionState();

        NULL_CHECK(env, rotationQuaternion,
                "The rotation quaternion does not exist.",);

        pMotionState->setKinematicRotationQuat(env, rotationQuaternion);
        pBody->setCenterOfMassTransform(pMotionState->worldTransform);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setSleepingThresholds
     * Signature: (JFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setSleepingThresholds
    (JNIEnv *env, jobject object, jlong bodyId, jfloat linear, jfloat angular) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.",);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        pBody->setSleepingThresholds(linear, angular);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    updateMassProps
     * Signature: (JJF)J
     * TODO should return void
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_updateMassProps
    (JNIEnv *env, jobject object, jlong bodyId, jlong shapeId, jfloat mass) {
        btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
        NULL_CHECK(env, pBody, "The btRigidBody does not exist.", 0);
        btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

        btCollisionShape *pShape = reinterpret_cast<btCollisionShape *> (shapeId);
        NULL_CHECK(env, pShape, "The btCollisionShape does not exist.", 0);

        btVector3 localInertia;
        int shapeType = pShape->getShapeType();
        if (shapeType == EMPTY_SHAPE_PROXYTYPE
                || shapeType == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
            localInertia.setZero(); // avoid a btAssert()
        } else {
            pShape->calculateLocalInertia(mass, localInertia);
        }
        pBody->setMassProps(mass, localInertia);

        return reinterpret_cast<jlong> (pBody);
    }

#ifdef __cplusplus
}
#endif
