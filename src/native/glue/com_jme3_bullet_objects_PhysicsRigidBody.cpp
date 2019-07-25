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
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(forceVector, "The force vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, forceVector, &vec1);

        body->applyCentralForce(vec1);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyCentralImpulse
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyCentralImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jobject impulseVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(impulseVector, "The impulse vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, impulseVector, &vec);

        body->applyCentralImpulse(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyForce
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyForce
    (JNIEnv *env, jobject object, jlong bodyId, jobject forceVector,
            jobject locationVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(forceVector, "The force vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, forceVector, &vec1);

        NULL_CHECK(locationVector, "The location vector does not exist.",)
        btVector3 vec2;
        jmeBulletUtil::convert(env, locationVector, &vec2);

        body->applyForce(vec1, vec2);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyImpulse
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jobject impulseVector,
            jobject locationVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(impulseVector, "The impulse vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, impulseVector, &vec1);

        NULL_CHECK(locationVector, "The location vector does not exist.",)
        btVector3 vec2;
        jmeBulletUtil::convert(env, locationVector, &vec2);

        body->applyImpulse(vec1, vec2);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyTorque
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyTorque
    (JNIEnv *env, jobject object, jlong bodyId, jobject torqueVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(torqueVector, "The torque vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, torqueVector, &vec1);

        body->applyTorque(vec1);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    applyTorqueImpulse
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyTorqueImpulse
    (JNIEnv *env, jobject object, jlong bodyId, jobject torqueImpulseVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(torqueImpulseVector, "The torque-impulse vector does not exist.",)
        btVector3 vec1;
        jmeBulletUtil::convert(env, torqueImpulseVector, &vec1);

        body->applyTorqueImpulse(vec1);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    clearForces
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_clearForces
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        body->clearForces();
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

        btMotionState* motionState
                = reinterpret_cast<btMotionState*> (motionStateId);
        NULL_CHECK(motionState, "The btMotionState does not exist.", 0)

        btCollisionShape* shape = reinterpret_cast<btCollisionShape*> (shapeId);
        NULL_CHECK(shape, "The btCollisionShape does not exist.", 0)

        btVector3 localInertia;
        int shapeType = shape->getShapeType();
        if (shapeType == EMPTY_SHAPE_PROXYTYPE
                || shapeType == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
            localInertia.setZero(); // avoid a btAssert()
        } else {
            shape->calculateLocalInertia(mass, localInertia);
        }
        btRigidBody* body
                = new btRigidBody(mass, motionState, shape, localInertia);
        body->setUserPointer(NULL);

        return reinterpret_cast<jlong> (body);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getAngularDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularDamping
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.", 0)

        return body->getAngularDamping();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getAngularFactor
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularFactor
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &body->getAngularFactor(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getAngularSleepingThreshold
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularSleepingThreshold
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.", 0)

        return body->getAngularSleepingThreshold();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getAngularVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &body->getAngularVelocity(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getGravity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getGravity
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &body->getGravity(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getInverseInertiaLocal
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getInverseInertiaLocal
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",);

        const btVector3& invInertia = body->getInvInertiaDiagLocal();

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &invInertia, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getInverseInertiaWorld
     * Signature: (JLcom/jme3/math/Matrix3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getInverseInertiaWorld
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeMatrix) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",);

        const btMatrix3x3& invInertia = body->getInvInertiaTensorWorld();

        NULL_CHECK(storeMatrix, "The store matrix does not exist.",)
        jmeBulletUtil::convert(env, &invInertia, storeMatrix);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getLinearDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearDamping
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.", 0)

        return body->getLinearDamping();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getLinearFactor
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearFactor
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &body->getLinearFactor(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getLinearSleepingThreshold
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearSleepingThreshold
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.", 0)

        return body->getLinearSleepingThreshold();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getLinearVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &body->getLinearVelocity(), storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getPhysicsLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getPhysicsLocation
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &body->getWorldTransform().getOrigin(),
                storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getPhysicsRotation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getPhysicsRotation
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeQuaternion) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(storeQuaternion, "The store quaternion does not exist.",)
        jmeBulletUtil::convertQuat(env, &body->getWorldTransform().getBasis(),
                storeQuaternion);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getPhysicsRotationMatrix
     * Signature: (JLcom/jme3/math/Matrix3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getPhysicsRotationMatrix
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeMatrix) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(storeMatrix, "The store matrix does not exist.",)
        jmeBulletUtil::convert(env, &body->getWorldTransform().getBasis(),
                storeMatrix);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    getRestitution
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getRestitution
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.", 0)

        return body->getRestitution();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    isActive
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_isActive
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.", false)

        return body->isActive();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    isInWorld
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_isInWorld
    (JNIEnv *env, jobject object, jlong bodyId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.", false)

        return body->isInWorld();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setAngularDamping
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularDamping
    (JNIEnv *env, jobject object, jlong bodyId, jfloat value) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        body->setDamping(body->getLinearDamping(), value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setAngularFactor
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularFactor
    (JNIEnv *env, jobject object, jlong bodyId, jobject factorVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(factorVector, "The factor vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, factorVector, &vec);

        body->setAngularFactor(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setAngularSleepingThreshold
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularSleepingThreshold
    (JNIEnv *env, jobject object, jlong bodyId, jfloat value) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        body->setSleepingThresholds(body->getLinearSleepingThreshold(), value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setAngularVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, velocityVector, &vec);

        body->setAngularVelocity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setCollisionShape
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setCollisionShape
    (JNIEnv *env, jobject object, jlong bodyId, jlong shapeId) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        btCollisionShape* shape = reinterpret_cast<btCollisionShape*> (shapeId);
        NULL_CHECK(shape, "The btCollisionShape does not exist.",)

        body->setCollisionShape(shape);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setDamping
     * Signature: (JFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setDamping
    (JNIEnv *env, jobject object, jlong bodyId, jfloat linear, jfloat angular) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        body->setDamping(linear, angular);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setGravity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setGravity
    (JNIEnv *env, jobject object, jlong bodyId, jobject gravityVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(gravityVector, "The gravity vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, gravityVector, &vec);

        body->setGravity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setInverseInertiaLocal
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setInverseInertiaLocal
    (JNIEnv *env, jobject object, jlong bodyId, jobject invInertiaVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(invInertiaVector, "The inverse-inertia vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, invInertiaVector, &vec);

        body->setInvInertiaDiagLocal(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setKinematic
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setKinematic
    (JNIEnv *env, jobject object, jlong bodyId, jboolean value) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",);

        int flags = body->getCollisionFlags();
        int bitmask = btCollisionObject::CF_KINEMATIC_OBJECT;
        if (value) {
            body->setCollisionFlags(flags | bitmask);
            body->setActivationState(DISABLE_DEACTIVATION);
        } else {
            body->setCollisionFlags(flags & ~bitmask);
            body->activate(true);
            body->forceActivationState(ACTIVE_TAG);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setLinearFactor
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearFactor
    (JNIEnv *env, jobject object, jlong bodyId, jobject factorVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(factorVector, "The factor vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, factorVector, &vec);

        body->setLinearFactor(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setLinearSleepingThreshold
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearSleepingThreshold
    (JNIEnv *env, jobject object, jlong bodyId, jfloat value) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        body->setSleepingThresholds(value, body->getAngularSleepingThreshold());
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setLinearVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",)

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, velocityVector, &vec);

        body->setLinearVelocity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setPhysicsLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsLocation
    (JNIEnv *env, jobject object, jlong bodyId, jobject locationVector) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",);
        jmeMotionState *motionState = (jmeMotionState*) body->getMotionState();

        NULL_CHECK(locationVector, "The location vector does not exist.",);
        motionState->setKinematicLocation(env, locationVector);

        body->setCenterOfMassTransform(motionState->worldTransform);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setPhysicsRotation
     * Signature: (JLcom/jme3/math/Matrix3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsRotation__JLcom_jme3_math_Matrix3f_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject rotationMatrix) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",);
        jmeMotionState *motionState = (jmeMotionState*) body->getMotionState();

        NULL_CHECK(rotationMatrix, "The rotation matrix does not exist.",);
        motionState->setKinematicRotation(env, rotationMatrix);

        body->setCenterOfMassTransform(motionState->worldTransform);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setPhysicsRotation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsRotation__JLcom_jme3_math_Quaternion_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject rotationQuaternion) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",);
        jmeMotionState *motionState = (jmeMotionState*) body->getMotionState();

        NULL_CHECK(rotationQuaternion, "The rotation quaternion does not exist.",);
        motionState->setKinematicRotationQuat(env, rotationQuaternion);

        body->setCenterOfMassTransform(motionState->worldTransform);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    setSleepingThresholds
     * Signature: (JFF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setSleepingThresholds
    (JNIEnv *env, jobject object, jlong bodyId, jfloat linear, jfloat angular) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.",);

        body->setSleepingThresholds(linear, angular);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsRigidBody
     * Method:    updateMassProps
     * Signature: (JJF)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_updateMassProps
    (JNIEnv *env, jobject object, jlong bodyId, jlong shapeId, jfloat mass) {
        btRigidBody* body = reinterpret_cast<btRigidBody*> (bodyId);
        NULL_CHECK(body, "The btRigidBody does not exist.", 0);

        btCollisionShape* shape = reinterpret_cast<btCollisionShape*> (shapeId);
        NULL_CHECK(shape, "The btCollisionShape does not exist.", 0);

        btVector3 localInertia;
        shape->calculateLocalInertia(mass, localInertia);
        body->setMassProps(mass, localInertia);

        return reinterpret_cast<jlong> (body);
    }

#ifdef __cplusplus
}
#endif
