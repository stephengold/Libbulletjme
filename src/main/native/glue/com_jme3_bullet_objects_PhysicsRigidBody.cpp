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

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    applyCentralForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyCentralForce
(JNIEnv *pEnv, jclass, jlong bodyId, jobject forceVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, forceVector, "The force vector does not exist.",)
    btVector3 vec1;
    jmeBulletUtil::convert(pEnv, forceVector, &vec1);
    EXCEPTION_CHK(pEnv,);

    pBody->applyCentralForce(vec1);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    applyCentralImpulse
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyCentralImpulse
(JNIEnv *pEnv, jclass, jlong bodyId, jobject impulseVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, impulseVector, "The impulse vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, impulseVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->applyCentralImpulse(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    applyForce
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyForce
(JNIEnv *pEnv, jclass, jlong bodyId, jobject forceVector,
        jobject locationVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, forceVector, "The force vector does not exist.",)
    btVector3 vec1;
    jmeBulletUtil::convert(pEnv, forceVector, &vec1);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)
    btVector3 vec2;
    jmeBulletUtil::convert(pEnv, locationVector, &vec2);
    EXCEPTION_CHK(pEnv,);

    pBody->applyForce(vec1, vec2);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    applyImpulse
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyImpulse
(JNIEnv *pEnv, jclass, jlong bodyId, jobject impulseVector,
        jobject locationVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, impulseVector, "The impulse vector does not exist.",)
    btVector3 vec1;
    jmeBulletUtil::convert(pEnv, impulseVector, &vec1);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)
    btVector3 vec2;
    jmeBulletUtil::convert(pEnv, locationVector, &vec2);
    EXCEPTION_CHK(pEnv,);

    pBody->applyImpulse(vec1, vec2);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    applyTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyTorque
(JNIEnv *pEnv, jclass, jlong bodyId, jobject torqueVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, torqueVector, "The torque vector does not exist.",)
    btVector3 vec1;
    jmeBulletUtil::convert(pEnv, torqueVector, &vec1);
    EXCEPTION_CHK(pEnv,);

    pBody->applyTorque(vec1);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    applyTorqueImpulse
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_applyTorqueImpulse
(JNIEnv *pEnv, jclass, jlong bodyId, jobject torqueImpulseVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, torqueImpulseVector,
            "The torque-impulse vector does not exist.",)
            btVector3 vec1;
    jmeBulletUtil::convert(pEnv, torqueImpulseVector, &vec1);
    EXCEPTION_CHK(pEnv,);

    pBody->applyTorqueImpulse(vec1);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    clearForces
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_clearForces
(JNIEnv *pEnv, jclass, jlong bodyId) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    pBody->clearForces();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    createRigidBody
 * Signature: (FJJ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_createRigidBody
(JNIEnv *pEnv, jclass, jfloat mass, jlong motionStateId,
        jlong shapeId) {
    jmeClasses::initJavaClasses(pEnv);

    btMotionState * const
            pMotionState = reinterpret_cast<btMotionState *> (motionStateId);
    NULL_CHK(pEnv, pMotionState, "The btMotionState does not exist.", 0)

    btCollisionShape * const
            pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", 0)

    btVector3 localInertia;
    const int shapeType = pShape->getShapeType();
    if (shapeType == EMPTY_SHAPE_PROXYTYPE
            || shapeType == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
        localInertia.setZero(); // avoid a btAssert()
    } else {
        pShape->calculateLocalInertia(mass, localInertia);
    }

    btRigidBody * const
            pBody = new btRigidBody(mass, pMotionState, pShape, localInertia); //dance014
    pBody->setUserPointer(NULL); // TODO unnecessary?

    return reinterpret_cast<jlong> (pBody);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getAngularDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularDamping
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", 0)

    return pBody->getAngularDamping();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getAngularFactor
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularFactor
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pBody->getAngularFactor(), storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getAngularSleepingThreshold
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularSleepingThreshold
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", 0)

    return pBody->getAngularSleepingThreshold();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getAngularVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularVelocity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pBody->getAngularVelocity(), storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getAngularVelocityDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getAngularVelocityDp
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVectorDp) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, storeVectorDp, "The store vector does not exist.",)
    jmeBulletUtil::convertDp(pEnv, &pBody->getAngularVelocity(), storeVectorDp);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getGravity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pBody->getGravity(), storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getGravityDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getGravityDp
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVectorDp) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, storeVectorDp, "The store vector does not exist.",)
    jmeBulletUtil::convertDp(pEnv, &pBody->getGravity(), storeVectorDp);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getInverseInertiaLocal
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getInverseInertiaLocal
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);

    const btVector3& invInertia = pBody->getInvInertiaDiagLocal();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &invInertia, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getInverseInertiaWorld
 * Signature: (JLcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getInverseInertiaWorld
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeMatrix) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);

    const btMatrix3x3& invInertia = pBody->getInvInertiaTensorWorld();

    NULL_CHK(pEnv, storeMatrix, "The store matrix does not exist.",)
    jmeBulletUtil::convert(pEnv, &invInertia, storeMatrix);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getLinearDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearDamping
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", 0)

    return pBody->getLinearDamping();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getLinearFactor
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearFactor
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pBody->getLinearFactor(), storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getLinearSleepingThreshold
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearSleepingThreshold
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", 0)

    return pBody->getLinearSleepingThreshold();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearVelocity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &pBody->getLinearVelocity(), storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getLinearVelocityDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getLinearVelocityDp
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVectorDp) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)

    NULL_CHK(pEnv, storeVectorDp, "The store vector does not exist.",)
    jmeBulletUtil::convertDp(pEnv, &pBody->getLinearVelocity(), storeVectorDp);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getMass
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getMass
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btRigidBody * const pBody
            = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", 0)

    btScalar mass = pBody->getMass();
    return (jfloat) mass;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getSquaredSpeed
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getSquaredSpeed
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", 0)

    btVector3 vec = pBody->getLinearVelocity();
    float squaredSpeed = vec.length2();
    return (jfloat) squaredSpeed;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getTotalForce
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getTotalForce
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& force = pBody->getTotalForce();
    jmeBulletUtil::convert(pEnv, &force, storeVector);

}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getTotalTorque
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getTotalTorque
(JNIEnv *pEnv, jclass, jlong bodyId, jobject storeVector) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& torque = pBody->getTotalTorque();
    jmeBulletUtil::convert(pEnv, &torque, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    getUseSpaceGravity
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_getUseSpaceGravity
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", JNI_FALSE);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY, JNI_FALSE);

    int flags = pBody->getFlags();
    bool result = (flags & BT_DISABLE_WORLD_GRAVITY) == 0x0;
    return jboolean(result);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setAngularDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularDamping
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat value) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    pBody->setDamping(pBody->getLinearDamping(), value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setAngularFactor
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularFactor
(JNIEnv *pEnv, jclass, jlong bodyId, jobject factorVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, factorVector, "The factor vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, factorVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setAngularFactor(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setAngularSleepingThreshold
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularSleepingThreshold
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat value) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    pBody->setSleepingThresholds(pBody->getLinearSleepingThreshold(), value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setAngularVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularVelocity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, velocityVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setAngularVelocity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setAngularVelocityDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setAngularVelocityDp
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVectorDp) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, velocityVectorDp, "The velocity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convertDp(pEnv, velocityVectorDp, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setAngularVelocity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setCollisionShape
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setCollisionShape
(JNIEnv *pEnv, jclass, jlong bodyId, jlong shapeId) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.",)

    pBody->setCollisionShape(pShape);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setDamping
 * Signature: (JFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setDamping
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat linear, jfloat angular) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    pBody->setDamping(linear, angular);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setGravity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject gravityVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, gravityVector, "The gravity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, gravityVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setGravity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setGravityDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setGravityDp
(JNIEnv *pEnv, jclass, jlong bodyId, jobject gravityVectorDp) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, gravityVectorDp, "The gravity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convertDp(pEnv, gravityVectorDp, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setGravity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setInverseInertiaLocal
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setInverseInertiaLocal
(JNIEnv *pEnv, jclass, jlong bodyId, jobject invInertiaVector) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, invInertiaVector, "The inverse-inertia vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, invInertiaVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setInvInertiaDiagLocal(vec);
    pBody->updateInertiaTensor();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setKinematic
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setKinematic
(JNIEnv *pEnv, jclass, jlong bodyId, jboolean value) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    int flags = pBody->getCollisionFlags();
    bool oldValue = pBody->isKinematicObject();

    if (value && !oldValue) { // dynamic/static -> kinematic
        flags &= ~btCollisionObject::CF_STATIC_OBJECT;
        flags |= btCollisionObject::CF_KINEMATIC_OBJECT;
        pBody->setCollisionFlags(flags);

        pBody->setActivationState(DISABLE_DEACTIVATION);

    } else if (oldValue && !value) { // kinematic -> dynamic/static
        bool zeroMass = (pBody->getMass() == btScalar(0.0));
        if (zeroMass) {
            flags |= btCollisionObject::CF_STATIC_OBJECT;
        }
        flags &= ~btCollisionObject::CF_KINEMATIC_OBJECT;
        pBody->setCollisionFlags(flags);

        if (!zeroMass) {
            pBody->activate(true);
            pBody->forceActivationState(ACTIVE_TAG);
        }
    }
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setLinearFactor
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearFactor
(JNIEnv *pEnv, jclass, jlong bodyId, jobject factorVector) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, factorVector, "The factor vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, factorVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setLinearFactor(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setLinearSleepingThreshold
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearSleepingThreshold
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat value) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    pBody->setSleepingThresholds(value,
            pBody->getAngularSleepingThreshold());
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearVelocity
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVector) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, velocityVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setLinearVelocity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setLinearVelocityDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setLinearVelocityDp
(JNIEnv *pEnv, jclass, jlong bodyId, jobject velocityVectorDp) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",)
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    NULL_CHK(pEnv, velocityVectorDp, "The velocity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convertDp(pEnv, velocityVectorDp, &vec);
    EXCEPTION_CHK(pEnv,);

    pBody->setLinearVelocity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setPhysicsLocation
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsLocation
(JNIEnv *pEnv, jclass, jlong bodyId, jobject locationVector) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);
    jmeMotionState *pMotionState = (jmeMotionState *) pBody->getMotionState();

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",);
    pMotionState->setKinematicLocation(pEnv, locationVector);

    pBody->setCenterOfMassTransform(pMotionState->worldTransform);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setPhysicsLocationDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsLocationDp
(JNIEnv *pEnv, jclass, jlong bodyId, jobject locationVectorDp) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);
    jmeMotionState *pMotionState = (jmeMotionState *) pBody->getMotionState();
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.",);

    NULL_CHK(pEnv, locationVectorDp, "The location vector does not exist.",);
    pMotionState->setKinematicLocationDp(pEnv, locationVectorDp);

    pBody->setCenterOfMassTransform(pMotionState->worldTransform);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setPhysicsRotation
 * Signature: (JLcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsRotation__JLcom_jme3_math_Matrix3f_2
(JNIEnv *pEnv, jclass, jlong bodyId, jobject rotationMatrix) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);
    jmeMotionState *pMotionState
            = (jmeMotionState *) pBody->getMotionState();

    NULL_CHK(pEnv, rotationMatrix, "The rotation matrix does not exist.",);

    pMotionState->setKinematicRotation(pEnv, rotationMatrix);
    pBody->setCenterOfMassTransform(pMotionState->worldTransform);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setPhysicsRotation
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsRotation__JLcom_jme3_math_Quaternion_2
(JNIEnv *pEnv, jclass, jlong bodyId, jobject rotationQuaternion) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);
    jmeMotionState *pMotionState
            = (jmeMotionState *) pBody->getMotionState();

    NULL_CHK(pEnv, rotationQuaternion,
            "The rotation quaternion does not exist.",);

    pMotionState->setKinematicRotationQuat(pEnv, rotationQuaternion);
    pBody->setCenterOfMassTransform(pMotionState->worldTransform);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setPhysicsRotationDp
 * Signature: (JLcom/simsilica/mathd/Matrix3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsRotationDp__JLcom_simsilica_mathd_Matrix3d_2
(JNIEnv *pEnv, jclass, jlong bodyId, jobject rotationMatrix3d) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);
    jmeMotionState * const pMotionState
            = (jmeMotionState *) pBody->getMotionState();

    NULL_CHK(pEnv, rotationMatrix3d, "The rotation Matrix3d does not exist.",);

    pMotionState->setKinematicRotationMatrix3d(pEnv, rotationMatrix3d);
    pBody->setCenterOfMassTransform(pMotionState->worldTransform);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setPhysicsRotationDp
 * Signature: (JLcom/simsilica/mathd/Quatd;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setPhysicsRotationDp__JLcom_simsilica_mathd_Quatd_2
(JNIEnv *pEnv, jclass, jlong bodyId, jobject rotationQuatd) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);
    jmeMotionState *pMotionState = (jmeMotionState *) pBody->getMotionState();

    NULL_CHK(pEnv, rotationQuatd, "The rotation Quatd does not exist.",);

    pMotionState->setKinematicRotationQuatd(pEnv, rotationQuatd);
    pBody->setCenterOfMassTransform(pMotionState->worldTransform);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setSleepingThresholds
 * Signature: (JFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setSleepingThresholds
(JNIEnv *pEnv, jclass, jlong bodyId, jfloat linear, jfloat angular) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    pBody->setSleepingThresholds(linear, angular);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    setUseSpaceGravity
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_setUseSpaceGravity
(JNIEnv *pEnv, jclass, jlong bodyId, jboolean use) {
    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    int flags = pBody->getFlags();
    if (use) {
        flags &= ~BT_DISABLE_WORLD_GRAVITY;
    } else {
        flags |= BT_DISABLE_WORLD_GRAVITY;
    }
    pBody->setFlags(flags);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsRigidBody
 * Method:    updateMassProps
 * Signature: (JJF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsRigidBody_updateMassProps
(JNIEnv *pEnv, jclass, jlong bodyId, jlong shapeId, jfloat mass) {
    btRigidBody *pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.",);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY,);

    btCollisionShape *pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.",);

    btVector3 localInertia;
    int shapeType = pShape->getShapeType();
    if (shapeType == EMPTY_SHAPE_PROXYTYPE
            || shapeType == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
        localInertia.setZero(); // avoid a btAssert()
    } else {
        pShape->calculateLocalInertia(mass, localInertia);
    }
    pBody->setMassProps(mass, localInertia);
    pBody->updateInertiaTensor();
}
