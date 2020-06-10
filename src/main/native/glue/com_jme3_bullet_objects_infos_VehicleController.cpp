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
 * Author: Stephen Gold
 *
 * Based on com_jme3_bullet_objects_infos_PhysicsVehicle by Normen Hansen.
 */

#include "com_jme3_bullet_objects_infos_VehicleController.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    addWheel
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;FFJZ)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_addWheel
(JNIEnv *pEnv, jclass, jlong controllerId, jobject locationVector,
        jobject directionVector, jobject axleVector, jfloat restLength,
        jfloat radius, jlong tuningId, jboolean frontWheel) {
    btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.", 0);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.", 0);
    btVector3 location;
    jmeBulletUtil::convert(pEnv, locationVector, &location);

    NULL_CHK(pEnv, directionVector, "The direction vector does not exist.", 0);
    btVector3 direction;
    jmeBulletUtil::convert(pEnv, directionVector, &direction);

    NULL_CHK(pEnv, axleVector, "The axle vector does not exist.", 0);
    btVector3 axle;
    jmeBulletUtil::convert(pEnv, axleVector, &axle);

    btRaycastVehicle::btVehicleTuning * const
            pTuning = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (
            tuningId);
    NULL_CHK(pEnv, pTuning, "The btVehicleTuning does not exist.", 0);

    btWheelInfo * const
            pWheel = &pController->addWheel(location, direction, axle,
            restLength, radius, *pTuning, frontWheel);
    int idx = pController->getNumWheels();
    return idx - 1;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    applyEngineForce
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_applyEngineForce
(JNIEnv *pEnv, jclass, jlong controllerId, jint wheelIndex, jfloat force) {
    btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.",);
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pController->getNumWheels());

    pController->applyEngineForce(force, wheelIndex);
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    brake
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_brake
(JNIEnv *pEnv, jclass, jlong controllerId, jint wheelIndex, jfloat value) {
    btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.",);
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pController->getNumWheels());

    pController->setBrake(value, wheelIndex);
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    createRaycastVehicle
 * Signature: (JJ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_createRaycastVehicle
(JNIEnv *pEnv, jclass, jlong bodyId, jlong casterId) {
    jmeClasses::initJavaClasses(pEnv);

    btRigidBody * const pBody = reinterpret_cast<btRigidBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btRigidBody does not exist.", 0);
    btAssert(pBody->getInternalType() & btCollisionObject::CO_RIGID_BODY);

    pBody->setActivationState(DISABLE_DEACTIVATION);

    btVehicleRaycaster * const
            pCaster = reinterpret_cast<btDefaultVehicleRaycaster *> (casterId);
    NULL_CHK(pEnv, pCaster, "The btVehicleRaycaster does not exist.", 0);

    btRaycastVehicle::btVehicleTuning tuning;
    btRaycastVehicle *
            pController = new btRaycastVehicle(tuning, pBody, pCaster); //dance032

    return reinterpret_cast<jlong> (pController);
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_finalizeNative
(JNIEnv *pEnv, jclass, jlong controllerId) {
    const btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.",);

    delete pController; //dance032
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    getCurrentVehicleSpeedKmHour
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_getCurrentVehicleSpeedKmHour
(JNIEnv *pEnv, jclass, jlong controllerId) {
    const btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.", 0);

    return pController->getCurrentSpeedKmHour();
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    getForwardAxisIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_getForwardAxisIndex
(JNIEnv *pEnv, jclass, jlong controllerId) {
    const btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.", 0);

    int axisIndex = pController->getForwardAxis();
    return (jint) axisIndex;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    getForwardVector
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_getForwardVector
(JNIEnv *pEnv, jclass, jlong controllerId, jobject storeVector) {
    const btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.",);

    const btVector3& forwardVector = pController->getForwardVector();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    jmeBulletUtil::convert(pEnv, &forwardVector, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    getNumWheels
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_getNumWheels
(JNIEnv *pEnv, jclass, jlong controllerId) {
    const btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.", 0);

    int count = pController->getNumWheels();
    return (jint) count;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    getRightAxisIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_getRightAxisIndex
(JNIEnv *pEnv, jclass, jlong controllerId) {
    const btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.", 0);

    int axisIndex = pController->getRightAxis();
    return (jint) axisIndex;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    getUpAxisIndex
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_getUpAxisIndex
(JNIEnv *pEnv, jclass, jlong controllerId) {
    const btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.", 0);

    int axisIndex = pController->getUpAxis();
    return (jint) axisIndex;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    rayCast
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_rayCast
(JNIEnv *pEnv, jclass, jlong controllerId, jint wheelIndex) {
    btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.", 0);
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pController->getNumWheels());

    btWheelInfo& wheel = pController->m_wheelInfo[wheelIndex];
    btScalar result = pController->rayCast(wheel);

    return (jfloat) result;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    resetSuspension
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_resetSuspension
(JNIEnv *pEnv, jclass, jlong controllerId) {
    btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.",);

    pController->resetSuspension();
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    setCoordinateSystem
 * Signature: (JIII)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_setCoordinateSystem
(JNIEnv *pEnv, jclass, jlong controllerId, jint right, jint up, jint forward) {
    btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.",);
    btAssert(right >= 0);
    btAssert(right <= 2);
    btAssert(up >= 0);
    btAssert(up <= 2);
    btAssert(forward >= 0);
    btAssert(forward <= 2);

    pController->setCoordinateSystem(right, up, forward);
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    steer
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_steer
(JNIEnv *pEnv, jclass, jlong controllerId, jint wheelIndex, jfloat angle) {
    btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.",);
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pController->getNumWheels());

    pController->setSteeringValue(angle, wheelIndex);
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleController
 * Method:    updateWheelTransform
 * Signature: (JIZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleController_updateWheelTransform
(JNIEnv *pEnv, jclass, jlong controllerId, jint wheelIndex,
        jboolean interpolated) {
    btRaycastVehicle * const
            pController = reinterpret_cast<btRaycastVehicle *> (controllerId);
    NULL_CHK(pEnv, pController, "The btRaycastVehicle does not exist.",);
    btAssert(wheelIndex >= 0);
    btAssert(wheelIndex < pController->getNumWheels());

    pController->updateWheelTransform(wheelIndex, interpolated);
}
