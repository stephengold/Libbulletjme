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
#include "com_jme3_bullet_objects_VehicleWheel.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    applyInfo
 * Signature: (JFFFFFFFFZF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_VehicleWheel_applyInfo
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex,
        jfloat suspensionStiffness, jfloat wheelsDampingRelaxation,
        jfloat wheelsDampingCompression, jfloat frictionSlip,
        jfloat rollInfluence, jfloat maxSuspensionTravelCm,
        jfloat maxSuspensionForce, jfloat radius, jboolean frontWheel,
        jfloat restLength) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.",)

    pVehicle->getWheelInfo(wheelIndex).m_suspensionStiffness
            = suspensionStiffness;
    pVehicle->getWheelInfo(wheelIndex).m_wheelsDampingRelaxation
            = wheelsDampingRelaxation;
    pVehicle->getWheelInfo(wheelIndex).m_wheelsDampingCompression
            = wheelsDampingCompression;
    pVehicle->getWheelInfo(wheelIndex).m_frictionSlip = frictionSlip;
    pVehicle->getWheelInfo(wheelIndex).m_rollInfluence = rollInfluence;
    pVehicle->getWheelInfo(wheelIndex).m_maxSuspensionTravelCm
            = maxSuspensionTravelCm;
    pVehicle->getWheelInfo(wheelIndex).m_maxSuspensionForce
            = maxSuspensionForce;
    pVehicle->getWheelInfo(wheelIndex).m_wheelsRadius = radius;
    pVehicle->getWheelInfo(wheelIndex).m_bIsFrontWheel = frontWheel;
    pVehicle->getWheelInfo(wheelIndex).m_suspensionRestLength1 = restLength;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getBrake
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getBrake
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", 0)

    btScalar scalar = pVehicle->getWheelInfo(wheelIndex).m_brake;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getCollisionLocation
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getCollisionLocation
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex, jobject out) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.",)

    jmeBulletUtil::convert(env,
            &pVehicle->getWheelInfo(wheelIndex).m_raycastInfo.m_contactPointWS,
            out);
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getCollisionNormal
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getCollisionNormal
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex,
        jobject out) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.",)

    jmeBulletUtil::convert(env,
            &pVehicle->getWheelInfo(wheelIndex).m_raycastInfo.m_contactNormalWS,
            out);
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getDeltaRotation
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getDeltaRotation
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", 0)

    return pVehicle->getWheelInfo(wheelIndex).m_deltaRotation;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getEngineForce
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getEngineForce
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", 0)

    btScalar scalar = pVehicle->getWheelInfo(wheelIndex).m_engineForce;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getRadius
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getRadius
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", 0)

    btScalar scalar = pVehicle->getWheelInfo(wheelIndex).m_wheelsRadius;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getRestLength
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getRestLength
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", 0)

    btScalar scalar
            = pVehicle->getWheelInfo(wheelIndex).m_suspensionRestLength1;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getRollInfluence
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getRollInfluence
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", 0)

    btScalar scalar = pVehicle->getWheelInfo(wheelIndex).m_rollInfluence;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getRotationAngle
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getRotationAngle
(JNIEnv *pEnv, jobject, jlong vehicleId, jint wheelIndex) {
    const btRaycastVehicle * const
            pVehicle = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0);

    const btWheelInfo& info = pVehicle->getWheelInfo(wheelIndex);
    btScalar scalar = info.m_rotation;
    return jfloat(scalar);
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getSkidInfo
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getSkidInfo
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", 0)

    return pVehicle->getWheelInfo(wheelIndex).m_skidInfo;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getSteerAngle
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getSteerAngle
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", 0)

    btScalar scalar = pVehicle->getWheelInfo(wheelIndex).m_steering;
    return (jfloat) scalar;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getSuspensionLength
 * Signature: (JI)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getSuspensionLength
(JNIEnv *pEnv, jobject, jlong vehicleId, jint wheelIndex) {
    const btRaycastVehicle * const
            pVehicle = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.", 0);

    const btWheelInfo& info = pVehicle->getWheelInfo(wheelIndex);
    btScalar scalar = info.m_raycastInfo.m_suspensionLength;
    return jfloat(scalar);
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getWheelLocation
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getWheelLocation
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex,
        jobject out) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.",)

    jmeBulletUtil::convert(env,
            &pVehicle->getWheelInfo(wheelIndex).m_worldTransform.getOrigin(),
            out);
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    getWheelRotation
 * Signature: (JLcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_VehicleWheel_getWheelRotation
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex,
        jobject out) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.",)

    jmeBulletUtil::convert(env,
            &pVehicle->getWheelInfo(wheelIndex).m_worldTransform.getBasis(),
            out);
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    isFront
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_VehicleWheel_isFront
(JNIEnv *env, jobject object, jlong vehicleId, jint wheelIndex) {
    btRaycastVehicle *pVehicle
            = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(env, pVehicle, "The btRaycastVehicle does not exist.", JNI_FALSE);

    bool result = pVehicle->getWheelInfo(wheelIndex).m_bIsFrontWheel;
    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    setRotationAngle
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_VehicleWheel_setRotationAngle
(JNIEnv *pEnv, jobject, jlong vehicleId, jint wheelIndex, jfloat angle) {
    btRaycastVehicle * const
            pVehicle = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",)

    btWheelInfo& info = pVehicle->getWheelInfo(wheelIndex);
    btScalar scalar = btScalar(angle);
    info.m_rotation = scalar;
}

/*
 * Class:     com_jme3_bullet_objects_VehicleWheel
 * Method:    setSuspensionLength
 * Signature: (JIF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_VehicleWheel_setSuspensionLength
(JNIEnv *pEnv, jobject, jlong vehicleId, jint wheelIndex, jfloat length) {
    btRaycastVehicle * const
            pVehicle = reinterpret_cast<btRaycastVehicle *> (vehicleId);
    NULL_CHK(pEnv, pVehicle, "The btRaycastVehicle does not exist.",)

    btWheelInfo& info = pVehicle->getWheelInfo(wheelIndex);
    btScalar scalar = btScalar(length);
    info.m_raycastInfo.m_suspensionLength = scalar;
}
