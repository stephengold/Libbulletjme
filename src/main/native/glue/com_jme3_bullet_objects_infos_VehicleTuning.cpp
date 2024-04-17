/*
 * Copyright (c) 2020 jMonkeyEngine
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
 */
#include "com_jme3_bullet_objects_infos_VehicleTuning.h"
#include "jmeBulletUtil.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleTuning
 * Method:    createNative
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_infos_VehicleTuning_createNative
(JNIEnv *env, jclass) {
    jmeClasses::initJavaClasses(env);

    btRaycastVehicle::btVehicleTuning *
            pTuning = new btRaycastVehicle::btVehicleTuning(); //dance029
    return reinterpret_cast<jlong> (pTuning);
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleTuning
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleTuning_finalizeNative
(JNIEnv *, jclass, jlong tuningId) {
    btRaycastVehicle::btVehicleTuning * const
            pTuning = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (
            tuningId);

    if (pTuning) {
        delete pTuning; //dance029
    }
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleTuning
 * Method:    setFrictionSlip
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleTuning_setFrictionSlip
(JNIEnv *env, jclass, jlong tuningId, jfloat newValue) {
    btRaycastVehicle::btVehicleTuning *pTuning
            = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (
            tuningId);
    NULL_CHK(env, pTuning, "The btVehicleTuning does not exist.",)

    pTuning->m_frictionSlip = (btScalar) newValue;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleTuning
 * Method:    setMaxSuspensionForce
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleTuning_setMaxSuspensionForce
(JNIEnv *env, jclass, jlong tuningId, jfloat newValue) {
    btRaycastVehicle::btVehicleTuning *pTuning
            = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (
            tuningId);
    NULL_CHK(env, pTuning, "The btVehicleTuning does not exist.",)

    pTuning->m_maxSuspensionForce = (btScalar) newValue;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleTuning
 * Method:    setMaxSuspensionTravelCm
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleTuning_setMaxSuspensionTravelCm
(JNIEnv *env, jclass, jlong tuningId, jfloat newValue) {
    btRaycastVehicle::btVehicleTuning *pTuning
            = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (tuningId);
    NULL_CHK(env, pTuning, "The btVehicleTuning does not exist.",)

    pTuning->m_maxSuspensionTravelCm = (btScalar) newValue;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleTuning
 * Method:    setSuspensionCompression
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleTuning_setSuspensionCompression
(JNIEnv *env, jclass, jlong tuningId, jfloat newValue) {
    btRaycastVehicle::btVehicleTuning *pTuning
            = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (tuningId);
    NULL_CHK(env, pTuning, "The btVehicleTuning does not exist.",)

    pTuning->m_suspensionCompression = (btScalar) newValue;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleTuning
 * Method:    setSuspensionDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleTuning_setSuspensionDamping
(JNIEnv *env, jclass, jlong tuningId, jfloat newValue) {
    btRaycastVehicle::btVehicleTuning *pTuning
            = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (tuningId);
    NULL_CHK(env, pTuning, "The btVehicleTuning does not exist.",)

    pTuning->m_suspensionDamping = (btScalar) newValue;
}

/*
 * Class:     com_jme3_bullet_objects_infos_VehicleTuning
 * Method:    setSuspensionStiffness
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_VehicleTuning_setSuspensionStiffness
(JNIEnv *env, jclass, jlong tuningId, jfloat newValue) {
    btRaycastVehicle::btVehicleTuning *pTuning
            = reinterpret_cast<btRaycastVehicle::btVehicleTuning *> (tuningId);
    NULL_CHK(env, pTuning, "The btVehicleTuning does not exist.",)

    pTuning->m_suspensionStiffness = (btScalar) newValue;
}
