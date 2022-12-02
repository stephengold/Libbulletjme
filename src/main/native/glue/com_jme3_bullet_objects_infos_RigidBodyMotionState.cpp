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
#include "com_jme3_bullet_objects_infos_RigidBodyMotionState.h"
#include "jmeBulletUtil.h"
#include "jmeMotionState.h"

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    applyTransform
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Quaternion;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_applyTransform
(JNIEnv *pEnv, jclass, jlong stateId, jobject location, jobject rotation) {
    jmeMotionState *pMotionState
            = reinterpret_cast<jmeMotionState *> (stateId);
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.", JNI_FALSE)

    return pMotionState->applyTransform(pEnv, location, rotation);
}

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    createMotionState
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_createMotionState
(JNIEnv *pEnv, jclass) {
    jmeClasses::initJavaClasses(pEnv);

    jmeMotionState *pMotionState = new jmeMotionState(); //dance034
    return reinterpret_cast<jlong> (pMotionState);
}

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_finalizeNative
(JNIEnv *pEnv, jclass, jlong stateId) {
    jmeMotionState *pMotionState
            = reinterpret_cast<jmeMotionState *> (stateId);
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.",);

    delete pMotionState; //dance034
}

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    getWorldLocation
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_getWorldLocation
(JNIEnv *pEnv, jclass, jlong stateId, jobject value) {
    const jmeMotionState * const pMotionState
            = reinterpret_cast<jmeMotionState *> (stateId);
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.",)

    NULL_CHK(pEnv, value, "The store vector does not exist.",);

    jmeBulletUtil::convert(pEnv, &pMotionState->worldTransform.getOrigin(), value);
}

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    getWorldLocationDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_getWorldLocationDp
(JNIEnv *pEnv, jclass, jlong stateId, jobject storeVector) {
    const jmeMotionState * const pMotionState
            = reinterpret_cast<jmeMotionState *> (stateId);
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.",)

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    jmeBulletUtil::convertDp(
            pEnv, &pMotionState->worldTransform.getOrigin(), storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    getWorldRotation
 * Signature: (JLcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_getWorldRotation
(JNIEnv *pEnv, jclass, jlong stateId, jobject storeMatrix) {
    const jmeMotionState * const pMotionState
            = reinterpret_cast<jmeMotionState *> (stateId);
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.",)

    NULL_CHK(pEnv, storeMatrix, "The store matrix does not exist.",);

    jmeBulletUtil::convert(
            pEnv, &pMotionState->worldTransform.getBasis(), storeMatrix);
}

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    getWorldRotationDp
 * Signature: (JLcom/simsilica/mathd/Matrix3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_getWorldRotationDp
(JNIEnv *pEnv, jclass, jlong stateId, jobject storeMatrix) {
    const jmeMotionState * const pMotionState
            = reinterpret_cast<jmeMotionState *> (stateId);
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.",)

    NULL_CHK(pEnv, storeMatrix, "The store matrix does not exist.",);

    jmeBulletUtil::convertDp(
            pEnv, &pMotionState->worldTransform.getBasis(), storeMatrix);
}

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    getWorldRotationQuat
 * Signature: (JLcom/jme3/math/Quaternion;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_getWorldRotationQuat
(JNIEnv *pEnv, jclass, jlong stateId, jobject storeQuat) {
    const jmeMotionState * const pMotionState
            = reinterpret_cast<jmeMotionState *> (stateId);
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.",)

    NULL_CHK(pEnv, storeQuat, "The store quat does not exist.",);

    jmeBulletUtil::convertQuat(
            pEnv, &pMotionState->worldTransform.getBasis(), storeQuat);
}

/*
 * Class:     com_jme3_bullet_objects_infos_RigidBodyMotionState
 * Method:    getWorldRotationQuatDp
 * Signature: (JLcom/simsilica/mathd/Quatd;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_RigidBodyMotionState_getWorldRotationQuatDp
(JNIEnv *pEnv, jclass, jlong stateId, jobject storeQuat) {
    const jmeMotionState * const pMotionState
            = reinterpret_cast<jmeMotionState *> (stateId);
    NULL_CHK(pEnv, pMotionState, "The motion state does not exist.",)

    NULL_CHK(pEnv, storeQuat, "The store quat does not exist.",);

    jmeBulletUtil::convertQuatDp(
            pEnv, &pMotionState->worldTransform.getBasis(), storeQuat);
}
