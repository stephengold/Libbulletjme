/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
 * Author: Dokthar
 */
#include "com_jme3_bullet_joints_SoftPhysicsJoint.h"
#include "jmeClasses.h"
#include "BulletSoftBody/btSoftBody.h"

/*
 * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_finalizeNative
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSoftBody::Joint *pJoint = reinterpret_cast<btSoftBody::Joint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The joint does not exist.",)

    btAlignedFree(pJoint);
}

/*
 * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
 * Method:    getConstraintForceMixing
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_getConstraintForceMixing
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSoftBody::Joint *pJoint
            = reinterpret_cast<btSoftBody::Joint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The joint does not exist.", 0)

    return pJoint->m_cfm;
}

/*
 * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
 * Method:    getErrorReductionParameter
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_getErrorReductionParameter
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSoftBody::Joint *pJoint
            = reinterpret_cast<btSoftBody::Joint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The joint does not exist.", 0)

    return pJoint->m_erp;
}

/*
 * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
 * Method:    getSplit
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_getSplit
(JNIEnv *pEnv, jclass, jlong jointId) {
    btSoftBody::Joint *pJoint
            = reinterpret_cast<btSoftBody::Joint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The joint does not exist.", 0)

    return pJoint->m_split;
}

/*
 * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
 * Method:    setConstraintForceMixing
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_setConstraintForceMixing
(JNIEnv *pEnv, jclass, jlong jointId, jfloat cfm) {
    btSoftBody::Joint *pJoint
            = reinterpret_cast<btSoftBody::Joint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The joint does not exist.",)

    pJoint->m_cfm = cfm;
}

/*
 * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
 * Method:    setErrorReductionParameter
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_setErrorReductionParameter
(JNIEnv *pEnv, jclass, jlong jointId, jfloat erp) {
    btSoftBody::Joint *pJoint
            = reinterpret_cast<btSoftBody::Joint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The joint does not exist.",)

    pJoint->m_erp = erp;
}

/*
 * Class:     com_jme3_bullet_joints_SoftPhysicsJoint
 * Method:    setSplit
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_SoftPhysicsJoint_setSplit
(JNIEnv *pEnv, jclass, jlong jointId, jfloat split) {
    btSoftBody::Joint *pJoint
            = reinterpret_cast<btSoftBody::Joint *> (jointId);
    NULL_CHK(pEnv, pJoint, "The joint does not exist.",)

    pJoint->m_split = split;
}
