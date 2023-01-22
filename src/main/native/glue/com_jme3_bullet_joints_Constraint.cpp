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
#include "com_jme3_bullet_joints_Constraint.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    getConstraintType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_joints_Constraint_getConstraintType
(JNIEnv *pEnv, jclass, jlong constraintId) {
    const btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.", 0)

    btTypedConstraintType constraintType = pConstraint->getConstraintType();
    ASSERT_CHK(pEnv, constraintType >= POINT2POINT_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, constraintType <= MAX_CONSTRAINT_TYPE, 0);

    return constraintType;
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    enableFeedback
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_enableFeedback
(JNIEnv *pEnv, jclass, jlong constraintId, jboolean enable) {
    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE,);

    bool needsFeedback = enable;
    pConstraint->enableFeedback(needsFeedback);
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_finalizeNative
(JNIEnv *pEnv, jclass, jlong constraintId) {
    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE,);

    delete pConstraint; //dance021
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    getAppliedImpulse
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_Constraint_getAppliedImpulse
(JNIEnv *pEnv, jclass, jlong constraintId) {
    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE, 0);

    btScalar result = pConstraint->getAppliedImpulse();
    return result;
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    getBreakingImpulseThreshold
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_Constraint_getBreakingImpulseThreshold
(JNIEnv *pEnv, jclass, jlong constraintId) {
    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.", 0)
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE, 0);

    btScalar result = pConstraint->getBreakingImpulseThreshold();
    return result;
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    getOverrideIterations
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_joints_Constraint_getOverrideIterations
(JNIEnv *pEnv, jclass, jlong constraintId) {
    const btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.", 0);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE, 0);

    int overrideIterations = pConstraint->getOverrideNumSolverIterations();
    return overrideIterations;
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    isEnabled
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_Constraint_isEnabled
(JNIEnv *pEnv, jclass, jlong constraintId) {
    const btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",
            JNI_FALSE);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE, 0);

    bool result = pConstraint->isEnabled();
    return result;
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    needsFeedback
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_Constraint_needsFeedback
(JNIEnv *pEnv, jclass, jlong constraintId) {
    const btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",
            JNI_FALSE);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE, 0);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE, 0);

    bool result = pConstraint->needsFeedback();
    return result;
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    overrideIterations
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_overrideIterations
(JNIEnv *pEnv, jclass, jlong constraintId, jint numIterations) {
    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE,);

    int overrideNumIterations = numIterations;
    pConstraint->setOverrideNumSolverIterations(overrideNumIterations);
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    setBreakingImpulseThreshold
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_setBreakingImpulseThreshold
(JNIEnv *pEnv, jclass, jlong constraintId, jfloat desiredValue) {
    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",)
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE,);

    btScalar threshold = btScalar(desiredValue);
    pConstraint->setBreakingImpulseThreshold(threshold);
}

/*
 * Class:     com_jme3_bullet_joints_Constraint
 * Method:    setEnabled
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_setEnabled
(JNIEnv *pEnv, jclass, jlong constraintId, jboolean desiredSetting) {
    btTypedConstraint * const
            pConstraint = reinterpret_cast<btTypedConstraint *> (constraintId);
    NULL_CHK(pEnv, pConstraint, "The btTypedConstraint does not exist.",)
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() >= POINT2POINT_CONSTRAINT_TYPE,);
    ASSERT_CHK(pEnv, pConstraint->getConstraintType() <= MAX_CONSTRAINT_TYPE,);

    bool enabled = desiredSetting;
    pConstraint->setEnabled(enabled);
}
