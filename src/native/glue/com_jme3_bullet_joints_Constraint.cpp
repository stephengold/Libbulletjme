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

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    enableFeedback
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_enableFeedback
    (JNIEnv *env, jobject object, jlong constraintId, jboolean enable) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.",);

        pConstraint->enableFeedback(enable);
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    finalizeNative
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_finalizeNative
    (JNIEnv *env, jobject object, jlong constraintId) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.",);

        delete pConstraint;
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    getAppliedImpulse
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_Constraint_getAppliedImpulse
    (JNIEnv *env, jobject object, jlong constraintId) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.", 0)

        return pConstraint->getAppliedImpulse();
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    getBreakingImpulseThreshold
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_joints_Constraint_getBreakingImpulseThreshold
    (JNIEnv *env, jobject object, jlong constraintId) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.", 0)

        return pConstraint->getBreakingImpulseThreshold();
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    getConstraintType
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_joints_Constraint_getConstraintType
    (JNIEnv *env, jobject object, jlong constraintId) {
        const btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.", 0)

        btTypedConstraintType constraintType = pConstraint->getConstraintType();
        return (jint) constraintType;
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    getOverrideIterations
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_joints_Constraint_getOverrideIterations
    (JNIEnv *env, jobject object, jlong constraintId) {
        const btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint,
                "The btTypedConstraint does not exist.", 0);

        int overrideIterations = pConstraint->getOverrideNumSolverIterations();
        return (jint) overrideIterations;
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    isEnabled
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_Constraint_isEnabled
    (JNIEnv *env, jobject object, jlong constraintId) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.",
                JNI_FALSE);

        jboolean result = pConstraint->isEnabled();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    needsFeedback
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_joints_Constraint_needsFeedback
    (JNIEnv *env, jobject object, jlong constraintId) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.",
                JNI_FALSE);

        jboolean result = pConstraint->needsFeedback();
        return result;
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    overrideIterations
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_overrideIterations
    (JNIEnv *env, jobject object, jlong constraintId, jint numIterations) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.",);

        pConstraint->setOverrideNumSolverIterations(numIterations);
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    setBreakingImpulseThreshold
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_setBreakingImpulseThreshold
    (JNIEnv *env, jobject object, jlong constraintId, jfloat desiredValue) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.",)

        pConstraint->setBreakingImpulseThreshold(desiredValue);
    }

    /*
     * Class:     com_jme3_bullet_joints_Constraint
     * Method:    setEnabled
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_joints_Constraint_setEnabled
    (JNIEnv *env, jobject object, jlong constraintId, jboolean desiredSetting) {
        btTypedConstraint *pConstraint
                = reinterpret_cast<btTypedConstraint *> (constraintId);
        NULL_CHECK(env, pConstraint, "The btTypedConstraint does not exist.",)

        pConstraint->setEnabled(desiredSetting);
    }

#ifdef __cplusplus
}
#endif
