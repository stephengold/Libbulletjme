/*
 * Copyright (c) 2020-2023 jMonkeyEngine
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
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "com_jme3_bullet_RotationOrder.h"
#include "jmeBulletUtil.h"

/*
 * Author: Stephen Gold
 */

/*
 * Class:     com_jme3_bullet_RotationOrder
 * Method:    matrixToEuler
 * Signature: (ILcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_RotationOrder_matrixToEuler
(JNIEnv *pEnv, jclass, jint rotOrder, jobject rotMatrix, jobject storeVector) {
    NULL_CHK(pEnv, storeVector, "The storeVector does not exist.", JNI_FALSE);

    NULL_CHK(pEnv, rotMatrix, "The rotMatrix does not exist.", JNI_FALSE);
    btMatrix3x3 basis;
    jmeBulletUtil::convert(pEnv, rotMatrix, &basis);
    EXCEPTION_CHK(pEnv, JNI_FALSE);

    bool result = false;
    btVector3 angles;
    switch (rotOrder) {
        case RO_XYZ:
            result = btGeneric6DofSpring2Constraint::matrixToEulerXYZ(basis,
                    angles);
            break;
        case RO_XZY:
            result = btGeneric6DofSpring2Constraint::matrixToEulerXZY(basis,
                    angles);
            break;
        case RO_YXZ:
            result = btGeneric6DofSpring2Constraint::matrixToEulerYXZ(basis,
                    angles);
            break;
        case RO_YZX:
            result = btGeneric6DofSpring2Constraint::matrixToEulerYZX(basis,
                    angles);
            break;
        case RO_ZXY:
            result = btGeneric6DofSpring2Constraint::matrixToEulerZXY(basis,
                    angles);
            break;
        case RO_ZYX:
            result = btGeneric6DofSpring2Constraint::matrixToEulerZYX(basis,
                    angles);
            break;

        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The rotation order is unknown.");
            return JNI_FALSE;
    }

    jmeBulletUtil::convert(pEnv, &angles, storeVector);

    return (jboolean) result;
}