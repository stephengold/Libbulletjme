/*
 * Copyright (c) 2018-2023 jMonkeyEngine
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
#include "com_jme3_bullet_collision_shapes_MultiSphere.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_MultiSphere
 * Method:    createShape
 * Signature: (F)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MultiSphere_createShape
(JNIEnv *pEnv, jclass, jobjectArray centers, jfloatArray radii,
        jint numSpheres) {
    jmeClasses::initJavaClasses(pEnv);

    int n = numSpheres;
    btVector3 * const pCenters = new btVector3[n]; //dance017
    for (int i = 0; i < n; ++i) {
        jobject center = pEnv->GetObjectArrayElement(centers, i);
        EXCEPTION_CHK(pEnv, 0);
        jmeBulletUtil::convert(pEnv, center, &pCenters[i]);
        EXCEPTION_CHK(pEnv, 0);
    }

    btScalar *pRadii;
#ifdef BT_USE_DOUBLE_PRECISION
    float * const pFloats = pEnv->GetFloatArrayElements(radii, 0);
    EXCEPTION_CHK(pEnv, 0);
    pRadii = new btScalar[n]; //dance018
    for (int i = 0; i < n; ++i) {
        pRadii[i] = pFloats[i];
    }
#else
    pRadii = pEnv->GetFloatArrayElements(radii, 0);
    EXCEPTION_CHK(pEnv, 0);
#endif

    btMultiSphereShape * const
            pShape = new btMultiSphereShape(pCenters, pRadii, n); //dance016

#ifdef BT_USE_DOUBLE_PRECISION
    delete[] pRadii; //dance018
    pEnv->ReleaseFloatArrayElements(radii, pFloats, 0);
#else
    pEnv->ReleaseFloatArrayElements(radii, pRadii, 0);
#endif
    EXCEPTION_CHK(pEnv, 0);
    delete[] pCenters; //dance017

    return reinterpret_cast<jlong> (pShape);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_MultiSphere
 * Method:    recalcAabb
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_MultiSphere_recalcAabb
(JNIEnv *pEnv, jclass, jlong shapeId) {
    btMultiSphereShape *pShape
            = reinterpret_cast<btMultiSphereShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btMultiSphereShape does not exist.",);
    ASSERT_CHK(pEnv, pShape->getShapeType() == MULTI_SPHERE_SHAPE_PROXYTYPE,);

    pShape->recalcLocalAabb();
}
