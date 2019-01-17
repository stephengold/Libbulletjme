/*
 * Copyright (c) 2018-2019 jMonkeyEngine
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

/**
 * Author: Stephen Gold
 */
#include "com_jme3_bullet_collision_shapes_MultiSphere.h"
#include "jmeBulletUtil.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_collision_shapes_MultiSphere
     * Method:    createShapeB
     * Signature: (Ljava/nio/ByteBuffer;I)J
     *
     * buffer contains float values: x,y,z,r for each sphere
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MultiSphere_createShapeB
    (JNIEnv *env, jobject object, jobject buffer, jint numSpheres) {
        jmeClasses::initJavaClasses(env);

        int n = numSpheres;
        if (n < 1) {
            jclass newExc = env->FindClass("java/lang/IllegalArgumentException");
            env->ThrowNew(newExc, "numSpheres must be positive");
            return 0L;
        }

        btAlignedObjectArray<btVector3> centers;
        centers.resize(n);
        btAlignedObjectArray<btScalar> radii;
        radii.resize(n);

        int numBytes = env->GetDirectBufferCapacity(buffer);
        if (numBytes < 16 * n) {
            jclass newExc = env->FindClass("java/lang/IllegalArgumentException");
            env->ThrowNew(newExc, "buffer too small");
            return 0L;
        }

        float* data = (float*) env->GetDirectBufferAddress(buffer);
        for (int i = 0; i < n; ++i) {
            int j = 4 * i;
            centers[i] = btVector3(data[j], data[j + 1], data[j + 2]);
            radii[i] = data[j + 3];
            if (!(radii[i] >= 0)) {
                jclass newExc
                        = env->FindClass("java/lang/IllegalArgumentException");
                env->ThrowNew(newExc, "Illegal radius for btMultiSphereShape.");
                return 0L;
            }
        }

        btMultiSphereShape* shape
                = new btMultiSphereShape(&centers[0], &radii[0], n);

        return reinterpret_cast<jlong> (shape);
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_MultiSphere
     * Method:    createShape
     * Signature: (F)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MultiSphere_createShape
    (JNIEnv *env, jobject object, jobjectArray centers, jfloatArray radii, jint numSpheres) {
        jmeClasses::initJavaClasses(env);

        int n = numSpheres;
        btVector3* positions = new btVector3[n];
        for (int i = 0; i < n; ++i) {
            jobject center = env->GetObjectArrayElement(centers, i);
            jmeBulletUtil::convert(env, center, &positions[i]);
        }

        btScalar* radi = env->GetFloatArrayElements(radii, 0);

        btMultiSphereShape* shape = new btMultiSphereShape(positions, radi, n);
        return reinterpret_cast<jlong> (shape);
    }

#ifdef __cplusplus
}
#endif
