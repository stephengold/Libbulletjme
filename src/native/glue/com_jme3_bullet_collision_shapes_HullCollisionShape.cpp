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
#include "com_jme3_bullet_collision_shapes_HullCollisionShape.h"
#include "jmeBulletUtil.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_collision_shapes_HullCollisionShape
     * Method:    countHullVertices
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_HullCollisionShape_countHullVertices
    (JNIEnv * env, jobject object, jlong shapeId) {
        btConvexHullShape* hull
                = reinterpret_cast<btConvexHullShape*> (shapeId);
        if (hull == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btConvexHullShape does not exist.");
            return 0;
        }

        int count = hull->getNumPoints();
        return count;
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_HullCollisionShape
     * Method:    createShapeB
     * Signature: (Ljava/nio/ByteBuffer;I)J
     *
     * buffer contains float values: x,y,z for each vertex
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_HullCollisionShape_createShapeB
    (JNIEnv * env, jobject object, jobject buffer, jint numVertices) {
        jmeClasses::initJavaClasses(env);

        int n = numVertices;
        if (n < 1) {
            jclass newExc = env->FindClass("java/lang/IllegalArgumentException");
            env->ThrowNew(newExc, "numVertices must be positive");
            return 0L;
        }

        int numBytes = env->GetDirectBufferCapacity(buffer);
        if (numBytes < 12 * n) {
            jclass newExc = env->FindClass("java/lang/IllegalArgumentException");
            env->ThrowNew(newExc, "buffer too small");
            return 0L;
        }

        btConvexHullShape* shape = new btConvexHullShape();

        float* data = (float*) env->GetDirectBufferAddress(buffer);
        for (int i = 0; i < n; ++i) {
            int j = 3 * i;
            btVector3 vect = btVector3(data[j], data[j + 1], data[j + 2]);
            shape->addPoint(vect);
        }

        return reinterpret_cast<jlong> (shape);
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_HullCollisionShape
     * Method:    createShape
     * Signature: ([F)J
     *
     * buffer contains float values: x,y,z for each vertex
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_HullCollisionShape_createShape
    (JNIEnv *env, jobject object, jobject array) {
        jmeClasses::initJavaClasses(env);

        float* data = (float*) env->GetDirectBufferAddress(array);
        //TODO: capacity will not always be length!
        int length = env->GetDirectBufferCapacity(array) / 4;
        btConvexHullShape* shape = new btConvexHullShape();
        for (int i = 0; i < length; i += 3) {
            btVector3 vect = btVector3(data[i], data[i + 1], data[i + 2]);

            shape->addPoint(vect);
        }

        return reinterpret_cast<jlong> (shape);
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_HullCollisionShape
     * Method:    getHullVertices
     * Signature: (JLjava/nio/ByteBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_HullCollisionShape_getHullVertices
    (JNIEnv *env, jobject object, jlong shapeId, jobject buffer) {
        btConvexHullShape* hull
                = reinterpret_cast<btConvexHullShape*> (shapeId);
        if (hull == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The btConvexHullShape does not exist.");
            return;
        }
        jlong bytesCapacity = env->GetDirectBufferCapacity(buffer);
        int numVerts = hull->getNumPoints();
        long bytesNeeded = 12 * (long) numVerts;
        if (bytesNeeded > bytesCapacity) {
            jclass newExc
                    = env->FindClass("java/lang/IllegalArgumentException");
            env->ThrowNew(newExc, "The buffer provided is too small.");
            return;
        }

        const btVector3* vertexPtr = hull->getUnscaledPoints();
        float* writePtr = (float*) env->GetDirectBufferAddress(buffer);
        for (int i = 0; i < numVerts; ++i) {
            writePtr[0] = vertexPtr->m_floats[0];
            writePtr[1] = vertexPtr->m_floats[1];
            writePtr[2] = vertexPtr->m_floats[2];
            ++vertexPtr;
            writePtr += 3;
        }
    }

#ifdef __cplusplus
}
#endif
