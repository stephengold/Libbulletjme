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

/*
 * Author: Stephen Gold
 */
#include "com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy.h"
#include "jmeBulletUtil.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"

JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_deSerialize
(JNIEnv *pEnv, jclass, jbyteArray bytearray) {
    int len = pEnv->GetArrayLength(bytearray);
    EXCEPTION_CHK(pEnv, 0);
    void *pBuffer = btAlignedAlloc(len, 16); //dance035
    pEnv->GetByteArrayRegion(bytearray, 0, len,
            reinterpret_cast<jbyte *> (pBuffer));
    EXCEPTION_CHK(pEnv, 0);

    btOptimizedBvh * const
            pBvh = btOptimizedBvh::deSerializeInPlace(pBuffer, len, true);
    return reinterpret_cast<jlong> (pBvh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_finalizeNative
(JNIEnv *, jclass, jlong bvhId) {
    if (bvhId != 0) {
        void *pBuffer = reinterpret_cast<void *> (bvhId);
        btAlignedFree(pBuffer); //dance035
    }
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getOptimizedBvh
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getOptimizedBvh
(JNIEnv *pEnv, jclass, jlong shapeId) {
    btBvhTriangleMeshShape * const
            pShape = reinterpret_cast<btBvhTriangleMeshShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btBvhTriangleMeshShape does not exist.", 0);
    ASSERT_CHK(pEnv, pShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE, 0);

    btOptimizedBvh * const pBvh = pShape->getOptimizedBvh();
    return reinterpret_cast<jlong> (pBvh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    serialize
 * Signature: (J)[B
 */
JNIEXPORT jbyteArray JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_serialize
(JNIEnv *pEnv, jclass, jlong bvhId) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    unsigned int bufferSize = pBvh->calculateSerializeBufferSize();
    char *pBuffer = (char *) btAlignedAlloc(bufferSize, 16); //dance015
    bool success = pBvh->serialize(pBuffer, bufferSize, true);
    if (!success) {
        pEnv->ThrowNew(jmeClasses::RuntimeException,
                "Unable to serialize, native error reported");
        return 0;
    }

    jbyteArray byteArray = pEnv->NewByteArray(bufferSize);
    EXCEPTION_CHK(pEnv, 0);
    pEnv->SetByteArrayRegion(byteArray, 0, bufferSize, (jbyte *) pBuffer);
    EXCEPTION_CHK(pEnv, 0);
    btAlignedFree(pBuffer); //dance015

    return byteArray;
}