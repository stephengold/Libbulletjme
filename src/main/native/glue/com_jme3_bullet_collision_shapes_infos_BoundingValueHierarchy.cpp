/*
 * Copyright (c) 2020-2024 jMonkeyEngine
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
#ifdef _WIN32
#include <Winsock2.h> // for htons()
#else
#include <arpa/inet.h> // for htons()
#endif
#include "jmeBulletUtil.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    deSerialize
 * Signature: ([B)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_deSerialize
(JNIEnv *pEnv, jclass, jbyteArray bytearray) {
    int len = pEnv->GetArrayLength(bytearray);
    EXCEPTION_CHK(pEnv, 0);
    void *pBuffer = btAlignedAlloc(len, 16); //dance035
    pEnv->GetByteArrayRegion(bytearray, 0, len,
            reinterpret_cast<jbyte *> (pBuffer));
    EXCEPTION_CHK(pEnv, 0);

    bool swapEndian = (htons(1) != 1);
    btOptimizedBvh * const
            pBvh = btOptimizedBvh::deSerializeInPlace(pBuffer, len, swapEndian);
    jlong result = reinterpret_cast<jlong> (pBvh);
#ifdef _DEBUG
    /*
     * sanity checks:
     */
    unsigned int bufferSize = pBvh->calculateSerializeBufferSize();
    btAssert(bufferSize == len);

    jlong bufferId = reinterpret_cast<jlong> (pBuffer);
    btAssert(bufferId == result);

    pBvh->checkSanity();
#endif

    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_finalizeNative
(JNIEnv *pEnv, jclass, jlong bvhId) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.",);
    pBvh->checkSanity();

    pBvh->~btOptimizedBvh();

    void *pBuffer = reinterpret_cast<void *> (bvhId);
    btAlignedFree(pBuffer); //dance035
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getAabb
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getAabb
(JNIEnv *pEnv, jclass, jlong bvhId, jobject storeMinima, jobject storeMaxima) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.",);
    NULL_CHK(pEnv, storeMaxima, "The storeMaxima does not exist.",);
    NULL_CHK(pEnv, storeMinima, "The storeMinima does not exist.",);

    const btVector3& aabbMin = pBvh->getAabbMin();
    jmeBulletUtil::convert(pEnv, &aabbMin, storeMinima);
    EXCEPTION_CHK(pEnv,);

    const btVector3& aabbMax = pBvh->getAabbMax();
    jmeBulletUtil::convert(pEnv, &aabbMax, storeMaxima);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getEscapeIndex
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getEscapeIndex
(JNIEnv *pEnv, jclass, jlong bvhId, jint nodeIndex) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    int result = -1;
    if (!pBvh->isLeafNode(nodeIndex)) {
        result = pBvh->getEscapeIndex(nodeIndex);
    }

    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getNumLeafNodes
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getNumLeafNodes
(JNIEnv *pEnv, jclass, jlong bvhId) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    int result = pBvh->getNumLeafNodes();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getNumNodes
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getNumNodes
(JNIEnv *pEnv, jclass, jlong bvhId) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    int result = pBvh->getNumNodes();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getNumSubtreeHeaders
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getNumSubtreeHeaders
(JNIEnv *pEnv, jclass, jlong bvhId) {
    btOptimizedBvh * const pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    BvhSubtreeInfoArray &array = pBvh->getSubtreeInfoArray();
    int result = array.size();

    return result;
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

    btOptimizedBvh *pBvh = pShape->getOptimizedBvh();
    if (pBvh == NULL) {
        pShape->buildOptimizedBvh();
        pBvh = pShape->getOptimizedBvh();
        btAssert(pBvh);
    }
    pBvh->checkSanity();

    return reinterpret_cast<jlong> (pBvh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getPartId
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getPartId
(JNIEnv *pEnv, jclass, jlong bvhId, jint nodeIndex) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    int result = -1;
    if (pBvh->isLeafNode(nodeIndex)) {
        result = pBvh->getPartId(nodeIndex);
    }

    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getQuantization
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getQuantization
(JNIEnv *pEnv, jclass, jlong bvhId, jobject storeVector) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.",);
    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);

    const btVector3& quantization = pBvh->getQuantization();
    jmeBulletUtil::convert(pEnv, &quantization, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getTraversalMode
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getTraversalMode
(JNIEnv *pEnv, jclass, jlong bvhId) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    int result = pBvh->getTraversalMode();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getTriangleIndex
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getTriangleIndex
(JNIEnv *pEnv, jclass, jlong bvhId, jint nodeIndex) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    int result = -1;
    if (pBvh->isLeafNode(nodeIndex)) {
        result = pBvh->getTriangleIndex(nodeIndex);
    }

    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    isCompressed
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_isCompressed
(JNIEnv *pEnv, jclass, jlong bvhId) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", JNI_FALSE);
    pBvh->checkSanity();

    bool result = pBvh->isQuantized();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    isLeafNode
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_isLeafNode
(JNIEnv *pEnv, jclass, jlong bvhId, jint nodeIndex) {
    const btOptimizedBvh * const
            pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.", 0);

    bool result = pBvh->isLeafNode(nodeIndex);
    return result;
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
    pBvh->checkSanity();

    unsigned int bufferSize = pBvh->calculateSerializeBufferSize();
    char *pBuffer = (char *) btAlignedAlloc(bufferSize, 16); //dance015
    bool swapEndian = (htons(1) != 1);
    bool success = pBvh->serialize(pBuffer, bufferSize, swapEndian);
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

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    setTraversalMode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_setTraversalMode
(JNIEnv *pEnv, jclass, jlong bvhId, jint mode) {
    btOptimizedBvh * const pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.",);

    enum btQuantizedBvh::btTraversalMode traversalMode
            = static_cast<enum btQuantizedBvh::btTraversalMode> (mode);
    btAssert(traversalMode == btQuantizedBvh::TRAVERSAL_STACKLESS ||
            traversalMode == btQuantizedBvh::TRAVERSAL_STACKLESS_CACHE_FRIENDLY ||
            traversalMode == btQuantizedBvh::TRAVERSAL_RECURSIVE);

    pBvh->setTraversalMode(traversalMode);
}