/*
 * Copyright (c) 2019-2020 jMonkeyEngine
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
#include "com_jme3_bullet_collision_shapes_infos_IndexedMesh.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    createByte
 * Signature: (Ljava/nio/ByteBuffer;Ljava/nio/FloatBuffer;IIII)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_createByte
(JNIEnv *pEnv, jclass, jobject byteBuffer, jobject floatBuffer,
        jint numTriangles, jint numVertices, jint vertexStride,
        jint indexStride) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, byteBuffer, "The index buffer does not exist.", 0);
    const unsigned char * const pIndices
            = (unsigned char *) pEnv->GetDirectBufferAddress(byteBuffer);
    NULL_CHK(pEnv, pIndices, "The index buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, floatBuffer, "The position buffer does not exist.", 0);
    const unsigned char * const pVertices
            = (unsigned char *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pVertices, "The position buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    btIndexedMesh * const pMesh = new btIndexedMesh(); //dance020
    pMesh->m_indexType = PHY_UCHAR;
    pMesh->m_vertexType = PHY_FLOAT;
    pMesh->m_triangleIndexBase = pIndices;
    pMesh->m_vertexBase = pVertices;
    pMesh->m_numTriangles = numTriangles;
    pMesh->m_numVertices = numVertices;
    pMesh->m_vertexStride = vertexStride;
    pMesh->m_triangleIndexStride = indexStride;

    return reinterpret_cast<jlong> (pMesh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    createInt
 * Signature: (Ljava/nio/IntBuffer;Ljava/nio/FloatBuffer;IIII)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_createInt
(JNIEnv *pEnv, jclass, jobject intBuffer, jobject floatBuffer,
        jint numTriangles, jint numVertices, jint vertexStride,
        jint indexStride) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, intBuffer, "The index buffer does not exist.", 0);
    const unsigned char * const pIndices
            = (unsigned char *) pEnv->GetDirectBufferAddress(intBuffer);
    NULL_CHK(pEnv, pIndices, "The index buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, floatBuffer, "The position buffer does not exist.", 0);
    const unsigned char * const pVertices
            = (unsigned char *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pVertices, "The position buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    btIndexedMesh * const pMesh = new btIndexedMesh(); //dance020
    pMesh->m_indexType = PHY_INTEGER;
    pMesh->m_vertexType = PHY_FLOAT;
    pMesh->m_triangleIndexBase = pIndices;
    pMesh->m_vertexBase = pVertices;
    pMesh->m_numTriangles = numTriangles;
    pMesh->m_numVertices = numVertices;
    pMesh->m_vertexStride = vertexStride;
    pMesh->m_triangleIndexStride = indexStride;

    return reinterpret_cast<jlong> (pMesh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    createShort
 * Signature: (Ljava/nio/ShortBuffer;Ljava/nio/FloatBuffer;IIII)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_createShort
(JNIEnv *pEnv, jclass, jobject shortBuffer, jobject floatBuffer,
        jint numTriangles, jint numVertices, jint vertexStride,
        jint indexStride) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, shortBuffer, "The index buffer does not exist.", 0);
    const unsigned char * const pIndices
            = (unsigned char *) pEnv->GetDirectBufferAddress(shortBuffer);
    NULL_CHK(pEnv, pIndices, "The index buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    NULL_CHK(pEnv, floatBuffer, "The position buffer does not exist.", 0);
    const unsigned char * const pVertices
            = (unsigned char *) pEnv->GetDirectBufferAddress(floatBuffer);
    NULL_CHK(pEnv, pVertices, "The position buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    btIndexedMesh * const pMesh = new btIndexedMesh(); //dance020
    pMesh->m_indexType = PHY_SHORT;
    pMesh->m_vertexType = PHY_FLOAT;
    pMesh->m_triangleIndexBase = pIndices;
    pMesh->m_vertexBase = pVertices;
    pMesh->m_numTriangles = numTriangles;
    pMesh->m_numVertices = numVertices;
    pMesh->m_vertexStride = vertexStride;
    pMesh->m_triangleIndexStride = indexStride;

    return reinterpret_cast<jlong> (pMesh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_IndexedMesh
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_finalizeNative
(JNIEnv *pEnv, jclass, jlong meshId) {
    btIndexedMesh * const pMesh = reinterpret_cast<btIndexedMesh *> (meshId);
    NULL_CHK(pEnv, pMesh, "The btIndexedMesh does not exist.",);

    delete pMesh; //dance020
}
