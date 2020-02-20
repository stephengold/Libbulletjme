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
 * Method:    createInt
 * Signature: (Ljava/nio/IntBuffer;Ljava/nio/FloatBuffer;IIII)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_createInt
(JNIEnv *env, jobject object, jobject intBuffer, jobject floatBuffer,
        jint numTriangles, jint numVertices, jint vertexStride,
        jint indexStride) {
    jmeClasses::initJavaClasses(env);

    NULL_CHECK(intBuffer, "The index buffer does not exist.", 0);
    const unsigned char * const pIndices
            = (unsigned char *) env->GetDirectBufferAddress(intBuffer);
    NULL_CHECK(pIndices, "The index buffer is not direct.", 0);

    NULL_CHECK(floatBuffer, "The position buffer does not exist.", 0);
    const unsigned char * const pVertices
            = (unsigned char *) env->GetDirectBufferAddress(floatBuffer);
    NULL_CHECK(pVertices, "The position buffer is not direct.", 0);

    btIndexedMesh * const pMesh = new btIndexedMesh();
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
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_IndexedMesh_finalizeNative
(JNIEnv *env, jobject object, jlong meshId) {
    btIndexedMesh *pMesh = reinterpret_cast<btIndexedMesh *> (meshId);
    NULL_CHECK(pMesh, "The btIndexedMesh does not exist.",);

    delete pMesh;
}
