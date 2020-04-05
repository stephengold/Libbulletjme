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
#include "com_jme3_bullet_collision_shapes_infos_CompoundMesh.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_CompoundMesh
 * Method:    addIndexedMesh
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_CompoundMesh_addIndexedMesh
(JNIEnv *env, jobject object, jlong compoundMeshId, jlong submeshId) {
    btTriangleIndexVertexArray *pMesh
            = reinterpret_cast<btTriangleIndexVertexArray *> (compoundMeshId);
    NULL_CHK(env, pMesh, "The btTriangleIndexVertexArray does not exist.",);

    btIndexedMesh *pSubmesh = reinterpret_cast<btIndexedMesh *> (submeshId);
    NULL_CHK(env, pSubmesh, "The btIndexedMesh does not exist.",);

    PHY_ScalarType indexType = pSubmesh->m_indexType;
    pMesh->addIndexedMesh(*pSubmesh, indexType);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_CompoundMesh
 * Method:    createEmptyTiva
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_CompoundMesh_createEmptyTiva
(JNIEnv *env, jobject object) {
    jmeClasses::initJavaClasses(env);
    btTriangleIndexVertexArray *pMesh = new btTriangleIndexVertexArray();
    return reinterpret_cast<jlong> (pMesh);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_CompoundMesh
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_CompoundMesh_finalizeNative
(JNIEnv *env, jobject object, jlong meshId) {
    btTriangleIndexVertexArray *pMesh
            = reinterpret_cast<btTriangleIndexVertexArray *> (meshId);
    NULL_CHK(env, pMesh, "The btTriangleIndexVertexArray does not exist.",);

    delete pMesh;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_CompoundMesh
 * Method:    getScaling
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_CompoundMesh_getScaling
(JNIEnv *env, jobject object, jlong meshId, jobject storeVector) {
    btTriangleIndexVertexArray *pMesh
            = reinterpret_cast<btTriangleIndexVertexArray *> (meshId);
    NULL_CHK(env, pMesh, "The btTriangleIndexVertexArray does not exist.",);
    NULL_CHK(env, storeVector, "The store vector does not exist.",);

    const btVector3 *pScale = &pMesh->getScaling();
    jmeBulletUtil::convert(env, pScale, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_CompoundMesh
 * Method:    setScaling
 * Signature: (JFFF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_CompoundMesh_setScaling
(JNIEnv *env, jobject object, jlong meshId, jfloat xScale, jfloat yScale,
        jfloat zScale) {
    btTriangleIndexVertexArray *pMesh
            = reinterpret_cast<btTriangleIndexVertexArray *> (meshId);
    NULL_CHK(env, pMesh, "The btTriangleIndexVertexArray does not exist.",);

    btVector3 scale;
    scale.setX(xScale);
    scale.setY(yScale);
    scale.setZ(zScale);
    pMesh->setScaling(scale);
}
