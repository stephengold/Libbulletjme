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
#include "com_jme3_bullet_collision_shapes_MeshCollisionShape.h"
#include "jmeClasses.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    createShape
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_createShape
(JNIEnv *pEnv, jclass, jboolean isMemoryEfficient, jboolean buildBVH,
        jlong meshId) {
    jmeClasses::initJavaClasses(pEnv);

    btStridingMeshInterface *pMesh
            = reinterpret_cast<btStridingMeshInterface *> (meshId);
    NULL_CHK(pEnv, pMesh, "The btStridingMeshInterface does not exist.", 0)

    btBvhTriangleMeshShape *
            pShape = new btBvhTriangleMeshShape(pMesh, isMemoryEfficient,
            buildBVH); //dance016
    return reinterpret_cast<jlong> (pShape);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    recalcAabb
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_recalcAabb
(JNIEnv *pEnv, jclass, jlong shapeId) {
    btBvhTriangleMeshShape *pShape
            = reinterpret_cast<btBvhTriangleMeshShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btBvhTriangleMeshShape does not exist.",);
    ASSERT_CHK(pEnv, pShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE,);

    pShape->recalcLocalAabb();
}

/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    saveBVH
 * Signature: (J)[B
 */
JNIEXPORT jbyteArray JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_saveBVH
(JNIEnv *pEnv, jclass, jlong meshobj) {
    btBvhTriangleMeshShape *pMesh
            = reinterpret_cast<btBvhTriangleMeshShape *> (meshobj);
    NULL_CHK(pEnv, pMesh, "The btBvhTriangleMeshShape does not exist.", 0);
    ASSERT_CHK(pEnv, pMesh->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE, 0);

    btOptimizedBvh *pBvh = pMesh->getOptimizedBvh();
    unsigned int ssize = pBvh->calculateSerializeBufferSize();
    char *pBuffer = (char *) btAlignedAlloc(ssize, 16); //dance015
    bool success = pBvh->serialize(pBuffer, ssize, true);
    if (!success) {
        pEnv->ThrowNew(jmeClasses::RuntimeException,
                "Unable to serialize, native error reported");
        return 0;
    }

    jbyteArray byteArray = pEnv->NewByteArray(ssize);
    EXCEPTION_CHK(pEnv, 0);
    pEnv->SetByteArrayRegion(byteArray, 0, ssize, (jbyte *) pBuffer);
    EXCEPTION_CHK(pEnv, 0);
    btAlignedFree(pBuffer); //dance015

    return byteArray;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    setOptimizedBvh
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_setOptimizedBvh
(JNIEnv *pEnv, jclass, jlong shapeId, jlong bvhId) {
    btBvhTriangleMeshShape * const
            pShape = reinterpret_cast<btBvhTriangleMeshShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btBvhTriangleMeshShape does not exist.",);
    ASSERT_CHK(pEnv, pShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE,);

    btOptimizedBvh * const pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.",);

    pShape->setOptimizedBvh(pBvh);
}
