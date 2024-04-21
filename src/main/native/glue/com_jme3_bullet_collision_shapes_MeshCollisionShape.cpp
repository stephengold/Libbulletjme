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
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    createShape
 * Signature: (ZZJ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_createShape
(JNIEnv *pEnv, jclass, jboolean quantized, jboolean buildBVH, jlong meshId) {
    jmeClasses::initJavaClasses(pEnv);

    btStridingMeshInterface *pMesh
            = reinterpret_cast<btStridingMeshInterface *> (meshId);
    NULL_CHK(pEnv, pMesh, "The btStridingMeshInterface does not exist.", 0)

    btBvhTriangleMeshShape *pShape
            = new btBvhTriangleMeshShape(pMesh, quantized, buildBVH); //dance016
    return reinterpret_cast<jlong> (pShape);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    hasBvh
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_hasBvh
(JNIEnv *pEnv, jclass, jlong shapeId) {
    btBvhTriangleMeshShape *pShape
            = reinterpret_cast<btBvhTriangleMeshShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btBvhTriangleMeshShape does not exist.", JNI_FALSE);
    ASSERT_CHK(pEnv, pShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE, JNI_FALSE);

    bool result = (pShape->getOptimizedBvh() != NULL);
    return result;
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
 * Method:    setOptimizedBvh
 * Signature: (JJLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_setOptimizedBvh
(JNIEnv *pEnv, jclass, jlong shapeId, jlong bvhId, jobject scaleVector) {
    btBvhTriangleMeshShape * const
            pShape = reinterpret_cast<btBvhTriangleMeshShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btBvhTriangleMeshShape does not exist.",);
    ASSERT_CHK(pEnv, pShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE,);

    btOptimizedBvh * const pBvh = reinterpret_cast<btOptimizedBvh *> (bvhId);
    NULL_CHK(pEnv, pBvh, "The btOptimizedBvh does not exist.",);

    btVector3 scaling;
    jmeBulletUtil::convert(pEnv, scaleVector, &scaling);
    EXCEPTION_CHK(pEnv,);

    pShape->setOptimizedBvh(pBvh, scaling);
}
