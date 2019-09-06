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
#include "jmeBulletUtil.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
     * Method:    createShape
     * Signature: (J)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_createShape
    (JNIEnv *env, jobject object, jboolean isMemoryEfficient, jboolean buildBVH,
            jlong meshId) {
        jmeClasses::initJavaClasses(env);

        btStridingMeshInterface *pMesh
                = reinterpret_cast<btStridingMeshInterface *> (meshId);
        NULL_CHECK(pMesh, "The btStridingMeshInterface does not exist.", 0)

        btBvhTriangleMeshShape *pShape = new btBvhTriangleMeshShape(pMesh,
                isMemoryEfficient, buildBVH);

        return reinterpret_cast<jlong> (pShape);
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
     * Method:    finalizeBVH
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_finalizeBVH
    (JNIEnv *env, jobject object, jlong nativeBVHBufferId) {
        if (nativeBVHBufferId != 0) {
            void *pBuffer = reinterpret_cast<void *> (nativeBVHBufferId);
            btAlignedFree(pBuffer);
        }
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
     * Method:    recalcAabb
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_recalcAabb
    (JNIEnv *env, jobject object, jlong shapeId) {
        btBvhTriangleMeshShape *pShape
                = reinterpret_cast<btBvhTriangleMeshShape *> (shapeId);
        NULL_CHECK(pShape, "The btBvhTriangleMeshShape does not exist.",);
        btAssert(pShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE);

        pShape->recalcLocalAabb();
    }

    /*
     * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
     * Method:    saveBVH
     * Signature: (J)[B
     */
    JNIEXPORT jbyteArray JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_saveBVH
    (JNIEnv *env, jobject, jlong meshobj) {
        btBvhTriangleMeshShape *pMesh
                = reinterpret_cast<btBvhTriangleMeshShape *> (meshobj);
        NULL_CHECK(pMesh, "The btBvhTriangleMeshShape does not exist.", 0);
        btAssert(pMesh->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE);

        btOptimizedBvh *pBvh = pMesh->getOptimizedBvh();
        unsigned int ssize = pBvh->calculateSerializeBufferSize();
        char *pBuffer = (char *) btAlignedAlloc(ssize, 16);
        bool success = pBvh->serialize(pBuffer, ssize, true);
        if (!success) {
            jclass newExc = env->FindClass("java/lang/RuntimeException");
            env->ThrowNew(newExc, "Unable to serialize, native error reported");
        }

        jbyteArray byteArray = env->NewByteArray(ssize);
        env->SetByteArrayRegion(byteArray, 0, ssize, (jbyte *) pBuffer);
        btAlignedFree(pBuffer);

        return byteArray;
    };

    /*
     * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
     * Method:    setBVH
     * Signature: ([BJ)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_setBVH
    (JNIEnv *env, jobject, jbyteArray bytearray, jlong meshobj) {
        int len = env->GetArrayLength(bytearray);
        void *pBuffer = btAlignedAlloc(len, 16);
        env->GetByteArrayRegion(bytearray, 0, len,
                reinterpret_cast<jbyte *> (pBuffer));

        btOptimizedBvh *pBvh
                = btOptimizedBvh::deSerializeInPlace(pBuffer, len, true);
        btBvhTriangleMeshShape *pShape
                = reinterpret_cast<btBvhTriangleMeshShape *> (meshobj);
        NULL_CHECK(pShape, "The btBvhTriangleMeshShape does not exist.", 0);
        pShape->setOptimizedBvh(pBvh);

        return reinterpret_cast<jlong> (pBuffer);
    };

#ifdef __cplusplus
}
#endif