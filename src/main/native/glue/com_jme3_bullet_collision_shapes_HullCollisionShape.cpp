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
#include "jmeClasses.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_HullCollisionShape
 * Method:    countHullVertices
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_HullCollisionShape_countHullVertices
(JNIEnv *pEnv, jclass, jlong shapeId) {
    btConvexHullShape * const pShape
            = reinterpret_cast<btConvexHullShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btConvexHullShape does not exist.", 0);
    ASSERT_CHK(pEnv, pShape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE, 0);

    int count = pShape->getNumPoints();
    return count;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_HullCollisionShape
 * Method:    createShapeF
 * Signature: (Ljava/nio/FloatBuffer;I)J
 *
 * buffer contains float values: x,y,z for each vertex
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_HullCollisionShape_createShapeF
(JNIEnv *pEnv, jclass, jobject buffer, jint numVertices) {
    jmeClasses::initJavaClasses(pEnv);

    int n = numVertices;
    if (n < 1) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "numVertices must be positive");
        return 0L;
    }

    NULL_CHK(pEnv, buffer, "The buffer does not exist.", 0);
    const jlong capacityFloats = pEnv->GetDirectBufferCapacity(buffer);
    EXCEPTION_CHK(pEnv, 0);
    if (capacityFloats < 3 * n) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "The buffer is too small.");
        return 0L;
    }
    const jfloat * const pBuffer
            = (jfloat *) pEnv->GetDirectBufferAddress(buffer);
    NULL_CHK(pEnv, pBuffer, "The buffer is not direct.", 0);
    EXCEPTION_CHK(pEnv, 0);

    btConvexHullShape * const pShape = new btConvexHullShape(); //dance016

    for (int i = 0; i < n; ++i) {
        const int j = 3 * i;
        btVector3 vect
                = btVector3(pBuffer[j], pBuffer[j + 1], pBuffer[j + 2]);
        pShape->addPoint(vect);
    }

    pShape->optimizeConvexHull();

    return reinterpret_cast<jlong> (pShape);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_HullCollisionShape
 * Method:    getHullVerticesF
 * Signature: (JLjava/nio/FloatBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_HullCollisionShape_getHullVerticesF
(JNIEnv *pEnv, jclass, jlong shapeId, jobject storeBuffer) {
    const btConvexHullShape * const pShape
            = reinterpret_cast<btConvexHullShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btConvexHullShape does not exist.",)
    ASSERT_CHK(pEnv, pShape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE,);

    NULL_CHK(pEnv, storeBuffer, "The store buffer does not exist.",);
    const jlong capacityFloats = pEnv->GetDirectBufferCapacity(storeBuffer);
    EXCEPTION_CHK(pEnv,);
    int numVerts = pShape->getNumPoints();
    jlong floatsNeeded = 3 * (jlong) numVerts;
    if (floatsNeeded > capacityFloats) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "The store buffer is too small.");
        return;
    }
    jfloat *pWrite = (jfloat *) pEnv->GetDirectBufferAddress(storeBuffer);
    NULL_CHK(pEnv, pWrite, "The store buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);

    const btVector3 *pVertices = pShape->getUnscaledPoints();
    for (int i = 0; i < numVerts; ++i) {
        pWrite[0] = pVertices->m_floats[0];
        pWrite[1] = pVertices->m_floats[1];
        pWrite[2] = pVertices->m_floats[2];
        ++pVertices;
        pWrite += 3;
    }
}

/*
 * Class:     com_jme3_bullet_collision_shapes_HullCollisionShape
 * Method:    recalcAabb
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_HullCollisionShape_recalcAabb
(JNIEnv *pEnv, jclass, jlong shapeId) {
    btConvexHullShape * const pShape
            = reinterpret_cast<btConvexHullShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btConvexHullShape does not exist.",);
    ASSERT_CHK(pEnv, pShape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE,);

    pShape->recalcLocalAabb();
}
