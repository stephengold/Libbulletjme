/*
 * Copyright (c) 2023 jMonkeyEngine
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
#include "com_jme3_bullet_collision_shapes_MinkowskiSum.h"
#include "jmeClasses.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_MinkowskiSum
 * Method:    createShape
 * Signature: (JJ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MinkowskiSum_createShape
(JNIEnv *pEnv, jclass, jlong shapeAId, jlong shapeBId) {
    jmeClasses::initJavaClasses(pEnv);

    const btCollisionShape *pShapeA
            = reinterpret_cast<btCollisionShape *> (shapeAId);
    NULL_CHK(pEnv, pShapeA, "Shape A does not exist.", 0)
    if (!pShapeA->isConvex()) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "Shape A isn't convex.");
        return 0;
    }
    const btConvexShape *pConvexA = (btConvexShape *) pShapeA;

    const btCollisionShape *pShapeB
            = reinterpret_cast<btCollisionShape *> (shapeBId);
    NULL_CHK(pEnv, pShapeB, "Shape B does not exist.", 0)
    if (!pShapeB->isConvex()) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "Shape B isn't convex.");
        return 0;
    }
    const btConvexShape *pConvexB = (btConvexShape *) pShapeB;

    btMinkowskiSumShape *pResult
            = new btMinkowskiSumShape(pConvexA, pConvexB); //dance016
    return reinterpret_cast<jlong> (pResult);
}
