/*
 * Copyright (c) 2019-2023 jMonkeyEngine
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
#include "com_jme3_bullet_collision_shapes_Convex2dShape.h"
#include "jmeClasses.h"
#include "BulletCollision/CollisionShapes/btConvex2dShape.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_Convex2dShape
 * Method:    createShape
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_Convex2dShape_createShape
(JNIEnv *pEnv, jclass, jlong childShapeId) {
    jmeClasses::initJavaClasses(pEnv);

    btCollisionShape *pChild
            = reinterpret_cast<btCollisionShape *> (childShapeId);
    NULL_CHK(pEnv, pChild, "The child shape does not exist.", 0)
    if (!pChild->isConvex()) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "The btCollisionShape isn't convex.");
        return 0;
    }
    btConvexShape *pConvex = (btConvexShape *) pChild;

    btConvex2dShape *pShape = new btConvex2dShape(pConvex); //dance016
    return reinterpret_cast<jlong> (pShape);
}
