/*
 * Copyright (c) 2022-2023 jMonkeyEngine
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

#include "jmeBulletUtil.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "com_jme3_bullet_collision_PersistentManifolds.h"

/*
 * Class:     com_jme3_bullet_collision_PersistentManifolds
 * Method:    countPoints
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_PersistentManifolds_countPoints
(JNIEnv *pEnv, jclass, jlong manifoldId) {
    const btPersistentManifold * const
            pManifold = reinterpret_cast<btPersistentManifold *> (manifoldId);
    NULL_CHK(pEnv, pManifold, "The btPersistentManifold does not exist.", 0);
    ASSERT_CHK(pEnv, pManifold->getObjectType() == BT_PERSISTENT_MANIFOLD_TYPE, 0);

    int result = pManifold->getNumContacts();
    ASSERT_CHK(pEnv, result >= 0, 0);
    ASSERT_CHK(pEnv, result <= MANIFOLD_CACHE_SIZE, 0);
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_PersistentManifolds
 * Method:    getBodyAId
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_PersistentManifolds_getBodyAId
(JNIEnv *pEnv, jclass, jlong manifoldId) {
    const btPersistentManifold * const
            pManifold = reinterpret_cast<btPersistentManifold *> (manifoldId);
    NULL_CHK(pEnv, pManifold, "The btPersistentManifold does not exist.", 0);
    ASSERT_CHK(pEnv, pManifold->getObjectType() == BT_PERSISTENT_MANIFOLD_TYPE, 0);

    const btCollisionObject * const result = pManifold->getBody0();
    NULL_CHK(pEnv, result, "The body A does not exist.", 0);
    return reinterpret_cast<jlong> (result);
}

/*
 * Class:     com_jme3_bullet_collision_PersistentManifolds
 * Method:    getBodyBId
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_PersistentManifolds_getBodyBId
(JNIEnv *pEnv, jclass, jlong manifoldId) {
    const btPersistentManifold * const
            pManifold = reinterpret_cast<btPersistentManifold *> (manifoldId);
    NULL_CHK(pEnv, pManifold, "The btPersistentManifold does not exist.", 0);
    ASSERT_CHK(pEnv, pManifold->getObjectType() == BT_PERSISTENT_MANIFOLD_TYPE, 0);

    const btCollisionObject * const result = pManifold->getBody1();
    NULL_CHK(pEnv, result, "The body B does not exist.", 0);
    return reinterpret_cast<jlong> (result);
}

/*
 * Class:     com_jme3_bullet_collision_PersistentManifolds
 * Method:    getPointId
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_PersistentManifolds_getPointId
(JNIEnv *pEnv, jclass, jlong manifoldId, jint pointIndex) {
    const btPersistentManifold * const
            pManifold = reinterpret_cast<btPersistentManifold *> (manifoldId);
    NULL_CHK(pEnv, pManifold, "The btPersistentManifold does not exist.", 0);
    ASSERT_CHK(pEnv, pManifold->getObjectType() == BT_PERSISTENT_MANIFOLD_TYPE, 0);

    int index = int(pointIndex);
    const btManifoldPoint * const result = &pManifold->getContactPoint(index);
    NULL_CHK(pEnv, result, "The contact point does not exist.", 0);
    return reinterpret_cast<jlong> (result);
}
