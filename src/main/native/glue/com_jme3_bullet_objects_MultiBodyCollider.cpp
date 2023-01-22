/*
 * Copyright (c) 2020-2023 jMonkeyEngine
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
#include "btMultiBodyLinkCollider.h"
#include "com_jme3_bullet_objects_MultiBodyCollider.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_objects_MultiBodyCollider
 * Method:    createCollider
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_MultiBodyCollider_createCollider
(JNIEnv *pEnv, jclass, jlong multiBodyId, jint linkIndex) {
    jmeClasses::initJavaClasses(pEnv);

    btMultiBody * const
            pMultiBody = reinterpret_cast<btMultiBody *> (multiBodyId);
    NULL_CHK(pEnv, pMultiBody, "The btMultiBody does not exist.", 0);

    int link = (int) linkIndex;
    btMultiBodyLinkCollider *
            pCollider = new btMultiBodyLinkCollider(pMultiBody, link); //dance014
    return reinterpret_cast<jlong> (pCollider);
}

/*
 * Class:     com_jme3_bullet_objects_MultiBodyCollider
 * Method:    setPhysicsLocation
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_MultiBodyCollider_setPhysicsLocation
(JNIEnv *pEnv, jclass, jlong colliderId, jobject locationVector) {
    btMultiBodyLinkCollider * const pCollider
            = reinterpret_cast<btMultiBodyLinkCollider *> (colliderId);
    NULL_CHK(pEnv, pCollider, "The btMultiBodyLinkCollider does not exist.",)
    ASSERT_CHK(pEnv, pCollider->getInternalType()
            & btCollisionObject::CO_FEATHERSTONE_LINK,);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)

    btVector3 * const pLocation = &pCollider->getWorldTransform().getOrigin();
    jmeBulletUtil::convert(pEnv, locationVector, pLocation);
}

/*
 * Class:     com_jme3_bullet_objects_MultiBodyCollider
 * Method:    setPhysicsLocationDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_MultiBodyCollider_setPhysicsLocationDp
(JNIEnv *pEnv, jclass, jlong colliderId, jobject locationVector) {
    btMultiBodyLinkCollider * const pCollider
            = reinterpret_cast<btMultiBodyLinkCollider *> (colliderId);
    NULL_CHK(pEnv, pCollider, "The btMultiBodyLinkCollider does not exist.",)
    ASSERT_CHK(pEnv, pCollider->getInternalType()
            & btCollisionObject::CO_FEATHERSTONE_LINK,);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)

    btVector3 * const pLocation = &pCollider->getWorldTransform().getOrigin();
    jmeBulletUtil::convertDp(pEnv, locationVector, pLocation);
}

/*
 * Class:     com_jme3_bullet_objects_MultiBodyCollider
 * Method:    setPhysicsRotation
 * Signature: (JLcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_MultiBodyCollider_setPhysicsRotation
(JNIEnv *pEnv, jclass, jlong colliderId, jobject rotationMatrix) {
    btMultiBodyLinkCollider * const pCollider
            = reinterpret_cast<btMultiBodyLinkCollider *> (colliderId);
    NULL_CHK(pEnv, pCollider, "The btMultiBodyLinkCollider does not exist.",)
    ASSERT_CHK(pEnv, pCollider->getInternalType()
            & btCollisionObject::CO_FEATHERSTONE_LINK,);

    NULL_CHK(pEnv, rotationMatrix, "The rotation matrix does not exist.",)

    btMatrix3x3 * const pRotation = &pCollider->getWorldTransform().getBasis();
    jmeBulletUtil::convert(pEnv, rotationMatrix, pRotation);
}

/*
 * Class:     com_jme3_bullet_objects_MultiBodyCollider
 * Method:    setPhysicsRotationDp
 * Signature: (JLcom/simsilica/mathd/Matrix3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_MultiBodyCollider_setPhysicsRotationDp
(JNIEnv *pEnv, jclass, jlong colliderId, jobject rotationMatrix) {
    btMultiBodyLinkCollider * const pCollider
            = reinterpret_cast<btMultiBodyLinkCollider *> (colliderId);
    NULL_CHK(pEnv, pCollider, "The btMultiBodyLinkCollider does not exist.",)
    ASSERT_CHK(pEnv, pCollider->getInternalType()
            & btCollisionObject::CO_FEATHERSTONE_LINK,);

    NULL_CHK(pEnv, rotationMatrix, "The rotation matrix does not exist.",)

    btMatrix3x3 * const pRotation = &pCollider->getWorldTransform().getBasis();
    jmeBulletUtil::convertDp(pEnv, rotationMatrix, pRotation);
}

