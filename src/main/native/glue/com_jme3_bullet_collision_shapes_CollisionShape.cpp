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
#include "com_jme3_bullet_collision_shapes_CollisionShape.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_finalizeNative
(JNIEnv *pEnv, jobject, jlong shapeId) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.",);

    delete pShape;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    getAabb
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_getAabb
(JNIEnv *pEnv, jobject, jlong shapeId, jobject location,
        jobject orientation, jobject storeMinima, jobject storeMaxima) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.",);

    btTransform trans;
    jmeBulletUtil::convert(pEnv, location, &trans.getOrigin());
    jmeBulletUtil::convert(pEnv, orientation, &trans.getBasis());

    btVector3 aabbMin;
    btVector3 aabbMax;
    pShape->getAabb(trans, aabbMin, aabbMax);

    jmeBulletUtil::convert(pEnv, &aabbMin, storeMinima);
    jmeBulletUtil::convert(pEnv, &aabbMax, storeMaxima);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    getLocalScaling
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_getLocalScaling
(JNIEnv *pEnv, jobject, jlong shapeId, jobject storeVector) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.",);
    NULL_CHK(pEnv, storeVector, "The storeVector does not exist.",);

    jmeBulletUtil::convert(pEnv, &pShape->getLocalScaling(), storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    getMargin
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_getMargin
(JNIEnv *pEnv, jobject, jlong shapeId) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", 0);

    return pShape->getMargin();
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    getShapeType
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_getShapeType
(JNIEnv *pEnv, jobject, jlong shapeId) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", 0);

    int shapeType = pShape->getShapeType();
    return (jint) shapeType;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isConcave
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isConcave
(JNIEnv *pEnv, jobject, jlong shapeId) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", JNI_FALSE);

    return pShape->isConcave();
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isConvex
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isConvex
(JNIEnv *pEnv, jobject, jlong shapeId) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", JNI_FALSE);

    return pShape->isConvex();
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isInfinite
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isInfinite
(JNIEnv *pEnv, jobject, jlong shapeId) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", JNI_FALSE);

    return pShape->isInfinite();
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isNonMoving
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isNonMoving
(JNIEnv *pEnv, jobject, jlong shapeId) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", JNI_FALSE);

    return pShape->isNonMoving();
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    isPolyhedral
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_isPolyhedral
(JNIEnv *pEnv, jobject, jlong shapeId) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", JNI_FALSE);

    return pShape->isPolyhedral();
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    setLocalScaling
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_setLocalScaling
(JNIEnv *pEnv, jobject, jlong shapeId, jobject scaleVector) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.",);
    NULL_CHK(pEnv, scaleVector, "The scale vector does not exist.",);

    btVector3 scl;
    jmeBulletUtil::convert(pEnv, scaleVector, &scl);

    pShape->setLocalScaling(scl);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CollisionShape
 * Method:    setMargin
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CollisionShape_setMargin
(JNIEnv *pEnv, jobject, jlong shapeId, jfloat newMargin) {
    btCollisionShape *pShape
            = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.",);

    pShape->setMargin(newMargin);
}
