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
#include "com_jme3_bullet_collision_shapes_CompoundCollisionShape.h"
#include "jmeBulletUtil.h"

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    addChildShape
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_addChildShape
(JNIEnv *env, jobject object, jlong compoundShapeId, jlong childShapeId,
        jobject offsetVector, jobject rotationMatrix) {
    btCompoundShape *pCompound
            = reinterpret_cast<btCompoundShape *> (compoundShapeId);
    NULL_CHK(env, pCompound, "The btCompoundShape does not exist.",)
    btAssert(pCompound->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);

    btCollisionShape *pChild
            = reinterpret_cast<btCollisionShape *> (childShapeId);
    NULL_CHK(env, pChild, "The child shape does not exist.",)

    NULL_CHK(env, offsetVector, "The offset vector does not exist.",)
    NULL_CHK(env, rotationMatrix, "The rotation matrix does not exist.",)
    btTransform trans;
    jmeBulletUtil::convert(env, offsetVector, &trans.getOrigin());
    jmeBulletUtil::convert(env, rotationMatrix, &trans.getBasis());

    pCompound->addChildShape(trans, pChild);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    calculatePrincipalAxisTransform
 * Signature: (JLjava/nio/FloatBuffer;Lcom/jme3/math/Transform;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_calculatePrincipalAxisTransform
(JNIEnv *env, jobject object, jlong shapeId, jobject massBuffer,
        jobject storeTransform, jobject storeInertia) {
    const btCompoundShape * const pShape
            = reinterpret_cast<btCompoundShape *> (shapeId);
    NULL_CHK(env, pShape, "The btCompoundShape does not exist.",)
    btAssert(pShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);

    NULL_CHK(env, massBuffer, "The mass buffer does not exist.",);
    const jfloat * const pBuffer
            = (jfloat *) env->GetDirectBufferAddress(massBuffer);
    NULL_CHK(env, pBuffer, "The mass buffer is not direct.",);
    const jlong capacity = env->GetDirectBufferCapacity(massBuffer);

    const int numChildren = pShape->getNumChildShapes();
    btTransform principal;
    btVector3 inertia;

    btScalar *pMasses = new btScalar[numChildren];
    for (int i = 0; i < numChildren && i < capacity; ++i) {
        pMasses[i] = pBuffer[i];
    }
    pShape->calculatePrincipalAxisTransform(pMasses, principal, inertia);
    delete[] pMasses;

    jmeBulletUtil::convert(env, &principal, storeTransform);
    jmeBulletUtil::convert(env, &inertia, storeInertia);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    countChildren
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_countChildren
(JNIEnv *env, jobject object, jlong compoundId) {
    const btCompoundShape * const pShape
            = reinterpret_cast<btCompoundShape *> (compoundId);
    NULL_CHK(env, pShape, "The btCompoundShape does not exist.", 0)
    btAssert(pShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);

    const int result = pShape->getNumChildShapes();

    return (jint) result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    createShape2
 * Signature: (ZI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_createShape2
(JNIEnv *env, jobject object, jboolean dynamicAabbTree,
        jint initialCapacity) {
    jmeClasses::initJavaClasses(env);

    bool enableDynamicAabbTree = (bool)dynamicAabbTree;
    const int initialChildCapacity = (int) initialCapacity;
    btCompoundShape *pShape = new btCompoundShape(enableDynamicAabbTree,
            initialChildCapacity);

    return reinterpret_cast<jlong> (pShape);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    recalcAabb
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_recalcAabb
(JNIEnv *env, jobject object, jlong shapeId) {
    btCompoundShape *pShape = reinterpret_cast<btCompoundShape *> (shapeId);
    NULL_CHK(env, pShape, "The btCompoundShape does not exist.",);
    btAssert(pShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);

    pShape->recalculateLocalAabb();
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    removeChildShape
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_removeChildShape
(JNIEnv *env, jobject object, jlong compoundShapeId, jlong childShapeId) {
    btCompoundShape * const pCompound
            = reinterpret_cast<btCompoundShape *> (compoundShapeId);
    NULL_CHK(env, pCompound, "The btCompoundShape does not exist.",)
    btAssert(pCompound->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);

    btCollisionShape * const pChild
            = reinterpret_cast<btCollisionShape *> (childShapeId);
    NULL_CHK(env, pChild, "The child shape does not exist.",)

    pCompound->removeChildShape(pChild);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    setChildTransform
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_setChildTransform
(JNIEnv *env, jobject object, jlong compoundShapeId, jlong childShapeId,
        jobject offsetVector, jobject rotationMatrix) {
    btCompoundShape * const pCompound
            = reinterpret_cast<btCompoundShape *> (compoundShapeId);
    NULL_CHK(env, pCompound, "The btCompoundShape does not exist.",)
    btAssert(pCompound->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);

    btCollisionShape * const pChild
            = reinterpret_cast<btCollisionShape *> (childShapeId);
    NULL_CHK(env, pChild, "The child shape does not exist.",);

    NULL_CHK(env, offsetVector, "The offset vector does not exist.",)
    NULL_CHK(env, rotationMatrix, "The rotation matrix does not exist.",)
    btTransform transform;
    jmeBulletUtil::convert(env, offsetVector, &transform.getOrigin());
    jmeBulletUtil::convert(env, rotationMatrix, &transform.getBasis());

    bool shouldRecalculateLocalAabb = true;
    int numChildren = pCompound->getNumChildShapes();
    for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
        const btCollisionShape *p = pCompound->getChildShape(childIndex);
        if (p == pChild) {
            pCompound->updateChildTransform(childIndex, transform,
                    shouldRecalculateLocalAabb);
        }
    }
}