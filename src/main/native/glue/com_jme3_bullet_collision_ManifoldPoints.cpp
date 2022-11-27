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

#include "jmeBulletUtil.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "com_jme3_bullet_collision_ManifoldPoints.h"

extern bool gContactCalcArea3Points;

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    createTestPoint
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_createTestPoint
(JNIEnv *pEnv, jclass) {
    jmeClasses::initJavaClasses(pEnv);

    btManifoldPoint * const pPoint = new btManifoldPoint();
    return reinterpret_cast<jlong> (pPoint);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getAppliedImpulse
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getAppliedImpulse
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_appliedImpulse;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getAppliedImpulseLateral1
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getAppliedImpulseLateral1
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_appliedImpulseLateral1;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getAppliedImpulseLateral2
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getAppliedImpulseLateral2
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_appliedImpulseLateral2;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getCombinedFriction
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getCombinedFriction
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_combinedFriction;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getCombinedRestitution
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getCombinedRestitution
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_combinedRestitution;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getCombinedRollingFriction
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getCombinedRollingFriction
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_combinedRollingFriction;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getCombinedSpinningFriction
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getCombinedSpinningFriction
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_combinedSpinningFriction;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getContactMotion1
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getContactMotion1
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_contactMotion1;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getContactMotion2
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getContactMotion2
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_contactMotion2;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getDistance1
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getDistance1
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jfloat result = pPoint->m_distance1;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getFlags
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getFlags
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0);

    jint result = pPoint->m_contactPointFlags;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getIndex0
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getIndex0
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jint result = pPoint->m_index0;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getIndex1
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getIndex1
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jint result = pPoint->m_index1;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getLateralFrictionDir1
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getLateralFrictionDir1
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, &pPoint->m_lateralFrictionDir1, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getLateralFrictionDir2
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getLateralFrictionDir2
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, &pPoint->m_lateralFrictionDir2, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getLifeTime
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getLifeTime
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jint result = pPoint->m_lifeTime;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getLocalPointA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getLocalPointA
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, &pPoint->m_localPointA, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getLocalPointB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getLocalPointB
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, &pPoint->m_localPointB, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getNormalWorldOnB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getNormalWorldOnB
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, &pPoint->m_normalWorldOnB, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getPartId0
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getPartId0
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jint result = pPoint->m_partId0;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getPartId1
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getPartId1
(JNIEnv *pEnv, jclass, jlong pointId) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.", 0)

    jint result = pPoint->m_partId1;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getPositionWorldOnA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getPositionWorldOnA
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, &pPoint->m_positionWorldOnA, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getPositionWorldOnADp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getPositionWorldOnADp
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convertDp(pEnv, &pPoint->m_positionWorldOnA, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getPositionWorldOnB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getPositionWorldOnB
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, &pPoint->m_positionWorldOnB, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    getPositionWorldOnBDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_getPositionWorldOnBDp
(JNIEnv *pEnv, jclass, jlong pointId, jobject storeVector) {
    const btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convertDp(pEnv, &pPoint->m_positionWorldOnB, storeVector);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    isContactCalcArea3Points
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_isContactCalcArea3Points
(JNIEnv *pEnv, jclass) {
    jmeClasses::initJavaClasses(pEnv);
    jboolean result = gContactCalcArea3Points;
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setAppliedImpulse
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setAppliedImpulse
(JNIEnv *pEnv, jclass, jlong pointId, jfloat impulse) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_appliedImpulse = btScalar(impulse);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setAppliedImpulseLateral1
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setAppliedImpulseLateral1
(JNIEnv *pEnv, jclass, jlong pointId, jfloat impulse) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_appliedImpulseLateral1 = btScalar(impulse);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setAppliedImpulseLateral2
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setAppliedImpulseLateral2
(JNIEnv *pEnv, jclass, jlong pointId, jfloat impulse) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_appliedImpulseLateral2 = btScalar(impulse);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setCombinedFriction
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setCombinedFriction
(JNIEnv *pEnv, jclass, jlong pointId, jfloat friction) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_combinedFriction = btScalar(friction);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setCombinedRestitution
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setCombinedRestitution
(JNIEnv *pEnv, jclass, jlong pointId, jfloat restitution) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_combinedRestitution = btScalar(restitution);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setCombinedRollingFriction
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setCombinedRollingFriction
(JNIEnv *pEnv, jclass, jlong pointId, jfloat friction) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_combinedRollingFriction = btScalar(friction);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setCombinedSpinningFriction
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setCombinedSpinningFriction
(JNIEnv *pEnv, jclass, jlong pointId, jfloat friction) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_combinedSpinningFriction = btScalar(friction);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setContactCalcArea3Points
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setContactCalcArea3Points
(JNIEnv *pEnv, jclass, jboolean setting) {
    jmeClasses::initJavaClasses(pEnv);
    gContactCalcArea3Points = bool(setting);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setContactMotion1
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setContactMotion1
(JNIEnv *pEnv, jclass, jlong pointId, jfloat motion) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_contactMotion1 = btScalar(motion);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setContactMotion2
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setContactMotion2
(JNIEnv *pEnv, jclass, jlong pointId, jfloat motion) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_contactMotion2 = btScalar(motion);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setDistance1
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setDistance1
(JNIEnv *pEnv, jclass, jlong pointId, jfloat distance) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_distance1 = btScalar(distance);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setFlags
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setFlags
(JNIEnv *pEnv, jclass, jlong pointId, jint flags) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    pPoint->m_contactPointFlags = int(flags);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setLateralFrictionDir1
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setLateralFrictionDir1
(JNIEnv *pEnv, jclass, jlong pointId, jobject directionVector) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, directionVector,
            &pPoint->m_lateralFrictionDir1);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setLateralFrictionDir2
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setLateralFrictionDir2
(JNIEnv *pEnv, jclass, jlong pointId, jobject directionVector) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, directionVector,
            &pPoint->m_lateralFrictionDir2);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setLocalPointA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setLocalPointA
(JNIEnv *pEnv, jclass, jlong pointId, jobject locationVector) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, locationVector, &pPoint->m_localPointA);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setLocalPointB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setLocalPointB
(JNIEnv *pEnv, jclass, jlong pointId, jobject locationVector) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, locationVector, &pPoint->m_localPointB);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setNormalWorldOnB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setNormalWorldOnB
(JNIEnv *pEnv, jclass, jlong pointId, jobject normalVector) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, normalVector, &pPoint->m_normalWorldOnB);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setPositionWorldOnA
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setPositionWorldOnA
(JNIEnv *pEnv, jclass, jlong pointId, jobject locationVector) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, locationVector, &pPoint->m_positionWorldOnA);
}

/*
 * Class:     com_jme3_bullet_collision_ManifoldPoints
 * Method:    setPositionWorldOnB
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_ManifoldPoints_setPositionWorldOnB
(JNIEnv *pEnv, jclass, jlong pointId, jobject locationVector) {
    btManifoldPoint * const
            pPoint = reinterpret_cast<btManifoldPoint *> (pointId);
    NULL_CHK(pEnv, pPoint, "The btManifoldPoint does not exist.",)

    jmeBulletUtil::convert(pEnv, locationVector, &pPoint->m_positionWorldOnB);
}
