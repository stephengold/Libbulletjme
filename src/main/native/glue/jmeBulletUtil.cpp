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
 * Author: Normen Hansen, Empire Phoenix, Lutherion
 */
#include <math.h>
#include "jmeBulletUtil.h"
#include "jmeUserInfo.h"

// Copy JMonkeyEngine Vector3f data to a Bullet btVector3 object.

void jmeBulletUtil::convert(
        JNIEnv *pEnv, jobject inVector3f, btVector3 *pvOut) {
    NULL_CHK(pEnv, inVector3f, "The input Vector3f does not exist.",)
    NULL_CHK(pEnv, pvOut, "The output btVector3 does not exist.",);

    float x = pEnv->GetFloatField(inVector3f, jmeClasses::Vector3f_x);
    EXCEPTION_CHK(pEnv,);
    float y = pEnv->GetFloatField(inVector3f, jmeClasses::Vector3f_y);
    EXCEPTION_CHK(pEnv,);
    float z = pEnv->GetFloatField(inVector3f, jmeClasses::Vector3f_z);
    EXCEPTION_CHK(pEnv,);

    pvOut->setValue(x, y, z);
}

// Copy SimMath Quatd data to a Bullet btQuaternion object.

void jmeBulletUtil::convertDp(
        JNIEnv *pEnv, jobject inQuatd, btQuaternion *pqOut) {
    NULL_CHK(pEnv, inQuatd, "The input Quatd does not exist.",)
    NULL_CHK(pEnv, pqOut, "The output btQuaternion does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Quatd_x, "The SimMath library is missing.",);

    double x = pEnv->GetDoubleField(inQuatd, jmeClasses::Quatd_x);
    EXCEPTION_CHK(pEnv,);
    double y = pEnv->GetDoubleField(inQuatd, jmeClasses::Quatd_y);
    EXCEPTION_CHK(pEnv,);
    double z = pEnv->GetDoubleField(inQuatd, jmeClasses::Quatd_z);
    EXCEPTION_CHK(pEnv,);
    double w = pEnv->GetDoubleField(inQuatd, jmeClasses::Quatd_w);
    EXCEPTION_CHK(pEnv,);

    pqOut->setValue(x, y, z, w);
}

// Copy SimMath Vec3d data to a Bullet btVector3 object.

void jmeBulletUtil::convertDp(JNIEnv *pEnv, jobject inVec3d, btVector3 *pvOut) {
    NULL_CHK(pEnv, inVec3d, "The input Vec3d does not exist.",)
    NULL_CHK(pEnv, pvOut, "The output btVector3 does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Vec3d_x, "The SimMath library is missing.",);

    double x = pEnv->GetDoubleField(inVec3d, jmeClasses::Vec3d_x);
    EXCEPTION_CHK(pEnv,);
    double y = pEnv->GetDoubleField(inVec3d, jmeClasses::Vec3d_y);
    EXCEPTION_CHK(pEnv,);
    double z = pEnv->GetDoubleField(inVec3d, jmeClasses::Vec3d_z);
    EXCEPTION_CHK(pEnv,);

    pvOut->setValue(x, y, z);
}

// Copy JMonkeyEngine Quaternion data to a Bullet btQuaternion object.

void jmeBulletUtil::convert(
        JNIEnv *pEnv, jobject inQuaternion, btQuaternion *pqOut) {
    NULL_CHK(pEnv, inQuaternion, "The input Quaternion does not exist.",)
    NULL_CHK(pEnv, pqOut, "The output btQuaternion does not exist.",);

    float x = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_x);
    EXCEPTION_CHK(pEnv,);
    float y = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_y);
    EXCEPTION_CHK(pEnv,);
    float z = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_z);
    EXCEPTION_CHK(pEnv,);
    float w = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_w);
    EXCEPTION_CHK(pEnv,);

    pqOut->setValue(x, y, z, w);
}

// Copy Bullet btVector3 data to a JMonkeyEngine Vector3f object.

void jmeBulletUtil::convert(
        JNIEnv *pEnv, const btVector3 *pvIn, jobject outVector3f) {
    NULL_CHK(pEnv, pvIn, "The input btVector3 does not exist.",)
    NULL_CHK(pEnv, outVector3f, "The output Vector3f does not exist.",);

    float x = pvIn->getX();
    float y = pvIn->getY();
    float z = pvIn->getZ();

    pEnv->CallObjectMethod(outVector3f, jmeClasses::Vector3f_set, x, y, z);
    // no check for exceptions!
}

// Copy Bullet btQuaternion data to a SimMath Quatd object.

void jmeBulletUtil::convertDp(
        JNIEnv *pEnv, const btQuaternion *pqIn, jobject outQuatd) {
    NULL_CHK(pEnv, pqIn, "The input btQuaternion does not exist.",)
    NULL_CHK(pEnv, outQuatd, "The output Quatd does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Quatd_set, "The SimMath library is missing.",);

    double x = pqIn->getX();
    double y = pqIn->getY();
    double z = pqIn->getZ();
    double w = pqIn->getW();

    pEnv->CallObjectMethod(outQuatd, jmeClasses::Quatd_set, x, y, z, w);
    // no check for exceptions!
}

// Copy Bullet btVector3 data to a SimMath Vec3d object.

void jmeBulletUtil::convertDp(
        JNIEnv *pEnv, const btVector3 *pvIn, jobject outVec3d) {
    NULL_CHK(pEnv, pvIn, "The input btVector3 does not exist.",)
    NULL_CHK(pEnv, outVec3d, "The output Vec3d does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Vec3d_set, "The SimMath library is missing.",);

    double x = pvIn->getX();
    double y = pvIn->getY();
    double z = pvIn->getZ();

    pEnv->CallObjectMethod(outVec3d, jmeClasses::Vec3d_set, x, y, z);
    // no check for exceptions!
}

// Copy Bullet btQuaternion data to a JMonkeyEngine Quaternion object.

void jmeBulletUtil::convert(
        JNIEnv *pEnv, const btQuaternion *pqIn, jobject outQuaternion) {
    NULL_CHK(pEnv, pqIn, "The input btQuaternion does not exist.",)
    NULL_CHK(pEnv, outQuaternion, "The output Quaternion does not exist.",);

    float x = pqIn->x();
    float y = pqIn->y();
    float z = pqIn->z();
    float w = pqIn->w();

    pEnv->CallObjectMethod(
            outQuaternion, jmeClasses::Quaternion_set, x, y, z, w);
    // no check for exceptions!
}

// Copy Bullet btTransform data to a JMonkeyEngine Transform object.

void jmeBulletUtil::convert(JNIEnv *pEnv,
        const btTransform *ptIn, jobject outTransform) {
    NULL_CHK(pEnv, ptIn, "The input btTransform does not exist.",)
    NULL_CHK(pEnv, outTransform, "The output Transform does not exist.",);

    jobject translation_out = pEnv->CallObjectMethod(
            outTransform, jmeClasses::Transform_translation);
    EXCEPTION_CHK(pEnv,);
    const btVector3& origin = ptIn->getOrigin();
    convert(pEnv, &origin, translation_out);

    jobject rotation_out = pEnv->CallObjectMethod(
            outTransform, jmeClasses::Transform_rotation);
    EXCEPTION_CHK(pEnv,);
    const btQuaternion rotation = ptIn->getRotation();
    convert(pEnv, &rotation, rotation_out);

    jobject scale_out
            = pEnv->CallObjectMethod(outTransform, jmeClasses::Transform_scale);
    pEnv->SetFloatField(scale_out, jmeClasses::Vector3f_x, 1);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(scale_out, jmeClasses::Vector3f_y, 1);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(scale_out, jmeClasses::Vector3f_z, 1);
}

// Copy JMonkeyEngine Matrix3f data to a Bullet btMatrix3x3 object.

void jmeBulletUtil::convert(
            JNIEnv *pEnv, jobject inMatrix3f, btMatrix3x3 *pmOut) {
    NULL_CHK(pEnv, inMatrix3f, "The input Matrix3f does not exist.",)
    NULL_CHK(pEnv, pmOut, "The output btMatrix3x3 does not exist.",);

    float m00 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m00);
    EXCEPTION_CHK(pEnv,);
    float m01 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m01);
    EXCEPTION_CHK(pEnv,);
    float m02 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m02);
    EXCEPTION_CHK(pEnv,);
    float m10 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m10);
    EXCEPTION_CHK(pEnv,);
    float m11 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m11);
    EXCEPTION_CHK(pEnv,);
    float m12 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m12);
    EXCEPTION_CHK(pEnv,);
    float m20 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m20);
    EXCEPTION_CHK(pEnv,);
    float m21 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m21);
    EXCEPTION_CHK(pEnv,);
    float m22 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m22);
    EXCEPTION_CHK(pEnv,);

    pmOut->setValue(m00, m01, m02, m10, m11, m12, m20, m21, m22);
}

// Copy Bullet btMatrix3x3 data to a JMonkeyEngine Matrix3f object.

void jmeBulletUtil::convert(JNIEnv *pEnv,
        const btMatrix3x3 *pmIn, jobject outMatrix3f) {
    NULL_CHK(pEnv, pmIn, "The input btMatrix3x3 does not exist.",)
    NULL_CHK(pEnv, outMatrix3f, "The output Matrix3f does not exist.",);

    float m00 = pmIn->getRow(0).m_floats[0];
    float m01 = pmIn->getRow(0).m_floats[1];
    float m02 = pmIn->getRow(0).m_floats[2];
    float m10 = pmIn->getRow(1).m_floats[0];
    float m11 = pmIn->getRow(1).m_floats[1];
    float m12 = pmIn->getRow(1).m_floats[2];
    float m20 = pmIn->getRow(2).m_floats[0];
    float m21 = pmIn->getRow(2).m_floats[1];
    float m22 = pmIn->getRow(2).m_floats[2];

    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m00, m00);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m01, m01);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m02, m02);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m10, m10);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m11, m11);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m12, m12);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m20, m20);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m21, m21);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m22, m22);
}

// Copy Bullet btMatrix3x3 data to a SimMath Matrix3d object.

void jmeBulletUtil::convertDp(
        JNIEnv *pEnv, const btMatrix3x3 *pmIn, jobject outMatrix3d) {
    NULL_CHK(pEnv, pmIn, "The input btMatrix3x3 does not exist.",)
    NULL_CHK(pEnv, outMatrix3d, "The output Matrix3d does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Matrix3d_m00,
            "The SimMath library is missing.",);

    double m00 = pmIn->getRow(0).m_floats[0];
    double m01 = pmIn->getRow(0).m_floats[1];
    double m02 = pmIn->getRow(0).m_floats[2];
    double m10 = pmIn->getRow(1).m_floats[0];
    double m11 = pmIn->getRow(1).m_floats[1];
    double m12 = pmIn->getRow(1).m_floats[2];
    double m20 = pmIn->getRow(2).m_floats[0];
    double m21 = pmIn->getRow(2).m_floats[1];
    double m22 = pmIn->getRow(2).m_floats[2];

    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m00, m00);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m01, m01);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m02, m02);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m10, m10);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m11, m11);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m12, m12);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m20, m20);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m21, m21);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetDoubleField(outMatrix3d, jmeClasses::Matrix3d_m22, m22);
}

// Convert a JMonkeyEngine Quaternion to a Bullet rotation matrix.

void jmeBulletUtil::convertQuat(
        JNIEnv *pEnv, jobject inQuaternion, btMatrix3x3 *pmOut) {
    NULL_CHK(pEnv, inQuaternion, "The input Quaternion does not exist.",)
    NULL_CHK(pEnv, pmOut, "The output btMatrix3x3 does not exist.",);

    btQuaternion q;
    convert(pEnv, inQuaternion, &q);
    pmOut->setRotation(q);
}

// Copy SimMath Matrix3d data to a Bullet btMatrix3x3 object.

void jmeBulletUtil::convertDp(
            JNIEnv *pEnv, jobject inMatrix3d, btMatrix3x3 *pmOut) {
    NULL_CHK(pEnv, inMatrix3d, "The input Matrix3d does not exist.",)
    NULL_CHK(pEnv, pmOut, "The output btMatrix3x3 does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Matrix3d_m00,
            "The SimMath library is missing.",);

    double m00 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m00);
    EXCEPTION_CHK(pEnv,);
    double m01 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m01);
    EXCEPTION_CHK(pEnv,);
    double m02 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m02);
    EXCEPTION_CHK(pEnv,);
    double m10 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m10);
    EXCEPTION_CHK(pEnv,);
    double m11 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m11);
    EXCEPTION_CHK(pEnv,);
    double m12 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m12);
    EXCEPTION_CHK(pEnv,);
    double m20 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m20);
    EXCEPTION_CHK(pEnv,);
    double m21 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m21);
    EXCEPTION_CHK(pEnv,);
    double m22 = pEnv->GetDoubleField(inMatrix3d, jmeClasses::Matrix3d_m22);
    EXCEPTION_CHK(pEnv,);

    pmOut->setValue(m00, m01, m02, m10, m11, m12, m20, m21, m22);
}

// Convert a SimMath Quatd to a Bullet rotation matrix.

void jmeBulletUtil::convertQuatDp(
        JNIEnv *pEnv, jobject inQuatd, btMatrix3x3 *pmOut) {
    NULL_CHK(pEnv, inQuatd, "The input Quatd does not exist.",)
    NULL_CHK(pEnv, pmOut, "The output btMatrix3x3 does not exist.",);

    btQuaternion q;
    convertDp(pEnv, inQuatd, &q);
    pmOut->setRotation(q);
}

// Convert a Bullet rotation matrix to a JMonkeyEngine Quaternion.

void jmeBulletUtil::convertQuat(JNIEnv *pEnv, const btMatrix3x3 *pmIn,
        jobject outQuaternion) {
    NULL_CHK(pEnv, pmIn, "The input btMatrix3x3 does not exist.",)
    NULL_CHK(pEnv, outQuaternion, "The output Quaternion does not exist.",);

    btQuaternion q;
    pmIn->getRotation(q);
    convert(pEnv, &q, outQuaternion);
}

// Convert a Bullet rotation matrix to a SimMath Quatd.

void jmeBulletUtil::convertQuatDp(
        JNIEnv *pEnv, const btMatrix3x3 *pmIn, jobject outQuatd) {
    NULL_CHK(pEnv, pmIn, "The input btMatrix3x3 does not exist.",)
    NULL_CHK(pEnv, outQuatd, "The output Quatd does not exist.",);

    btQuaternion q;
    pmIn->getRotation(q);
    convertDp(pEnv, &q, outQuatd);
}

// Add a ray-test result to a list.

void jmeBulletUtil::addRayTestResult(JNIEnv *pEnv, jobject resultList,
        const btVector3 *pHitNormal, btScalar hitFraction,
        const btCollisionObject *pHitObject, int partIndex, int triangleIndex) {
    jobject result = pEnv->AllocObject(jmeClasses::PhysicsRay_Class);
    EXCEPTION_CHK(pEnv,);
    jobject normalvec = pEnv->AllocObject(jmeClasses::Vector3f);
    EXCEPTION_CHK(pEnv,);

    convert(pEnv, pHitNormal, normalvec);
    jmeUserPointer const pUser = (jmeUserPointer) pHitObject->getUserPointer();

    pEnv->SetObjectField(result, jmeClasses::PhysicsRay_normal, normalvec);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(
            result, jmeClasses::PhysicsRay_hitFraction, hitFraction);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetIntField(result, jmeClasses::PhysicsRay_partIndex, partIndex);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetIntField(
            result, jmeClasses::PhysicsRay_triangleIndex, triangleIndex);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetObjectField(result, jmeClasses::PhysicsRay_collisionObject,
            pUser->m_javaRef);
    EXCEPTION_CHK(pEnv,);

    pEnv->CallBooleanMethod(resultList, jmeClasses::List_addMethod, result);
    // no check for exceptions!
}

// Add a sweep-test result to a list.

void jmeBulletUtil::addSweepTestResult(JNIEnv *pEnv, jobject resultList,
        const btVector3 *pHitNormal, btScalar hitFraction,
        const btCollisionObject *pHitObject, int partIndex, int triangleIndex) {
    jobject result = pEnv->AllocObject(jmeClasses::PhysicsSweep_Class);
    EXCEPTION_CHK(pEnv,);
    jobject normalvec = pEnv->AllocObject(jmeClasses::Vector3f);
    EXCEPTION_CHK(pEnv,);

    convert(pEnv, pHitNormal, normalvec);
    jmeUserPointer pUser = (jmeUserPointer) pHitObject->getUserPointer();

    pEnv->SetObjectField(result, jmeClasses::PhysicsSweep_normal, normalvec);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetFloatField(
            result, jmeClasses::PhysicsSweep_hitFraction, hitFraction);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetIntField(result, jmeClasses::PhysicsSweep_partIndex, partIndex);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetIntField(result, jmeClasses::PhysicsSweep_triangleIndex,
            triangleIndex);
    EXCEPTION_CHK(pEnv,);
    pEnv->SetObjectField(result, jmeClasses::PhysicsSweep_collisionObject,
            pUser->m_javaRef);
    EXCEPTION_CHK(pEnv,);

    pEnv->CallBooleanMethod(resultList, jmeClasses::List_addMethod, result);
    // no check for exceptions!
}

// Convert a JMonkeyEngine Transform to a Bullet btTransform and a scale vector.

void jmeBulletUtil::convert(JNIEnv *pEnv, jobject inTransform,
        btTransform *ptOut, btVector3 *pvOutScale) {
    NULL_CHK(pEnv, inTransform, "The input Transform does not exist.",)
    NULL_CHK(pEnv, ptOut, "The output btTransform does not exist.",);
    NULL_CHK(pEnv, pvOutScale, "The output btVector3 does not exist.",);

    jobject translation_vec = pEnv->CallObjectMethod(
            inTransform, jmeClasses::Transform_translation);
    EXCEPTION_CHK(pEnv,);

    jobject rot_quat = pEnv->CallObjectMethod(
            inTransform, jmeClasses::Transform_rotation);
    EXCEPTION_CHK(pEnv,);

    jobject scale_vec
            = pEnv->CallObjectMethod(inTransform, jmeClasses::Transform_scale);
    EXCEPTION_CHK(pEnv,);

    btVector3 native_translation_vec;
    convert(pEnv, translation_vec, &native_translation_vec);
    ptOut->setOrigin(native_translation_vec);

    btQuaternion native_rot_quat;
    convert(pEnv, rot_quat, &native_rot_quat);
    ptOut->setRotation(native_rot_quat);

    convert(pEnv, scale_vec, pvOutScale);
}
