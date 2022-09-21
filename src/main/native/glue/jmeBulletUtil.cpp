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
#include <math.h>
#include "jmeBulletUtil.h"
#include "jmeUserInfo.h"

/*
 * Author: Normen Hansen, Empire Phoenix, Lutherion
 */

void jmeBulletUtil::convert(JNIEnv *pEnv, jobject inVector3f, btVector3 *pvOut) {
    NULL_CHK(pEnv, inVector3f, "The input Vector3f does not exist.",)
    NULL_CHK(pEnv, pvOut, "The output btVector3 does not exist.",);

    float x = pEnv->GetFloatField(inVector3f, jmeClasses::Vector3f_x);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float y = pEnv->GetFloatField(inVector3f, jmeClasses::Vector3f_y);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float z = pEnv->GetFloatField(inVector3f, jmeClasses::Vector3f_z);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    pvOut->setX(x);
    pvOut->setY(y);
    pvOut->setZ(z);
}

// Copy a SimMath Quatd to a btQuaternion

void jmeBulletUtil::convertDp(JNIEnv *pEnv, jobject inQuatd, btQuaternion *pqOut) {
    NULL_CHK(pEnv, inQuatd, "The input Quatd does not exist.",)
    NULL_CHK(pEnv, pqOut, "The output btQuaternion does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Quatd_x, "The SimMath library is missing.",);

    double x = pEnv->GetDoubleField(inQuatd, jmeClasses::Quatd_x);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    double y = pEnv->GetDoubleField(inQuatd, jmeClasses::Quatd_y);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    double z = pEnv->GetDoubleField(inQuatd, jmeClasses::Quatd_z);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    double w = pEnv->GetDoubleField(inQuatd, jmeClasses::Quatd_w);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    pqOut->setX(x);
    pqOut->setY(y);
    pqOut->setZ(z);
    pqOut->setW(w);
}

// Copy a SimMath Vec3d to a btVector3

void jmeBulletUtil::convertDp(JNIEnv *pEnv, jobject inVec3d, btVector3 *pvOut) {
    NULL_CHK(pEnv, inVec3d, "The input Vec3d does not exist.",)
    NULL_CHK(pEnv, pvOut, "The output btVector3 does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Vec3d_x, "The SimMath library is missing.",);

    double x = pEnv->GetDoubleField(inVec3d, jmeClasses::Vec3d_x);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    double y = pEnv->GetDoubleField(inVec3d, jmeClasses::Vec3d_y);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    double z = pEnv->GetDoubleField(inVec3d, jmeClasses::Vec3d_z);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    pvOut->setX(x);
    pvOut->setY(y);
    pvOut->setZ(z);
}

void jmeBulletUtil::convert(JNIEnv *pEnv, jobject inQuaternion, btQuaternion *pqOut) {
    NULL_CHK(pEnv, inQuaternion, "The input Quaternion does not exist.",)
    NULL_CHK(pEnv, pqOut, "The output btQuaternion does not exist.",);

    float x = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_x);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float y = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_y);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float z = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_z);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float w = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_w);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    pqOut->setX(x);
    pqOut->setY(y);
    pqOut->setZ(z);
    pqOut->setW(w);
}

void jmeBulletUtil::convert(JNIEnv *pEnv, const btVector3 *pvIn, jobject outVector3f) {
    NULL_CHK(pEnv, pvIn, "The input btVector3 does not exist.",)
    NULL_CHK(pEnv, outVector3f, "The output Vector3f does not exist.",);

    float x = pvIn->getX();
    float y = pvIn->getY();
    float z = pvIn->getZ();

    pEnv->SetFloatField(outVector3f, jmeClasses::Vector3f_x, x);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outVector3f, jmeClasses::Vector3f_y, y);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outVector3f, jmeClasses::Vector3f_z, z);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

// Copy a btQuaternion to a SimMath Quatd

void jmeBulletUtil::convertDp(JNIEnv *pEnv, const btQuaternion *pqIn,
        jobject outQuatd) {
    NULL_CHK(pEnv, pqIn, "The input btQuaternion does not exist.",)
    NULL_CHK(pEnv, outQuatd, "The output Quatd does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Quatd_x, "The SimMath library is missing.",);

    pEnv->SetDoubleField(outQuatd, jmeClasses::Quatd_w, pqIn->w());
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetDoubleField(outQuatd, jmeClasses::Quatd_x, pqIn->x());
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetDoubleField(outQuatd, jmeClasses::Quatd_y, pqIn->y());
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetDoubleField(outQuatd, jmeClasses::Quatd_z, pqIn->z());
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

// Copy a btVector3 to a SimMath Vec3d

void jmeBulletUtil::convertDp(JNIEnv *pEnv, const btVector3 *pvIn, jobject outVec3d) {
    NULL_CHK(pEnv, pvIn, "The input btVector3 does not exist.",)
    NULL_CHK(pEnv, outVec3d, "The output Vec3d does not exist.",);
    NULL_CHK(pEnv, jmeClasses::Vec3d_x, "The SimMath library is missing.",);

    double x = pvIn->getX();
    double y = pvIn->getY();
    double z = pvIn->getZ();

    pEnv->SetDoubleField(outVec3d, jmeClasses::Vec3d_x, x);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetDoubleField(outVec3d, jmeClasses::Vec3d_y, y);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetDoubleField(outVec3d, jmeClasses::Vec3d_z, z);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

// Copy a btQuaternion to a JME Quaternion

void jmeBulletUtil::convert(JNIEnv *pEnv, const btQuaternion *pqIn,
        jobject outQuaternion) {
    NULL_CHK(pEnv, pqIn, "The input btQuaternion does not exist.",)
    NULL_CHK(pEnv, outQuaternion, "The output Quaternion does not exist.",);

    pEnv->SetFloatField(outQuaternion, jmeClasses::Quaternion_w, pqIn->w());
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outQuaternion, jmeClasses::Quaternion_x, pqIn->x());
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outQuaternion, jmeClasses::Quaternion_y, pqIn->y());
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outQuaternion, jmeClasses::Quaternion_z, pqIn->z());
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

// Copy a btTransform to a JME Transform

void jmeBulletUtil::convert(JNIEnv *pEnv,
        const btTransform *ptIn, jobject outTransform) {
    NULL_CHK(pEnv, ptIn, "The input btTransform does not exist.",)
    NULL_CHK(pEnv, outTransform, "The output Transform does not exist.",);

    jobject translation_out
            = pEnv->CallObjectMethod(outTransform, jmeClasses::Transform_translation);
    const btVector3& origin = ptIn->getOrigin();
    convert(pEnv, &origin, translation_out);

    jobject rotation_out
            = pEnv->CallObjectMethod(outTransform, jmeClasses::Transform_rotation);
    const btQuaternion rotation = ptIn->getRotation();
    convert(pEnv, &rotation, rotation_out);

    jobject scale_out = pEnv->CallObjectMethod(outTransform, jmeClasses::Transform_scale);
    pEnv->SetFloatField(scale_out, jmeClasses::Vector3f_x, 1);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(scale_out, jmeClasses::Vector3f_y, 1);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(scale_out, jmeClasses::Vector3f_z, 1);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

void jmeBulletUtil::convert(JNIEnv *pEnv, jobject inMatrix3f, btMatrix3x3 *pmOut) {
    NULL_CHK(pEnv, inMatrix3f, "The input Matrix3f does not exist.",)
    NULL_CHK(pEnv, pmOut, "The output btMatrix3x3 does not exist.",);

    float m00 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m00);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float m01 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m01);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float m02 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m02);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float m10 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m10);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float m11 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m11);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float m12 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m12);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float m20 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m20);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float m21 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m21);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float m22 = pEnv->GetFloatField(inMatrix3f, jmeClasses::Matrix3f_m22);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    pmOut->setValue(m00, m01, m02, m10, m11, m12, m20, m21, m22);
}

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
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m01, m01);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m02, m02);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m10, m10);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m11, m11);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m12, m12);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m20, m20);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m21, m21);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outMatrix3f, jmeClasses::Matrix3f_m22, m22);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

void jmeBulletUtil::convertQuat(JNIEnv *pEnv,
        jobject inQuaternion, btMatrix3x3 *pmOut) {
    NULL_CHK(pEnv, inQuaternion, "The input Quaternion does not exist.",)
    NULL_CHK(pEnv, pmOut, "The output btMatrix3x3 does not exist.",);

    float x = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_x);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float y = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_y);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float z = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_z);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    float w = pEnv->GetFloatField(inQuaternion, jmeClasses::Quaternion_w);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    // TODO use btMatrix3x3 constructor from btQuaternion

    float norm = w * w + x * x + y * y + z * z;
    float s = (norm == 1.0) ? 2.0 : (norm > 0.1) ? 2.0 / norm : 0.0;

    // compute xs/ys/zs first to save 6 multiplications, since xs/ys/zs
    // will be used 2-4 times each.
    float xs = x * s;
    float ys = y * s;
    float zs = z * s;
    float xx = x * xs;
    float xy = x * ys;
    float xz = x * zs;
    float xw = w * xs;
    float yy = y * ys;
    float yz = y * zs;
    float yw = w * ys;
    float zz = z * zs;
    float zw = w * zs;

    // using s=2/norm (instead of 1/norm) saves 9 multiplications by 2 here
    pmOut->setValue(1.0 - (yy + zz), (xy - zw), (xz + yw),
            (xy + zw), 1.0 - (xx + zz), (yz - xw),
            (xz - yw), (yz + xw), 1.0 - (xx + yy));
}

void jmeBulletUtil::convertQuat(JNIEnv *pEnv, const btMatrix3x3 *pmIn,
        jobject outQuaternion) {
    NULL_CHK(pEnv, pmIn, "The input btMatrix3x3 does not exist.",)
    NULL_CHK(pEnv, outQuaternion, "The output Quaternion does not exist.",);
    // TODO use btMatrix3x3 getRotation() method

    // the trace is the sum of the diagonal elements; see
    // http://mathworld.wolfram.com/MatrixTrace.html
    float t = pmIn->getRow(0).m_floats[0] + pmIn->getRow(1).m_floats[1] + pmIn->getRow(2).m_floats[2];
    float w, x, y, z;
    // we protect the division by s by ensuring that s>=1
    if (t >= 0) { // |w| >= .5
        float s = sqrt(t + 1); // |s|>=1 ...
        w = 0.5f * s;
        s = 0.5f / s; // so this division isn't bad
        x = (pmIn->getRow(2).m_floats[1] - pmIn->getRow(1).m_floats[2]) * s;
        y = (pmIn->getRow(0).m_floats[2] - pmIn->getRow(2).m_floats[0]) * s;
        z = (pmIn->getRow(1).m_floats[0] - pmIn->getRow(0).m_floats[1]) * s;
    } else if ((pmIn->getRow(0).m_floats[0] > pmIn->getRow(1).m_floats[1]) && (pmIn->getRow(0).m_floats[0] > pmIn->getRow(2).m_floats[2])) {
        float s = sqrt(1.0f + pmIn->getRow(0).m_floats[0] - pmIn->getRow(1).m_floats[1] - pmIn->getRow(2).m_floats[2]); // |s|>=1
        x = s * 0.5f; // |x| >= .5
        s = 0.5f / s;
        y = (pmIn->getRow(1).m_floats[0] + pmIn->getRow(0).m_floats[1]) * s;
        z = (pmIn->getRow(0).m_floats[2] + pmIn->getRow(2).m_floats[0]) * s;
        w = (pmIn->getRow(2).m_floats[1] - pmIn->getRow(1).m_floats[2]) * s;
    } else if (pmIn->getRow(1).m_floats[1] > pmIn->getRow(2).m_floats[2]) {
        float s = sqrt(1.0f + pmIn->getRow(1).m_floats[1] - pmIn->getRow(0).m_floats[0] - pmIn->getRow(2).m_floats[2]); // |s|>=1
        y = s * 0.5f; // |y| >= .5
        s = 0.5f / s;
        x = (pmIn->getRow(1).m_floats[0] + pmIn->getRow(0).m_floats[1]) * s;
        z = (pmIn->getRow(2).m_floats[1] + pmIn->getRow(1).m_floats[2]) * s;
        w = (pmIn->getRow(0).m_floats[2] - pmIn->getRow(2).m_floats[0]) * s;
    } else {
        float s = sqrt(1.0f + pmIn->getRow(2).m_floats[2] - pmIn->getRow(0).m_floats[0] - pmIn->getRow(1).m_floats[1]); // |s|>=1
        z = s * 0.5f; // |z| >= .5
        s = 0.5f / s;
        x = (pmIn->getRow(0).m_floats[2] + pmIn->getRow(2).m_floats[0]) * s;
        y = (pmIn->getRow(2).m_floats[1] + pmIn->getRow(1).m_floats[2]) * s;
        w = (pmIn->getRow(1).m_floats[0] - pmIn->getRow(0).m_floats[1]) * s;
    }

    pEnv->SetFloatField(outQuaternion, jmeClasses::Quaternion_x, x);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outQuaternion, jmeClasses::Quaternion_y, y);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outQuaternion, jmeClasses::Quaternion_z, z);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(outQuaternion, jmeClasses::Quaternion_w, w);
    //  TODO pEnv->CallObjectMethod(out, jmeClasses::Quaternion_set, x, y, z, w);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

/*
 * Add a ray-test result to a list.
 */
void jmeBulletUtil::addRayTestResult(JNIEnv *pEnv, jobject resultList,
        const btVector3 *pHitNormal, btScalar hitFraction,
        const btCollisionObject *pHitObject, int partIndex, int triangleIndex) {
    jobject result = pEnv->AllocObject(jmeClasses::PhysicsRay_Class);
    jobject normalvec = pEnv->AllocObject(jmeClasses::Vector3f);

    convert(pEnv, pHitNormal, normalvec);
    jmeUserPointer const pUser = (jmeUserPointer) pHitObject->getUserPointer();

    pEnv->SetObjectField(result, jmeClasses::PhysicsRay_normal, normalvec);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(result, jmeClasses::PhysicsRay_hitFraction, hitFraction);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetIntField(result, jmeClasses::PhysicsRay_partIndex, partIndex);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetIntField(result, jmeClasses::PhysicsRay_triangleIndex,
            triangleIndex);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetObjectField(result, jmeClasses::PhysicsRay_collisionObject,
            pUser->m_javaRef);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    pEnv->CallBooleanMethod(resultList, jmeClasses::List_addMethod, result);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

/*
 * Add a sweep-test result to a list.
 */
void jmeBulletUtil::addSweepTestResult(JNIEnv *pEnv, jobject resultList,
        const btVector3 *pHitNormal, btScalar hitFraction,
        const btCollisionObject *pHitObject, int partIndex, int triangleIndex) {
    jobject result = pEnv->AllocObject(jmeClasses::PhysicsSweep_Class);
    jobject normalvec = pEnv->AllocObject(jmeClasses::Vector3f);

    convert(pEnv, pHitNormal, normalvec);
    jmeUserPointer pUser = (jmeUserPointer) pHitObject->getUserPointer();

    pEnv->SetObjectField(result, jmeClasses::PhysicsSweep_normal, normalvec);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetFloatField(result, jmeClasses::PhysicsSweep_hitFraction,
            hitFraction);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetIntField(result, jmeClasses::PhysicsSweep_partIndex, partIndex);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetIntField(result, jmeClasses::PhysicsSweep_triangleIndex,
            triangleIndex);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    pEnv->SetObjectField(result, jmeClasses::PhysicsSweep_collisionObject,
            pUser->m_javaRef);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    pEnv->CallBooleanMethod(resultList, jmeClasses::List_addMethod, result);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}

void jmeBulletUtil::convert(JNIEnv *pEnv, jobject inTransform, btTransform *ptOut,
        btVector3 *pvOutScale) {
    NULL_CHK(pEnv, inTransform, "The input Transform does not exist.",)
    NULL_CHK(pEnv, ptOut, "The output btTransform does not exist.",);
    NULL_CHK(pEnv, pvOutScale, "The output btVector3 does not exist.",);

    jobject translation_vec
            = pEnv->CallObjectMethod(inTransform, jmeClasses::Transform_translation);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    jobject rot_quat = pEnv->CallObjectMethod(inTransform, jmeClasses::Transform_rotation);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    jobject scale_vec = pEnv->CallObjectMethod(inTransform, jmeClasses::Transform_scale);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    btVector3 native_translation_vec;
    convert(pEnv, translation_vec, &native_translation_vec);
    ptOut->setOrigin(native_translation_vec);

    btQuaternion native_rot_quat;
    convert(pEnv, rot_quat, &native_rot_quat);
    ptOut->setRotation(native_rot_quat);

    convert(pEnv, scale_vec, pvOutScale);
}
