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

/*
 * Author: Normen Hansen, Empire Phoenix, Lutherion
 */
void jmeBulletUtil::convert(JNIEnv *env, jobject in, btVector3 *pvOut) {
    NULL_CHECK(in, "The input Vector3f does not exist.",)
    NULL_CHECK(pvOut, "The output btVector3 does not exist.",);

    float x = env->GetFloatField(in, jmeClasses::Vector3f_x);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float y = env->GetFloatField(in, jmeClasses::Vector3f_y);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float z = env->GetFloatField(in, jmeClasses::Vector3f_z);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }

    pvOut->setX(x);
    pvOut->setY(y);
    pvOut->setZ(z);
}

void jmeBulletUtil::convert(JNIEnv *env, jobject in, btQuaternion *pqOut) {
    NULL_CHECK(in, "The input Quaternion does not exist.",)
    NULL_CHECK(pqOut, "The output btQuaternion does not exist.",);

    float x = env->GetFloatField(in, jmeClasses::Quaternion_x);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float y = env->GetFloatField(in, jmeClasses::Quaternion_y);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float z = env->GetFloatField(in, jmeClasses::Quaternion_z);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float w = env->GetFloatField(in, jmeClasses::Quaternion_w);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }

    pqOut->setX(x);
    pqOut->setY(y);
    pqOut->setZ(z);
    pqOut->setW(w);
}

void jmeBulletUtil::convert(JNIEnv *env, const btVector3 *pvIn, jobject out) {
    NULL_CHECK(pvIn, "The input btVector3 does not exist.",)
    NULL_CHECK(out, "The output Vector3f does not exist.",);

    float x = pvIn->getX();
    float y = pvIn->getY();
    float z = pvIn->getZ();

    env->SetFloatField(out, jmeClasses::Vector3f_x, x);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Vector3f_y, y);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Vector3f_z, z);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
}

// Copy a btQuaternion to a JME Quaternion

void jmeBulletUtil::convert(JNIEnv *env, const btQuaternion *pqIn,
        jobject out) {
    NULL_CHECK(pqIn, "The input btQuaternion does not exist.",)
    NULL_CHECK(out, "The output Quaternion does not exist.",);

    env->SetFloatField(out, jmeClasses::Quaternion_w, pqIn->w());
    env->SetFloatField(out, jmeClasses::Quaternion_x, pqIn->x());
    env->SetFloatField(out, jmeClasses::Quaternion_y, pqIn->y());
    env->SetFloatField(out, jmeClasses::Quaternion_z, pqIn->z());
}

// Copy a btTransform to a JME Transform

void jmeBulletUtil::convert(JNIEnv *env, const btTransform *ptIn,
        jobject out) {
    NULL_CHECK(ptIn, "The input btTransform does not exist.",)
    NULL_CHECK(out, "The output Transform does not exist.",);

    jobject translation_out
            = env->CallObjectMethod(out, jmeClasses::Transform_translation);
    const btVector3& origin = ptIn->getOrigin();
    convert(env, &origin, translation_out);

    jobject rotation_out
            = env->CallObjectMethod(out, jmeClasses::Transform_rotation);
    const btQuaternion rotation = ptIn->getRotation();
    convert(env, &rotation, rotation_out);

    jobject scale_out = env->CallObjectMethod(out, jmeClasses::Transform_scale);
    env->SetFloatField(scale_out, jmeClasses::Vector3f_x, 1);
    env->SetFloatField(scale_out, jmeClasses::Vector3f_y, 1);
    env->SetFloatField(scale_out, jmeClasses::Vector3f_z, 1);
}

void jmeBulletUtil::convert(JNIEnv *env, jobject in, btMatrix3x3 *pmOut) {
    NULL_CHECK(in, "The input Matrix3f does not exist.",)
    NULL_CHECK(pmOut, "The output btMatrix3x3 does not exist.",);

    float m00 = env->GetFloatField(in, jmeClasses::Matrix3f_m00);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float m01 = env->GetFloatField(in, jmeClasses::Matrix3f_m01);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float m02 = env->GetFloatField(in, jmeClasses::Matrix3f_m02);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float m10 = env->GetFloatField(in, jmeClasses::Matrix3f_m10);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float m11 = env->GetFloatField(in, jmeClasses::Matrix3f_m11);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float m12 = env->GetFloatField(in, jmeClasses::Matrix3f_m12);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float m20 = env->GetFloatField(in, jmeClasses::Matrix3f_m20);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float m21 = env->GetFloatField(in, jmeClasses::Matrix3f_m21);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float m22 = env->GetFloatField(in, jmeClasses::Matrix3f_m22);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }

    pmOut->setValue(m00, m01, m02, m10, m11, m12, m20, m21, m22);
}

void jmeBulletUtil::convert(JNIEnv *env, const btMatrix3x3 *pmIn, jobject out) {
    NULL_CHECK(pmIn, "The input btMatrix3x3 does not exist.",)
    NULL_CHECK(out, "The output Matrix3f does not exist.",);

    float m00 = pmIn->getRow(0).m_floats[0];
    float m01 = pmIn->getRow(0).m_floats[1];
    float m02 = pmIn->getRow(0).m_floats[2];
    float m10 = pmIn->getRow(1).m_floats[0];
    float m11 = pmIn->getRow(1).m_floats[1];
    float m12 = pmIn->getRow(1).m_floats[2];
    float m20 = pmIn->getRow(2).m_floats[0];
    float m21 = pmIn->getRow(2).m_floats[1];
    float m22 = pmIn->getRow(2).m_floats[2];

    env->SetFloatField(out, jmeClasses::Matrix3f_m00, m00);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Matrix3f_m01, m01);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Matrix3f_m02, m02);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Matrix3f_m10, m10);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Matrix3f_m11, m11);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Matrix3f_m12, m12);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Matrix3f_m20, m20);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Matrix3f_m21, m21);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Matrix3f_m22, m22);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
}

void jmeBulletUtil::convertQuat(JNIEnv *env, jobject in, btMatrix3x3 *pmOut) {
    NULL_CHECK(in, "The input Quaternion does not exist.",)
    NULL_CHECK(pmOut, "The output btMatrix3x3 does not exist.",);

    float x = env->GetFloatField(in, jmeClasses::Quaternion_x);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float y = env->GetFloatField(in, jmeClasses::Quaternion_y);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float z = env->GetFloatField(in, jmeClasses::Quaternion_z);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    float w = env->GetFloatField(in, jmeClasses::Quaternion_w);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }

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

void jmeBulletUtil::convertQuat(JNIEnv *env, const btMatrix3x3 *pmIn,
        jobject out) {
    NULL_CHECK(pmIn, "The input btMatrix3x3 does not exist.",)
    NULL_CHECK(out, "The output Quaternion does not exist.",);

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

    env->SetFloatField(out, jmeClasses::Quaternion_x, x);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Quaternion_y, y);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Quaternion_z, z);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
    env->SetFloatField(out, jmeClasses::Quaternion_w, w);
    //  TODO env->CallObjectMethod(out, jmeClasses::Quaternion_set, x, y, z, w);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
}

/*
 * Add a ray-test result to a list.
 */
void jmeBulletUtil::addRayTestResult(JNIEnv *env, jobject resultList,
        const btVector3 *pHitNormal, btScalar hitFraction,
        const btCollisionObject *pHitObject, int partIndex, int triangleIndex) {
    jobject result = env->AllocObject(jmeClasses::PhysicsRay_Class);
    jobject normalvec = env->AllocObject(jmeClasses::Vector3f);

    convert(env, pHitNormal, normalvec);
    jmeUserPointer *pUser = (jmeUserPointer *) pHitObject->getUserPointer();

    env->SetObjectField(result, jmeClasses::PhysicsRay_normal, normalvec);
    env->SetFloatField(result, jmeClasses::PhysicsRay_hitFraction, hitFraction);
    env->SetIntField(result, jmeClasses::PhysicsRay_partIndex, partIndex);
    env->SetIntField(result, jmeClasses::PhysicsRay_triangleIndex,
            triangleIndex);
    env->SetObjectField(result, jmeClasses::PhysicsRay_collisionObject,
            pUser->javaCollisionObject);

    env->CallBooleanMethod(resultList, jmeClasses::PhysicsRay_addmethod,
            result);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
}

/*
 * Add a sweep-test result to a list.
 */
void jmeBulletUtil::addSweepTestResult(JNIEnv *env, jobject resultList,
        const btVector3 *pHitNormal, btScalar hitFraction,
        const btCollisionObject *pHitObject, int partIndex, int triangleIndex) {
    jobject result = env->AllocObject(jmeClasses::PhysicsSweep_Class);
    jobject normalvec = env->AllocObject(jmeClasses::Vector3f);

    convert(env, pHitNormal, normalvec);
    jmeUserPointer *pUser = (jmeUserPointer *) pHitObject->getUserPointer();

    env->SetObjectField(result, jmeClasses::PhysicsSweep_normal, normalvec);
    env->SetFloatField(result, jmeClasses::PhysicsSweep_hitFraction,
            hitFraction);
    env->SetIntField(result, jmeClasses::PhysicsSweep_partIndex, partIndex);
    env->SetIntField(result, jmeClasses::PhysicsSweep_triangleIndex,
            triangleIndex);
    env->SetObjectField(result, jmeClasses::PhysicsSweep_collisionObject,
            pUser->javaCollisionObject);

    env->CallBooleanMethod(resultList, jmeClasses::PhysicsSweep_addmethod,
            result);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }
}

void jmeBulletUtil::convert(JNIEnv *env, jobject in, btTransform *ptOut,
        btVector3 *pvOutScale) {
    NULL_CHECK(in, "The input Transform does not exist.",)
    NULL_CHECK(ptOut, "The output btTransform does not exist.",);
    NULL_CHECK(pvOutScale, "The output btVector3 does not exist.",);

    jobject translation_vec
            = env->CallObjectMethod(in, jmeClasses::Transform_translation);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }

    jobject rot_quat = env->CallObjectMethod(in, jmeClasses::Transform_rotation);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }

    jobject scale_vec = env->CallObjectMethod(in, jmeClasses::Transform_scale);
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return;
    }

    btVector3 native_translation_vec;
    convert(env, translation_vec, &native_translation_vec);
    ptOut->setOrigin(native_translation_vec);

    btQuaternion native_rot_quat;
    convert(env, rot_quat, &native_rot_quat);
    ptOut->setRotation(native_rot_quat);

    convert(env, scale_vec, pvOutScale);
}
