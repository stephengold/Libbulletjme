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
#ifndef _Included_jmeClasses
#define _Included_jmeClasses

#include <jni.h>

#define LIBBULLETJME_VERSION "17.5.1"

#define EXCEPTION_CHK(pEnv, retval) \
    if (pEnv->ExceptionCheck()) { \
        return retval; \
    }

#ifdef _DEBUG
#define ASSERT_CHK(pEnv, assertion, retval) \
    if (!(assertion)) { \
        (pEnv)->ThrowNew(jmeClasses::RuntimeException, "expected " #assertion); \
        return retval; \
    }
#define NULL_CHK(pEnv, pointer, message, retval) \
    if ((pointer) == NULL) { \
        (pEnv)->ThrowNew(jmeClasses::NullPointerException, message); \
        return retval; \
    }
#else
#define ASSERT_CHK(pEnv, assertion, retval)
#define NULL_CHK(pEnv, pointer, message, retval)
#endif

class jmeClasses {
public:
    static void initJavaClasses(JNIEnv *);

    // pointer to the Java virtual machine
    static JavaVM * vm;

    static jclass IllegalArgumentException;
    static jclass RuntimeException;

    static jmethodID List_addMethod;

    static jmethodID CollisionSpace_notifyCollisionGroupListeners;

    static jmethodID PhysicsSpace_preTick;
    static jmethodID PhysicsSpace_postTick;
    static jmethodID PhysicsSpace_onContactEnded;
    static jmethodID PhysicsSpace_onContactProcessed;
    static jmethodID PhysicsSpace_onContactStarted;

    static jmethodID PhysicsGhostObject_addOverlappingObject;

    static jfieldID Vec3d_x;
    static jfieldID Vec3d_y;
    static jfieldID Vec3d_z;
    static jmethodID Vec3d_set;

    static jclass Vector3f;
    static jfieldID Vector3f_x;
    static jfieldID Vector3f_y;
    static jfieldID Vector3f_z;
    static jmethodID Vector3f_set;

    static jfieldID Quatd_x;
    static jfieldID Quatd_y;
    static jfieldID Quatd_z;
    static jfieldID Quatd_w;
    static jmethodID Quatd_set;

    static jfieldID Quaternion_x;
    static jfieldID Quaternion_y;
    static jfieldID Quaternion_z;
    static jfieldID Quaternion_w;
    static jmethodID Quaternion_set;

    static jfieldID Matrix3d_m00;
    static jfieldID Matrix3d_m01;
    static jfieldID Matrix3d_m02;
    static jfieldID Matrix3d_m10;
    static jfieldID Matrix3d_m11;
    static jfieldID Matrix3d_m12;
    static jfieldID Matrix3d_m20;
    static jfieldID Matrix3d_m21;
    static jfieldID Matrix3d_m22;

    static jfieldID Matrix3f_m00;
    static jfieldID Matrix3f_m01;
    static jfieldID Matrix3f_m02;
    static jfieldID Matrix3f_m10;
    static jfieldID Matrix3f_m11;
    static jfieldID Matrix3f_m12;
    static jfieldID Matrix3f_m20;
    static jfieldID Matrix3f_m21;
    static jfieldID Matrix3f_m22;

    static jclass NativeLibrary_Class;
    static jmethodID NativeLibrary_reinitialization;

    static jclass NullPointerException;

    static jclass PhysicsCollisionEvent_Class;
    static jmethodID PhysicsCollisionEvent_init;

    static jmethodID PhysicsCollisionListener_method;

    static jclass PhysicsRay_Class;
    static jfieldID PhysicsRay_collisionObject;
    static jfieldID PhysicsRay_hitFraction;
    static jfieldID PhysicsRay_normal;
    static jfieldID PhysicsRay_partIndex;
    static jfieldID PhysicsRay_triangleIndex;

    static jclass PhysicsSweep_Class;
    static jfieldID PhysicsSweep_collisionObject;
    static jfieldID PhysicsSweep_hitFraction;
    static jfieldID PhysicsSweep_normal;
    static jfieldID PhysicsSweep_partIndex;
    static jfieldID PhysicsSweep_triangleIndex;

    static jmethodID Transform_rotation;
    static jmethodID Transform_translation;
    static jmethodID Transform_scale;

    static jclass Vhacd4;
    static jmethodID Vhacd4_addHull;
    static jmethodID Vhacd4_update;

    static jclass Vhacd;
    static jmethodID Vhacd_addHull;
    static jmethodID Vhacd_update;

    static jmethodID DebugMeshCallback_addVector;

    static bool printFlag;
    static bool reinitializationCallbackFlag;

private:
    jmeClasses() {
    }

    ~jmeClasses() {
    }
};

#endif
