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
#include "jmeClasses.h"
#include <stdio.h>
#include "LinearMath/btThreads.h"

/*
 * Author: Normen Hansen, Empire Phoenix, Lutherion
 */

// public fields

JavaVM * jmeClasses::vm;

jclass jmeClasses::IllegalArgumentException;
jclass jmeClasses::RuntimeException;

jmethodID jmeClasses::List_addMethod;

jmethodID jmeClasses::CollisionSpace_notifyCollisionGroupListeners;

jmethodID jmeClasses::PhysicsSpace_preTick;
jmethodID jmeClasses::PhysicsSpace_postTick;
jmethodID jmeClasses::PhysicsSpace_onContactEnded;
jmethodID jmeClasses::PhysicsSpace_onContactProcessed;
jmethodID jmeClasses::PhysicsSpace_onContactStarted;

jmethodID jmeClasses::PhysicsGhostObject_addOverlappingObject;

jfieldID jmeClasses::Vec3d_x;
jfieldID jmeClasses::Vec3d_y;
jfieldID jmeClasses::Vec3d_z;
jmethodID jmeClasses::Vec3d_set;

jclass jmeClasses::Vector3f;
jfieldID jmeClasses::Vector3f_x;
jfieldID jmeClasses::Vector3f_y;
jfieldID jmeClasses::Vector3f_z;
jmethodID jmeClasses::Vector3f_set;

jfieldID jmeClasses::Quatd_x;
jfieldID jmeClasses::Quatd_y;
jfieldID jmeClasses::Quatd_z;
jfieldID jmeClasses::Quatd_w;
jmethodID jmeClasses::Quatd_set;

jfieldID jmeClasses::Quaternion_x;
jfieldID jmeClasses::Quaternion_y;
jfieldID jmeClasses::Quaternion_z;
jfieldID jmeClasses::Quaternion_w;
jmethodID jmeClasses::Quaternion_set;

jfieldID jmeClasses::Matrix3d_m00;
jfieldID jmeClasses::Matrix3d_m01;
jfieldID jmeClasses::Matrix3d_m02;
jfieldID jmeClasses::Matrix3d_m10;
jfieldID jmeClasses::Matrix3d_m11;
jfieldID jmeClasses::Matrix3d_m12;
jfieldID jmeClasses::Matrix3d_m20;
jfieldID jmeClasses::Matrix3d_m21;
jfieldID jmeClasses::Matrix3d_m22;

jfieldID jmeClasses::Matrix3f_m00;
jfieldID jmeClasses::Matrix3f_m01;
jfieldID jmeClasses::Matrix3f_m02;
jfieldID jmeClasses::Matrix3f_m10;
jfieldID jmeClasses::Matrix3f_m11;
jfieldID jmeClasses::Matrix3f_m12;
jfieldID jmeClasses::Matrix3f_m20;
jfieldID jmeClasses::Matrix3f_m21;
jfieldID jmeClasses::Matrix3f_m22;

jclass jmeClasses::NativeLibrary_Class;
jmethodID jmeClasses::NativeLibrary_reinitialization;

jclass jmeClasses::NullPointerException;

jmethodID jmeClasses::DebugMeshCallback_addVector;

jclass jmeClasses::PhysicsCollisionEvent_Class;
jmethodID jmeClasses::PhysicsCollisionEvent_init;

jmethodID jmeClasses::PhysicsCollisionListener_method;

jclass jmeClasses::PhysicsRay_Class;
jfieldID jmeClasses::PhysicsRay_collisionObject;
jfieldID jmeClasses::PhysicsRay_hitFraction;
jfieldID jmeClasses::PhysicsRay_normal;
jfieldID jmeClasses::PhysicsRay_partIndex;
jfieldID jmeClasses::PhysicsRay_triangleIndex;

jclass jmeClasses::PhysicsSweep_Class;
jfieldID jmeClasses::PhysicsSweep_collisionObject;
jfieldID jmeClasses::PhysicsSweep_hitFraction;
jfieldID jmeClasses::PhysicsSweep_normal;
jfieldID jmeClasses::PhysicsSweep_partIndex;
jfieldID jmeClasses::PhysicsSweep_triangleIndex;

jmethodID jmeClasses::Transform_rotation;
jmethodID jmeClasses::Transform_translation;
jmethodID jmeClasses::Transform_scale;

jclass jmeClasses::Vhacd4;
jmethodID jmeClasses::Vhacd4_addHull;
jmethodID jmeClasses::Vhacd4_update;

jclass jmeClasses::Vhacd;
jmethodID jmeClasses::Vhacd_addHull;
jmethodID jmeClasses::Vhacd_update;
/*
 * global flag to enable/disable the initialization message
 *
 * Invoke Java_com_jme3_bullet_util_NativeLibrary_setStartupMessageEnabled
 * to alter this flag.
 */
bool jmeClasses::printFlag = true;
/*
 * global flag to enable/disable the reinitialization callback
 *
 * Invoke Java_com_jme3_bullet_util_NativeLibrary_setReinitializationCallbackEnabled
 * to alter this flag.
 */
bool jmeClasses::reinitializationCallbackFlag = false;
/*
 * macros used to initialize the global variables:
 */
#define GLOBAL_CLASS(var, resource) { \
    (var) = pEnv->FindClass(resource); \
    EXCEPTION_CHK(pEnv,); \
    (var) = (jclass) pEnv->NewGlobalRef(var); \
    EXCEPTION_CHK(pEnv,); \
}
#define GLOBAL_FIELD(var, clss, name, sig) { \
    (var) = pEnv->GetFieldID((clss), (name), (sig)); \
    EXCEPTION_CHK(pEnv,); \
}
#define GLOBAL_METHOD(var, clss, name, sig) { \
    (var) = pEnv->GetMethodID((clss), (name), (sig)); \
    EXCEPTION_CHK(pEnv,); \
}
#define GLOBAL_STATIC_METHOD(var, clss, name, sig) { \
    (var) = pEnv->GetStaticMethodID((clss), (name), (sig)); \
    EXCEPTION_CHK(pEnv,); \
}

/*
 * Initialize this instance for the specified environment.
 */
void jmeClasses::initJavaClasses(JNIEnv *pEnv) {
    if (vm != NULL) { // already initialized
        if (jmeClasses::reinitializationCallbackFlag) {
            /*
             * Invoke NativeLibrary.reinitialization()
             * in order to perform incremental cleanup.
             */
            pEnv->CallStaticVoidMethod(NativeLibrary_Class,
                    NativeLibrary_reinitialization);
        }

        return;
    }

    if (printFlag) {
#ifdef _DEBUG
        printf("Debug_");
#endif

#ifdef DEBUG_PERSISTENCY
        printf("DebugPersistency_");
#endif // DEBUG_PERSISTENCY

#ifdef BT_USE_DOUBLE_PRECISION
        printf("Dp_");
#endif
#if BT_THREADSAFE
        printf("Mt_");
#endif
#ifdef BT_ENABLE_PROFILE
        printf("Quickprof_");
#endif
        printf("Libbulletjme version %s initializing\n", LIBBULLETJME_VERSION);
        fflush(stdout);
    }

#if BT_THREADSAFE
    btITaskScheduler *pScheduler = btGetOpenMPTaskScheduler();
    pScheduler->setNumThreads(2);
    btSetTaskScheduler(pScheduler);
#endif // BT_THREADSAFE

    pEnv->GetJavaVM(&vm);

    GLOBAL_CLASS(IllegalArgumentException,
            "java/lang/IllegalArgumentException");
    GLOBAL_CLASS(RuntimeException, "java/lang/RuntimeException");

    jclass list = pEnv->FindClass("java/util/List");
    EXCEPTION_CHK(pEnv,);
    GLOBAL_METHOD(List_addMethod, list, "add", "(Ljava/lang/Object;)Z");

    jclass collisionSpace = pEnv->FindClass("com/jme3/bullet/CollisionSpace");
    EXCEPTION_CHK(pEnv,);
    GLOBAL_METHOD(CollisionSpace_notifyCollisionGroupListeners,
            collisionSpace,
            "notifyCollisionGroupListeners",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;Lcom/jme3/bullet/collision/PhysicsCollisionObject;)Z"
    );

    jclass physicsSpace = pEnv->FindClass("com/jme3/bullet/PhysicsSpace");
    EXCEPTION_CHK(pEnv,);
    GLOBAL_METHOD(PhysicsSpace_preTick, physicsSpace, "preTick_native", "(F)V");
    GLOBAL_METHOD(PhysicsSpace_postTick, physicsSpace, "postTick_native", "(F)V");
    GLOBAL_METHOD(PhysicsSpace_onContactEnded,
            physicsSpace, "onContactEnded", "(J)V");
    GLOBAL_METHOD(PhysicsSpace_onContactProcessed,
            physicsSpace,
            "onContactProcessed",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;Lcom/jme3/bullet/collision/PhysicsCollisionObject;J)V"
    );
    GLOBAL_METHOD(PhysicsSpace_onContactStarted,
            physicsSpace, "onContactStarted", "(J)V");

    jclass physicsGhostObject
            = pEnv->FindClass("com/jme3/bullet/objects/PhysicsGhostObject");
    EXCEPTION_CHK(pEnv,);
    GLOBAL_METHOD(PhysicsGhostObject_addOverlappingObject,
            physicsGhostObject,
            "addOverlappingObject_native",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;)V"
    );

    jclass vec3d = pEnv->FindClass("com/simsilica/mathd/Vec3d");
    if (pEnv->ExceptionCheck()) {
        pEnv->ExceptionClear();

        printf("WARNING: Libbulletjme didn't find the SimMath library.\n");
        fflush(stdout);

        Vec3d_x = NULL;
        Vec3d_y = NULL;
        Vec3d_z = NULL;
        Vec3d_set = NULL;

        Quatd_x = NULL;
        Quatd_y = NULL;
        Quatd_z = NULL;
        Quatd_w = NULL;
        Quatd_set = NULL;

        Matrix3d_m00 = NULL;
        Matrix3d_m01 = NULL;
        Matrix3d_m02 = NULL;
        Matrix3d_m10 = NULL;
        Matrix3d_m11 = NULL;
        Matrix3d_m12 = NULL;
        Matrix3d_m20 = NULL;
        Matrix3d_m21 = NULL;
        Matrix3d_m22 = NULL;

    } else {
        GLOBAL_FIELD(Vec3d_x, vec3d, "x", "D");
        GLOBAL_FIELD(Vec3d_y, vec3d, "y", "D");
        GLOBAL_FIELD(Vec3d_z, vec3d, "z", "D");
        GLOBAL_METHOD(Vec3d_set,
                vec3d, "set", "(DDD)Lcom/simsilica/mathd/Vec3d;");

        jclass quatd = pEnv->FindClass("com/simsilica/mathd/Quatd");
        EXCEPTION_CHK(pEnv,);
        GLOBAL_FIELD(Quatd_x, quatd, "x", "D");
        GLOBAL_FIELD(Quatd_y, quatd, "y", "D");
        GLOBAL_FIELD(Quatd_z, quatd, "z", "D");
        GLOBAL_FIELD(Quatd_w, quatd, "w", "D");
        GLOBAL_METHOD(Quatd_set,
                quatd, "set", "(DDDD)Lcom/simsilica/mathd/Quatd;");

        jclass matrix3d = pEnv->FindClass("com/simsilica/mathd/Matrix3d");
        EXCEPTION_CHK(pEnv,);
        GLOBAL_FIELD(Matrix3d_m00, matrix3d, "m00", "D");
        GLOBAL_FIELD(Matrix3d_m01, matrix3d, "m01", "D");
        GLOBAL_FIELD(Matrix3d_m02, matrix3d, "m02", "D");
        GLOBAL_FIELD(Matrix3d_m10, matrix3d, "m10", "D");
        GLOBAL_FIELD(Matrix3d_m11, matrix3d, "m11", "D");
        GLOBAL_FIELD(Matrix3d_m12, matrix3d, "m12", "D");
        GLOBAL_FIELD(Matrix3d_m20, matrix3d, "m20", "D");
        GLOBAL_FIELD(Matrix3d_m21, matrix3d, "m21", "D");
        GLOBAL_FIELD(Matrix3d_m22, matrix3d, "m22", "D");
    }

    GLOBAL_CLASS(Vector3f, "com/jme3/math/Vector3f");
    GLOBAL_FIELD(Vector3f_x, Vector3f, "x", "F");
    GLOBAL_FIELD(Vector3f_y, Vector3f, "y", "F");
    GLOBAL_FIELD(Vector3f_z, Vector3f, "z", "F");
    GLOBAL_METHOD(Vector3f_set,
            Vector3f, "set", "(FFF)Lcom/jme3/math/Vector3f;");

    jclass quaternion = pEnv->FindClass("com/jme3/math/Quaternion");
    EXCEPTION_CHK(pEnv,);
    GLOBAL_FIELD(Quaternion_x, quaternion, "x", "F");
    GLOBAL_FIELD(Quaternion_y, quaternion, "y", "F");
    GLOBAL_FIELD(Quaternion_z, quaternion, "z", "F");
    GLOBAL_FIELD(Quaternion_w, quaternion, "w", "F");
    GLOBAL_METHOD(Quaternion_set,
            quaternion, "set", "(FFFF)Lcom/jme3/math/Quaternion;");

    jclass matrix3f = pEnv->FindClass("com/jme3/math/Matrix3f");
    EXCEPTION_CHK(pEnv,);
    GLOBAL_FIELD(Matrix3f_m00, matrix3f, "m00", "F");
    GLOBAL_FIELD(Matrix3f_m01, matrix3f, "m01", "F");
    GLOBAL_FIELD(Matrix3f_m02, matrix3f, "m02", "F");
    GLOBAL_FIELD(Matrix3f_m10, matrix3f, "m10", "F");
    GLOBAL_FIELD(Matrix3f_m11, matrix3f, "m11", "F");
    GLOBAL_FIELD(Matrix3f_m12, matrix3f, "m12", "F");
    GLOBAL_FIELD(Matrix3f_m20, matrix3f, "m20", "F");
    GLOBAL_FIELD(Matrix3f_m21, matrix3f, "m21", "F");
    GLOBAL_FIELD(Matrix3f_m22, matrix3f, "m22", "F");

    GLOBAL_CLASS(NullPointerException, "java/lang/NullPointerException");

    jclass debugMeshCallback
            = pEnv->FindClass("com/jme3/bullet/util/DebugMeshCallback");
    EXCEPTION_CHK(pEnv,);

    GLOBAL_METHOD(DebugMeshCallback_addVector,
            debugMeshCallback, "addVector", "(FFFII)V");

    GLOBAL_CLASS(PhysicsCollisionEvent_Class,
            "com/jme3/bullet/collision/PhysicsCollisionEvent");
    GLOBAL_METHOD(PhysicsCollisionEvent_init,
            PhysicsCollisionEvent_Class,
            "<init>",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;Lcom/jme3/bullet/collision/PhysicsCollisionObject;J)V"
    );

    jclass physicsCollisionListener = pEnv->FindClass(
            "com/jme3/bullet/collision/PhysicsCollisionListener");
    EXCEPTION_CHK(pEnv,);
    GLOBAL_METHOD(PhysicsCollisionListener_method,
            physicsCollisionListener,
            "collision",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionEvent;)V"
    );

    GLOBAL_CLASS(PhysicsRay_Class,
            "com/jme3/bullet/collision/PhysicsRayTestResult");

    GLOBAL_FIELD(PhysicsRay_collisionObject,
            PhysicsRay_Class,
            "collisionObject",
            "Lcom/jme3/bullet/collision/PhysicsCollisionObject;"
    );
    GLOBAL_FIELD(PhysicsRay_hitFraction, PhysicsRay_Class, "hitFraction", "F");
    GLOBAL_FIELD(PhysicsRay_normal,
            PhysicsRay_Class, "normal", "Lcom/jme3/math/Vector3f;");
    GLOBAL_FIELD(PhysicsRay_partIndex, PhysicsRay_Class, "partIndex", "I");
    GLOBAL_FIELD(PhysicsRay_triangleIndex,
            PhysicsRay_Class, "triangleIndex", "I");

    GLOBAL_CLASS(PhysicsSweep_Class,
            "com/jme3/bullet/collision/PhysicsSweepTestResult");
    GLOBAL_FIELD(PhysicsSweep_collisionObject,
            PhysicsSweep_Class,
            "collisionObject",
            "Lcom/jme3/bullet/collision/PhysicsCollisionObject;"
    );
    GLOBAL_FIELD(PhysicsSweep_hitFraction,
            PhysicsSweep_Class, "hitFraction", "F");
    GLOBAL_FIELD(PhysicsSweep_normal,
            PhysicsSweep_Class, "normal", "Lcom/jme3/math/Vector3f;");
    GLOBAL_FIELD(PhysicsSweep_partIndex,
            PhysicsSweep_Class, "partIndex", "I");
    GLOBAL_FIELD(PhysicsSweep_triangleIndex,
            PhysicsSweep_Class, "triangleIndex", "I");

    jclass transform = pEnv->FindClass("com/jme3/math/Transform");
    EXCEPTION_CHK(pEnv,);

    GLOBAL_METHOD(Transform_rotation,
            transform, "getRotation", "()Lcom/jme3/math/Quaternion;");

    GLOBAL_METHOD(Transform_scale,
            transform, "getScale", "()Lcom/jme3/math/Vector3f;");

    GLOBAL_METHOD(Transform_translation,
            transform, "getTranslation", "()Lcom/jme3/math/Vector3f;");

    GLOBAL_CLASS(Vhacd4, "vhacd4/Vhacd4");
    GLOBAL_STATIC_METHOD(Vhacd4_addHull, Vhacd4, "addHull", "(J)V");
    GLOBAL_STATIC_METHOD(Vhacd4_update,
            Vhacd4, "update", "(DDDLjava/lang/String;Ljava/lang/String;)V");

    GLOBAL_CLASS(Vhacd, "vhacd/VHACD");
    GLOBAL_STATIC_METHOD(Vhacd_addHull, Vhacd, "addHull", "(J)V");
    GLOBAL_STATIC_METHOD(Vhacd_update,
            Vhacd, "update", "(DDDLjava/lang/String;Ljava/lang/String;)V");

    GLOBAL_CLASS(NativeLibrary_Class, "com/jme3/bullet/util/NativeLibrary");
    GLOBAL_STATIC_METHOD(NativeLibrary_reinitialization,
            NativeLibrary_Class, "reinitialization", "()V");
    /*
     * Invoke NativeLibrary.postInitialization()
     * in order to start the Physics Cleaner thread.
     */
    jmethodID postInitialization = pEnv->GetStaticMethodID(
            NativeLibrary_Class, "postInitialization", "()V");
    EXCEPTION_CHK(pEnv,);
    pEnv->CallStaticVoidMethod(NativeLibrary_Class, postInitialization);
}
