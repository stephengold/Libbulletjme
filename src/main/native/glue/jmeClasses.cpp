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
            EXCEPTION_CHK(pEnv,);
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

    IllegalArgumentException = (jclass) pEnv->NewGlobalRef(pEnv->FindClass(
            "java/lang/IllegalArgumentException"));
    EXCEPTION_CHK(pEnv,);

    jclass List = pEnv->FindClass("java/util/List");
    EXCEPTION_CHK(pEnv,);
    List_addMethod = pEnv->GetMethodID(List, "add", "(Ljava/lang/Object;)Z");
    EXCEPTION_CHK(pEnv,);

    jclass CollisionSpace = pEnv->FindClass("com/jme3/bullet/CollisionSpace");
    EXCEPTION_CHK(pEnv,);
    CollisionSpace_notifyCollisionGroupListeners = pEnv->GetMethodID(
            CollisionSpace, "notifyCollisionGroupListeners_native",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;Lcom/jme3/bullet/collision/PhysicsCollisionObject;)Z");
    EXCEPTION_CHK(pEnv,);

    jclass PhysicsSpace = pEnv->FindClass("com/jme3/bullet/PhysicsSpace");
    EXCEPTION_CHK(pEnv,);
    PhysicsSpace_preTick
            = pEnv->GetMethodID(PhysicsSpace, "preTick_native", "(F)V");
    EXCEPTION_CHK(pEnv,);
    PhysicsSpace_postTick
            = pEnv->GetMethodID(PhysicsSpace, "postTick_native", "(F)V");
    EXCEPTION_CHK(pEnv,);
    PhysicsSpace_onContactEnded
            = pEnv->GetMethodID(PhysicsSpace, "onContactEnded", "(J)V");
    EXCEPTION_CHK(pEnv,);
    PhysicsSpace_onContactProcessed = pEnv->GetMethodID(PhysicsSpace,
            "onContactProcessed",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;Lcom/jme3/bullet/collision/PhysicsCollisionObject;J)V"
    );
    EXCEPTION_CHK(pEnv,);
    PhysicsSpace_onContactStarted
            = pEnv->GetMethodID(PhysicsSpace, "onContactStarted", "(J)V");
    EXCEPTION_CHK(pEnv,);

    jclass PhysicsGhostObject
            = pEnv->FindClass("com/jme3/bullet/objects/PhysicsGhostObject");
    EXCEPTION_CHK(pEnv,);
    PhysicsGhostObject_addOverlappingObject = pEnv->GetMethodID(PhysicsGhostObject,
            "addOverlappingObject_native",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;)V");
    EXCEPTION_CHK(pEnv,);

    jclass Vec3d = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/simsilica/mathd/Vec3d"));
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
        Vec3d_x = pEnv->GetFieldID(Vec3d, "x", "D");
        EXCEPTION_CHK(pEnv,);
        Vec3d_y = pEnv->GetFieldID(Vec3d, "y", "D");
        EXCEPTION_CHK(pEnv,);
        Vec3d_z = pEnv->GetFieldID(Vec3d, "z", "D");
        EXCEPTION_CHK(pEnv,);
        Vec3d_set = pEnv->GetMethodID(Vec3d, "set", "(DDD)Lcom/simsilica/mathd/Vec3d;");
        EXCEPTION_CHK(pEnv,);

        jclass Quatd = pEnv->FindClass("com/simsilica/mathd/Quatd");
        EXCEPTION_CHK(pEnv,);
        Quatd_x = pEnv->GetFieldID(Quatd, "x", "D");
        EXCEPTION_CHK(pEnv,);
        Quatd_y = pEnv->GetFieldID(Quatd, "y", "D");
        EXCEPTION_CHK(pEnv,);
        Quatd_z = pEnv->GetFieldID(Quatd, "z", "D");
        EXCEPTION_CHK(pEnv,);
        Quatd_w = pEnv->GetFieldID(Quatd, "w", "D");
        EXCEPTION_CHK(pEnv,);
        Quatd_set = pEnv->GetMethodID(Quatd, "set", "(DDDD)Lcom/simsilica/mathd/Quatd;");
        EXCEPTION_CHK(pEnv,);

        jclass Matrix3d = pEnv->FindClass("com/simsilica/mathd/Matrix3d");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m00 = pEnv->GetFieldID(Matrix3d, "m00", "D");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m01 = pEnv->GetFieldID(Matrix3d, "m01", "D");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m02 = pEnv->GetFieldID(Matrix3d, "m02", "D");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m10 = pEnv->GetFieldID(Matrix3d, "m10", "D");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m11 = pEnv->GetFieldID(Matrix3d, "m11", "D");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m12 = pEnv->GetFieldID(Matrix3d, "m12", "D");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m20 = pEnv->GetFieldID(Matrix3d, "m20", "D");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m21 = pEnv->GetFieldID(Matrix3d, "m21", "D");
        EXCEPTION_CHK(pEnv,);
        Matrix3d_m22 = pEnv->GetFieldID(Matrix3d, "m22", "D");
        EXCEPTION_CHK(pEnv,);
    }

    Vector3f = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/math/Vector3f"));
    EXCEPTION_CHK(pEnv,);
    Vector3f_x = pEnv->GetFieldID(Vector3f, "x", "F");
    EXCEPTION_CHK(pEnv,);
    Vector3f_y = pEnv->GetFieldID(Vector3f, "y", "F");
    EXCEPTION_CHK(pEnv,);
    Vector3f_z = pEnv->GetFieldID(Vector3f, "z", "F");
    EXCEPTION_CHK(pEnv,);
    Vector3f_set = pEnv->GetMethodID(Vector3f, "set", "(FFF)Lcom/jme3/math/Vector3f;");
    EXCEPTION_CHK(pEnv,);

    jclass Quaternion = pEnv->FindClass("com/jme3/math/Quaternion");
    EXCEPTION_CHK(pEnv,);
    Quaternion_x = pEnv->GetFieldID(Quaternion, "x", "F");
    EXCEPTION_CHK(pEnv,);
    Quaternion_y = pEnv->GetFieldID(Quaternion, "y", "F");
    EXCEPTION_CHK(pEnv,);
    Quaternion_z = pEnv->GetFieldID(Quaternion, "z", "F");
    EXCEPTION_CHK(pEnv,);
    Quaternion_w = pEnv->GetFieldID(Quaternion, "w", "F");
    EXCEPTION_CHK(pEnv,);
    Quaternion_set = pEnv->GetMethodID(Quaternion, "set", "(FFFF)Lcom/jme3/math/Quaternion;");
    EXCEPTION_CHK(pEnv,);

    jclass Matrix3f = pEnv->FindClass("com/jme3/math/Matrix3f");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m00 = pEnv->GetFieldID(Matrix3f, "m00", "F");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m01 = pEnv->GetFieldID(Matrix3f, "m01", "F");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m02 = pEnv->GetFieldID(Matrix3f, "m02", "F");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m10 = pEnv->GetFieldID(Matrix3f, "m10", "F");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m11 = pEnv->GetFieldID(Matrix3f, "m11", "F");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m12 = pEnv->GetFieldID(Matrix3f, "m12", "F");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m20 = pEnv->GetFieldID(Matrix3f, "m20", "F");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m21 = pEnv->GetFieldID(Matrix3f, "m21", "F");
    EXCEPTION_CHK(pEnv,);
    Matrix3f_m22 = pEnv->GetFieldID(Matrix3f, "m22", "F");
    EXCEPTION_CHK(pEnv,);

    NullPointerException = (jclass) pEnv->NewGlobalRef(pEnv->FindClass(
            "java/lang/NullPointerException"));
    EXCEPTION_CHK(pEnv,);

    jclass DebugMeshCallback
            = pEnv->FindClass("com/jme3/bullet/util/DebugMeshCallback");
    EXCEPTION_CHK(pEnv,);

    DebugMeshCallback_addVector
            = pEnv->GetMethodID(DebugMeshCallback, "addVector", "(FFFII)V");
    EXCEPTION_CHK(pEnv,);

    PhysicsCollisionEvent_Class = pEnv->FindClass(
            "com/jme3/bullet/collision/PhysicsCollisionEvent");
    EXCEPTION_CHK(pEnv,);
    PhysicsCollisionEvent_Class
            = (jclass) pEnv->NewGlobalRef(PhysicsCollisionEvent_Class);
    EXCEPTION_CHK(pEnv,);
    PhysicsCollisionEvent_init = pEnv->GetMethodID(PhysicsCollisionEvent_Class,
            "<init>",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;Lcom/jme3/bullet/collision/PhysicsCollisionObject;J)V");
    EXCEPTION_CHK(pEnv,);

    jclass physicsCollisionListener = pEnv->FindClass(
            "com/jme3/bullet/collision/PhysicsCollisionListener");
    EXCEPTION_CHK(pEnv,);
    PhysicsCollisionListener_method = pEnv->GetMethodID(
            physicsCollisionListener, "collision",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionEvent;)V");
    EXCEPTION_CHK(pEnv,);

    PhysicsRay_Class = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/bullet/collision/PhysicsRayTestResult"));
    EXCEPTION_CHK(pEnv,);

    PhysicsRay_collisionObject = pEnv->GetFieldID(PhysicsRay_Class,
            "collisionObject",
            "Lcom/jme3/bullet/collision/PhysicsCollisionObject;");
    EXCEPTION_CHK(pEnv,);
    PhysicsRay_hitFraction
            = pEnv->GetFieldID(PhysicsRay_Class, "hitFraction", "F");
    EXCEPTION_CHK(pEnv,);
    PhysicsRay_normal = pEnv->GetFieldID(
            PhysicsRay_Class, "normal", "Lcom/jme3/math/Vector3f;");
    EXCEPTION_CHK(pEnv,);
    PhysicsRay_partIndex = pEnv->GetFieldID(PhysicsRay_Class, "partIndex", "I");
    EXCEPTION_CHK(pEnv,);
    PhysicsRay_triangleIndex
            = pEnv->GetFieldID(PhysicsRay_Class, "triangleIndex", "I");
    EXCEPTION_CHK(pEnv,);

    PhysicsSweep_Class = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/bullet/collision/PhysicsSweepTestResult"));
    EXCEPTION_CHK(pEnv,);

    PhysicsSweep_collisionObject = pEnv->GetFieldID(PhysicsSweep_Class,
            "collisionObject",
            "Lcom/jme3/bullet/collision/PhysicsCollisionObject;");
    EXCEPTION_CHK(pEnv,);
    PhysicsSweep_hitFraction
            = pEnv->GetFieldID(PhysicsSweep_Class, "hitFraction", "F");
    EXCEPTION_CHK(pEnv,);
    PhysicsSweep_normal = pEnv->GetFieldID(
            PhysicsSweep_Class, "normal", "Lcom/jme3/math/Vector3f;");
    EXCEPTION_CHK(pEnv,);
    PhysicsSweep_partIndex
            = pEnv->GetFieldID(PhysicsSweep_Class, "partIndex", "I");
    EXCEPTION_CHK(pEnv,);
    PhysicsSweep_triangleIndex
            = pEnv->GetFieldID(PhysicsSweep_Class, "triangleIndex", "I");
    EXCEPTION_CHK(pEnv,);

    jclass Transform = pEnv->FindClass("com/jme3/math/Transform");
    EXCEPTION_CHK(pEnv,);

    Transform_rotation = pEnv->GetMethodID(
            Transform, "getRotation", "()Lcom/jme3/math/Quaternion;");
    EXCEPTION_CHK(pEnv,);

    Transform_translation = pEnv->GetMethodID(
            Transform, "getTranslation", "()Lcom/jme3/math/Vector3f;");
    EXCEPTION_CHK(pEnv,);

    Transform_scale = pEnv->GetMethodID(
            Transform, "getScale", "()Lcom/jme3/math/Vector3f;");
    EXCEPTION_CHK(pEnv,);

    Vhacd4 = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("vhacd4/Vhacd4"));
    EXCEPTION_CHK(pEnv,);

    Vhacd4_addHull = pEnv->GetStaticMethodID(Vhacd4, "addHull", "(J)V");
    EXCEPTION_CHK(pEnv,);

    Vhacd4_update = pEnv->GetStaticMethodID(Vhacd4, "update",
            "(DDDLjava/lang/String;Ljava/lang/String;)V");
    EXCEPTION_CHK(pEnv,);

    Vhacd = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("vhacd/VHACD"));
    EXCEPTION_CHK(pEnv,);

    Vhacd_addHull = pEnv->GetStaticMethodID(Vhacd, "addHull", "(J)V");
    EXCEPTION_CHK(pEnv,);

    Vhacd_update = pEnv->GetStaticMethodID(
            Vhacd, "update", "(DDDLjava/lang/String;Ljava/lang/String;)V");
    EXCEPTION_CHK(pEnv,);

    NativeLibrary_Class = pEnv->FindClass("com/jme3/bullet/util/NativeLibrary");
    EXCEPTION_CHK(pEnv,);
    NativeLibrary_Class = (jclass) pEnv->NewGlobalRef(NativeLibrary_Class);
    EXCEPTION_CHK(pEnv,);
    NativeLibrary_reinitialization = pEnv->GetStaticMethodID(
            NativeLibrary_Class, "reinitialization", "()V");
    EXCEPTION_CHK(pEnv,);
    /*
     * Invoke NativeLibrary.postInitialization()
     * in order to start the Physics Cleaner thread.
     */
    jmethodID postInitialization = pEnv->GetStaticMethodID(
            NativeLibrary_Class, "postInitialization", "()V");
    EXCEPTION_CHK(pEnv,);
    pEnv->CallStaticVoidMethod(NativeLibrary_Class, postInitialization);
    EXCEPTION_CHK(pEnv,);
}
