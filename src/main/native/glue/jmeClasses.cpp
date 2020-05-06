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

/*
 * Author: Normen Hansen, Empire Phoenix, Lutherion
 */

// public fields

JavaVM * jmeClasses::vm;

jclass jmeClasses::IllegalArgumentException;

jclass jmeClasses::CollisionSpace;
jmethodID jmeClasses::CollisionSpace_notifyCollisionGroupListeners;

jclass jmeClasses::PhysicsSpace;
jmethodID jmeClasses::PhysicsSpace_preTick;
jmethodID jmeClasses::PhysicsSpace_postTick;
jmethodID jmeClasses::PhysicsSpace_addCollisionEvent;

jclass jmeClasses::PhysicsGhostObject;
jmethodID jmeClasses::PhysicsGhostObject_addOverlappingObject;

jclass jmeClasses::Vector3f;
jfieldID jmeClasses::Vector3f_x;
jfieldID jmeClasses::Vector3f_y;
jfieldID jmeClasses::Vector3f_z;

jclass jmeClasses::Quaternion;
jfieldID jmeClasses::Quaternion_x;
jfieldID jmeClasses::Quaternion_y;
jfieldID jmeClasses::Quaternion_z;
jfieldID jmeClasses::Quaternion_w;

jclass jmeClasses::Matrix3f;
jfieldID jmeClasses::Matrix3f_m00;
jfieldID jmeClasses::Matrix3f_m01;
jfieldID jmeClasses::Matrix3f_m02;
jfieldID jmeClasses::Matrix3f_m10;
jfieldID jmeClasses::Matrix3f_m11;
jfieldID jmeClasses::Matrix3f_m12;
jfieldID jmeClasses::Matrix3f_m20;
jfieldID jmeClasses::Matrix3f_m21;
jfieldID jmeClasses::Matrix3f_m22;

jclass jmeClasses::NullPointerException;

jclass jmeClasses::DebugMeshCallback;
jmethodID jmeClasses::DebugMeshCallback_addVector;

jclass jmeClasses::PhysicsRay_Class;
jmethodID jmeClasses::PhysicsRay_newSingleResult;
jfieldID jmeClasses::PhysicsRay_collisionObject;
jfieldID jmeClasses::PhysicsRay_hitFraction;
jfieldID jmeClasses::PhysicsRay_normal;
jfieldID jmeClasses::PhysicsRay_partIndex;
jfieldID jmeClasses::PhysicsRay_triangleIndex;

jclass jmeClasses::PhysicsRay_listresult;
jmethodID jmeClasses::PhysicsRay_addmethod;

jclass jmeClasses::PhysicsSweep_Class;
jmethodID jmeClasses::PhysicsSweep_newSingleResult;
jfieldID jmeClasses::PhysicsSweep_collisionObject;
jfieldID jmeClasses::PhysicsSweep_hitFraction;
jfieldID jmeClasses::PhysicsSweep_normal;
jfieldID jmeClasses::PhysicsSweep_partIndex;
jfieldID jmeClasses::PhysicsSweep_triangleIndex;

jclass jmeClasses::PhysicsSweep_listresult;
jmethodID jmeClasses::PhysicsSweep_addmethod;

jclass jmeClasses::Transform;
jmethodID jmeClasses::Transform_rotation;
jmethodID jmeClasses::Transform_translation;
jmethodID jmeClasses::Transform_scale;

jclass jmeClasses::Vhacd;
jmethodID jmeClasses::Vhacd_addHull;
jmethodID jmeClasses::Vhacd_update;

/*
 * global flag to enable/disable the init message
 *
 * Invoke Java_com_jme3_bullet_util_NativeLibrary_setStartupMessageEnabled
 * to alter this flag.
 */
int jmeClasses::printFlag = 1; // TRUE

/*
 * Initialize this instance for the specified environment.
 */
void jmeClasses::initJavaClasses(JNIEnv *pEnv) {
    if (vm != NULL) return; // already initialized

    if (printFlag) {
#ifdef _DEBUG
        printf("Debug_");
#endif
#ifdef BT_USE_DOUBLE_PRECISION
        printf("Dp_");
#endif
        printf("Libbulletjme version 5.8.0 initializing\n");
        fflush(stdout);
    }

    pEnv->GetJavaVM(&vm);

    IllegalArgumentException = (jclass) pEnv->NewGlobalRef(pEnv->FindClass(
            "java/lang/IllegalArgumentException"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    CollisionSpace = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/bullet/CollisionSpace"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    CollisionSpace_notifyCollisionGroupListeners = pEnv->GetMethodID(
            CollisionSpace, "notifyCollisionGroupListeners_native",
            "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;Lcom/jme3/bullet/collision/PhysicsCollisionObject;)Z");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsSpace = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/bullet/PhysicsSpace"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsSpace_preTick = pEnv->GetMethodID(PhysicsSpace, "preTick_native", "(F)V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsSpace_postTick = pEnv->GetMethodID(PhysicsSpace, "postTick_native", "(F)V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsSpace_addCollisionEvent = pEnv->GetMethodID(PhysicsSpace, "addCollisionEvent_native", "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;Lcom/jme3/bullet/collision/PhysicsCollisionObject;J)V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsGhostObject = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/bullet/objects/PhysicsGhostObject"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsGhostObject_addOverlappingObject = pEnv->GetMethodID(PhysicsGhostObject, "addOverlappingObject_native", "(Lcom/jme3/bullet/collision/PhysicsCollisionObject;)V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Vector3f = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/math/Vector3f"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Vector3f_x = pEnv->GetFieldID(Vector3f, "x", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Vector3f_y = pEnv->GetFieldID(Vector3f, "y", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Vector3f_z = pEnv->GetFieldID(Vector3f, "z", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Quaternion = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/math/Quaternion"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Quaternion_x = pEnv->GetFieldID(Quaternion, "x", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Quaternion_y = pEnv->GetFieldID(Quaternion, "y", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Quaternion_z = pEnv->GetFieldID(Quaternion, "z", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Quaternion_w = pEnv->GetFieldID(Quaternion, "w", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Matrix3f = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/math/Matrix3f"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m00 = pEnv->GetFieldID(Matrix3f, "m00", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m01 = pEnv->GetFieldID(Matrix3f, "m01", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m02 = pEnv->GetFieldID(Matrix3f, "m02", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m10 = pEnv->GetFieldID(Matrix3f, "m10", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m11 = pEnv->GetFieldID(Matrix3f, "m11", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m12 = pEnv->GetFieldID(Matrix3f, "m12", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m20 = pEnv->GetFieldID(Matrix3f, "m20", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m21 = pEnv->GetFieldID(Matrix3f, "m21", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    Matrix3f_m22 = pEnv->GetFieldID(Matrix3f, "m22", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    NullPointerException = (jclass) pEnv->NewGlobalRef(pEnv->FindClass(
            "java/lang/NullPointerException"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    DebugMeshCallback = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/bullet/util/DebugMeshCallback"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    DebugMeshCallback_addVector = pEnv->GetMethodID(DebugMeshCallback, "addVector", "(FFFII)V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsRay_Class = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/bullet/collision/PhysicsRayTestResult"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsRay_newSingleResult = pEnv->GetMethodID(PhysicsRay_Class, "<init>", "()V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsRay_collisionObject = pEnv->GetFieldID(PhysicsRay_Class,
            "collisionObject",
            "Lcom/jme3/bullet/collision/PhysicsCollisionObject;");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsRay_hitFraction = pEnv->GetFieldID(PhysicsRay_Class, "hitFraction",
            "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsRay_normal = pEnv->GetFieldID(PhysicsRay_Class, "normal",
            "Lcom/jme3/math/Vector3f;");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsRay_partIndex = pEnv->GetFieldID(PhysicsRay_Class, "partIndex", "I");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsRay_triangleIndex = pEnv->GetFieldID(PhysicsRay_Class,
            "triangleIndex", "I");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsRay_listresult = pEnv->FindClass("java/util/List");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsRay_listresult = (jclass) pEnv->NewGlobalRef(PhysicsRay_listresult);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsRay_addmethod = pEnv->GetMethodID(PhysicsRay_listresult, "add", "(Ljava/lang/Object;)Z");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsSweep_Class = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/bullet/collision/PhysicsSweepTestResult"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsSweep_newSingleResult = pEnv->GetMethodID(PhysicsSweep_Class, "<init>", "()V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsSweep_collisionObject = pEnv->GetFieldID(PhysicsSweep_Class,
            "collisionObject",
            "Lcom/jme3/bullet/collision/PhysicsCollisionObject;");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsSweep_hitFraction = pEnv->GetFieldID(PhysicsSweep_Class,
            "hitFraction", "F");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsSweep_normal = pEnv->GetFieldID(PhysicsSweep_Class, "normal",
            "Lcom/jme3/math/Vector3f;");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsSweep_partIndex = pEnv->GetFieldID(PhysicsSweep_Class, "partIndex",
            "I");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
    PhysicsSweep_triangleIndex = pEnv->GetFieldID(PhysicsSweep_Class,
            "triangleIndex", "I");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsSweep_listresult = pEnv->FindClass("java/util/List");
    PhysicsSweep_listresult = (jclass) pEnv->NewGlobalRef(PhysicsSweep_listresult);
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    PhysicsSweep_addmethod = pEnv->GetMethodID(PhysicsSweep_listresult, "add", "(Ljava/lang/Object;)Z");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Transform = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("com/jme3/math/Transform"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Transform_rotation = pEnv->GetMethodID(Transform, "getRotation", "()Lcom/jme3/math/Quaternion;");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Transform_translation = pEnv->GetMethodID(Transform, "getTranslation", "()Lcom/jme3/math/Vector3f;");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Transform_scale = pEnv->GetMethodID(Transform, "getScale",
            "()Lcom/jme3/math/Vector3f;");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Vhacd = (jclass) pEnv->NewGlobalRef(pEnv->FindClass("vhacd/VHACD"));
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Vhacd_addHull = pEnv->GetStaticMethodID(Vhacd, "addHull", "(J)V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }

    Vhacd_update = pEnv->GetStaticMethodID(Vhacd, "update", "(DDDLjava/lang/String;Ljava/lang/String;)V");
    if (pEnv->ExceptionCheck()) {
        pEnv->Throw(pEnv->ExceptionOccurred());
        return;
    }
}