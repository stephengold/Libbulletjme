/*
 * Copyright (c) 2009-2015 jMonkeyEngine
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

/**
 * Author: Dokthar
 */
#include "com_jme3_bullet_objects_PhysicsSoftBody_Material.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
     * Method:    getLinearStiffnessFactor
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_getLinearStiffnessFactor
    (JNIEnv *env, jobject object, jlong matId) {
        btSoftBody::Material* mat = reinterpret_cast<btSoftBody::Material*> (matId);
        if (mat == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }
        return mat->m_kLST;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
     * Method:    setLinearStiffnessFactor
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_setLinearStiffnessFactor
    (JNIEnv *env, jobject object, jlong matId, jfloat factor) {
        btSoftBody::Material* mat = reinterpret_cast<btSoftBody::Material*> (matId);
        if (mat == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }
        mat->m_kLST = factor;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
     * Method:    getAngularStiffnessFactor
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_getAngularStiffnessFactor
    (JNIEnv *env, jobject object, jlong matId) {
        btSoftBody::Material* mat = reinterpret_cast<btSoftBody::Material*> (matId);
        if (mat == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }
        return mat->m_kAST;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
     * Method:    setAngularStiffnessFactor
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_setAngularStiffnessFactor
    (JNIEnv *env, jobject object, jlong matId, jfloat factor) {
        btSoftBody::Material* mat = reinterpret_cast<btSoftBody::Material*> (matId);
        if (mat == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }
        mat->m_kAST = factor;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
     * Method:    getVolumeStiffnessFactor
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_getVolumeStiffnessFactor
    (JNIEnv *env, jobject object, jlong matId) {
        btSoftBody::Material* mat = reinterpret_cast<btSoftBody::Material*> (matId);
        if (mat == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return 0;
        }
        return mat->m_kVST;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
     * Method:    setVolumeStiffnessFactor
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_setVolumeStiffnessFactor
    (JNIEnv *env, jobject object, jlong matId, jfloat factor) {
        btSoftBody::Material* mat = reinterpret_cast<btSoftBody::Material*> (matId);
        if (mat == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The native object does not exist.");
            return;
        }
        mat->m_kVST = factor;
    }

#ifdef __cplusplus
}
#endif
