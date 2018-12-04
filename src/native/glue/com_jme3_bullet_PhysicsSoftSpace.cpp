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
#include "com_jme3_bullet_PhysicsSoftSpace.h"
#include "jmePhysicsSoftSpace.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

/**
 * Author: Dokthar
 */
#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_PhysicsSoftSpace
     * Method:    createPhysicsSoftSpace
     * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;IZ)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_createPhysicsSoftSpace
    (JNIEnv *env, jobject object, jobject min_vec, jobject max_vec, jint broadphase, jboolean threading) {
        jmeClasses::initJavaClasses(env);
        jmePhysicsSoftSpace* space = new jmePhysicsSoftSpace(env, object);
        if (space == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The physics space has not been created.");
            return 0;
        }
        space->createPhysicsSoftSpace(min_vec, max_vec, broadphase, threading);
        return reinterpret_cast<jlong> (space);
    }

    /*
     * Class:     com_jme3_bullet_PhysicsSoftSpace
     * Method:    addSoftBody
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_addSoftBody
    (JNIEnv *env, jobject object, jlong spaceId, jlong softBodyId) {
        jmePhysicsSoftSpace* space = reinterpret_cast<jmePhysicsSoftSpace*> (spaceId);
        btSoftBody* collisionObject = reinterpret_cast<btSoftBody*> (softBodyId);
        if (space == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The physics space does not exist.");
            return;
        }
        if (collisionObject == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The collision object does not exist.");
            return;
        }
        jmeUserPointer *userPointer = (jmeUserPointer*) collisionObject->getUserPointer();
        userPointer->space = space;
        space->getSoftDynamicsWorld()->addSoftBody(collisionObject);
    }

    /*
     * Class:     com_jme3_bullet_PhysicsSoftSpace
     * Method:    removeSoftBody
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_removeSoftBody
    (JNIEnv *env, jobject object, jlong spaceId, jlong softBodyId) {
        jmePhysicsSoftSpace* space = reinterpret_cast<jmePhysicsSoftSpace*> (spaceId);
        btSoftBody* collisionObject = reinterpret_cast<btSoftBody*> (softBodyId);
        if (space == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The physics space does not exist.");
            return;
        }
        if (collisionObject == NULL) {
            jclass newExc = env->FindClass("java/lang/NullPointerException");
            env->ThrowNew(newExc, "The collision object does not exist.");
            return;
        }
        jmeUserPointer *userPointer = (jmeUserPointer*) collisionObject->getUserPointer();
        userPointer->space = NULL;
        space->getSoftDynamicsWorld()->removeSoftBody(collisionObject);
    }

    /*
     * Class:     com_jme3_bullet_PhysicsSoftSpace
     * Method:    getWorldInfo
     * Signature: (J)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_PhysicsSoftSpace_getWorldInfo
    (JNIEnv *env, jobject object, jlong spaceId) {
        jmePhysicsSoftSpace* space = reinterpret_cast<jmePhysicsSoftSpace*> (spaceId);
        btSoftRigidDynamicsWorld* world = space->getSoftDynamicsWorld();
        btSoftBodyWorldInfo* worldInfo = &(world->getWorldInfo());
        return reinterpret_cast<jlong> (worldInfo);
    }

#ifdef __cplusplus
}
#endif