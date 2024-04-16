/*
 * Copyright (c) 2024 jMonkeyEngine
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
#include "com_jme3_bullet_CollisionConfiguration.h"
#include "jmeClasses.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

/*
 * Author: Stephen Gold
 */

/*
 * Class:     com_jme3_bullet_CollisionConfiguration
 * Method:    createNative
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_CollisionConfiguration_createNative
(JNIEnv *pEnv, jclass) {
    jmeClasses::initJavaClasses(pEnv);

    btCollisionConfiguration *pConfig
            = new btSoftBodyRigidBodyCollisionConfiguration(); //dance010
    return reinterpret_cast<jlong> (pConfig);
}

/*
 * Class:     com_jme3_bullet_CollisionConfiguration
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_CollisionConfiguration_finalizeNative
(JNIEnv *pEnv, jclass, jlong configId) {
    btCollisionConfiguration * const
            pConfig = reinterpret_cast<btCollisionConfiguration *> (configId);
    NULL_CHK(pEnv, pConfig, "The btCollisionConfiguration does not exist.",);

    delete pConfig; //dance010
}