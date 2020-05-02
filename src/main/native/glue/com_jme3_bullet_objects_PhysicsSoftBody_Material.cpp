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

/*
 * Author: Dokthar
 */
#include "com_jme3_bullet_objects_PhysicsSoftBody_Material.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
 * Method:    getAngularStiffnessFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_getAngularStiffnessFactor
(JNIEnv *pEnv, jobject object, jlong materialId) {
    btSoftBody::Material *pMaterial
            = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.", 0)

    return pMaterial->m_kAST;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
 * Method:    getLinearStiffnessFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_getLinearStiffnessFactor
(JNIEnv *pEnv, jobject object, jlong materialId) {
    btSoftBody::Material *pMaterial
            = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.", 0)

    return pMaterial->m_kLST;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
 * Method:    getVolumeStiffnessFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_getVolumeStiffnessFactor
(JNIEnv *pEnv, jobject object, jlong materialId) {
    btSoftBody::Material *pMaterial
            = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.", 0)

    return pMaterial->m_kVST;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
 * Method:    setAngularStiffnessFactor
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_setAngularStiffnessFactor
(JNIEnv *pEnv, jobject object, jlong materialId, jfloat factor) {
    btSoftBody::Material *pMaterial
            = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.",)

    pMaterial->m_kAST = factor;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
 * Method:    setLinearStiffnessFactor
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_setLinearStiffnessFactor
(JNIEnv *pEnv, jobject object, jlong materialId, jfloat factor) {
    btSoftBody::Material *pMaterial
            = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.",)

    pMaterial->m_kLST = factor;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsSoftBody_Material
 * Method:    setVolumeStiffnessFactor
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_00024Material_setVolumeStiffnessFactor
(JNIEnv *pEnv, jobject object, jlong materialId, jfloat factor) {
    btSoftBody::Material *pMaterial
            = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.",)

    pMaterial->m_kVST = factor;
}
