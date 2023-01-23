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
#include "com_jme3_bullet_objects_infos_SoftBodyMaterial.h"
#include "jmeClasses.h"
#include "BulletSoftBody/btSoftBody.h"

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyMaterial
 * Method:    getAngularStiffnessFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyMaterial_getAngularStiffnessFactor
(JNIEnv *pEnv, jclass, jlong materialId) {
    const btSoftBody::Material * const
            pMaterial = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.", 0);

    return pMaterial->m_kAST;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyMaterial
 * Method:    getLinearStiffnessFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyMaterial_getLinearStiffnessFactor
(JNIEnv *pEnv, jclass, jlong materialId) {
    const btSoftBody::Material * const
            pMaterial = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.", 0);

    return pMaterial->m_kLST;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyMaterial
 * Method:    getMaterialId
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyMaterial_getMaterialId
(JNIEnv *pEnv, jclass, jlong bodyId) {
    const btSoftBody * const
            pBody = reinterpret_cast<btSoftBody *> (bodyId);
    NULL_CHK(pEnv, pBody, "The btSoftBody does not exist.", 0);
    ASSERT_CHK(pEnv, pBody->getInternalType() & btCollisionObject::CO_SOFT_BODY, 0);
    const btSoftBody::Material *pMaterial = pBody->m_materials[0];

    return reinterpret_cast<jlong> (pMaterial);
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyMaterial
 * Method:    getVolumeStiffnessFactor
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyMaterial_getVolumeStiffnessFactor
(JNIEnv *pEnv, jclass, jlong materialId) {
    const btSoftBody::Material * const
            pMaterial = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.", 0);

    return pMaterial->m_kVST;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyMaterial
 * Method:    setAngularStiffnessFactor
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyMaterial_setAngularStiffnessFactor
(JNIEnv *pEnv, jclass, jlong materialId, jfloat factor) {
    btSoftBody::Material * const
            pMaterial = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.",);

    pMaterial->m_kAST = factor;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyMaterial
 * Method:    setLinearStiffnessFactor
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyMaterial_setLinearStiffnessFactor
(JNIEnv *pEnv, jclass, jlong materialId, jfloat factor) {
    btSoftBody::Material * const
            pMaterial = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.",);

    pMaterial->m_kLST = factor;
}

/*
 * Class:     com_jme3_bullet_objects_infos_SoftBodyMaterial
 * Method:    setVolumeStiffnessFactor
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_SoftBodyMaterial_setVolumeStiffnessFactor
(JNIEnv *pEnv, jclass, jlong materialId, jfloat factor) {
    btSoftBody::Material * const
            pMaterial = reinterpret_cast<btSoftBody::Material *> (materialId);
    NULL_CHK(pEnv, pMaterial, "The material does not exist.",);

    pMaterial->m_kVST = factor;
}
