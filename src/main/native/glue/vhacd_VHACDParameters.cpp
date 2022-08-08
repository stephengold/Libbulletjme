/*
 * Copyright (c) 2020-2022 jMonkeyEngine
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
 * Author: Stephen Gold
 */
#include "vhacd_VHACDParameters.h"
#include "jmeBulletUtil.h"
#include "VHACD.h"

using namespace VHACD;

/*
 * Class:     vhacd_VHACDParameters
 * Method:    create
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_vhacd_VHACDParameters_create
(JNIEnv *pEnv, jclass) {
    jmeClasses::initJavaClasses(pEnv);

    IVHACD::Parameters * const pParam = new IVHACD::Parameters(); //dance023
    return reinterpret_cast<jlong> (pParam);
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_finalizeNative
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    delete pParam; //dance023
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getMaxNumVerticesPerCH
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getMaxNumVerticesPerCH
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_maxNumVerticesPerCH;
    return (jint) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getResolution
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getResolution
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_resolution;
    return (jint) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setMaxNumVerticesPerCH
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setMaxNumVerticesPerCH
(JNIEnv *pEnv, jclass, jlong objectId, jint limit) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_maxNumVerticesPerCH = (uint32_t) limit;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setResolution
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setResolution
(JNIEnv *pEnv, jclass, jlong objectId, jint numVoxels) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_resolution = (uint32_t) numVoxels;
}
