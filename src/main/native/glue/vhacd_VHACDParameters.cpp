/*
 * Copyright (c) 2020-2023 jMonkeyEngine
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
#include "jmeClasses.h"
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
 * Method:    getAlpha
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getAlpha
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    double result = pParam->m_alpha;
    return (jdouble) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getBeta
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getBeta
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    double result = pParam->m_beta;
    return (jdouble) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getConcavity
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getConcavity
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    double result = pParam->m_concavity;
    return (jdouble) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getConvexhullApproximation
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getConvexhullApproximation
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_convexhullApproximation;
    return (jint) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getConvexhullDownsampling
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getConvexhullDownsampling
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_convexhullDownsampling;
    return (jint) result;
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
 * Method:    getMinVolumePerCH
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getMinVolumePerCH
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    double result = pParam->m_minVolumePerCH;
    return (jdouble) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getMode
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getMode
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_mode;
    return (jint) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getOclAcceleration
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getOclAcceleration
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_oclAcceleration;
    return (jint) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getPca
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_vhacd_VHACDParameters_getPca
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_pca;
    return (jboolean) result;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    getPlaneDownsampling
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getPlaneDownsampling
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_planeDownsampling;
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
 * Method:    setAlpha
 * Signature: (JD)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setAlpha
(JNIEnv *pEnv, jclass, jlong objectId, jdouble alpha) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_alpha = (double) alpha;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setBeta
 * Signature: (JD)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setBeta
(JNIEnv *pEnv, jclass, jlong objectId, jdouble beta) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_beta = (double) beta;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setConcavity
 * Signature: (JD)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setConcavity
(JNIEnv *pEnv, jclass, jlong objectId, jdouble depth) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_concavity = (double) depth;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setConvexhullApproximation
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setConvexhullApproximation
(JNIEnv *pEnv, jclass, jlong objectId, jint value) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_convexhullApproximation = (uint32_t) value;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setConvexhullDownsampling
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setConvexhullDownsampling
(JNIEnv *pEnv, jclass, jlong objectId, jint precision) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_convexhullDownsampling = (uint32_t) precision;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setMaxNumVerticesPerCH
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setMaxNumVerticesPerCH
(JNIEnv *pEnv, jclass, jlong objectId, jint numVertices) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_maxNumVerticesPerCH = (uint32_t) numVertices;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setMinVolumePerCH
 * Signature: (JD)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setMinVolumePerCH
(JNIEnv *pEnv, jclass, jlong objectId, jdouble volume) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_minVolumePerCH = (double) volume;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setMode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setMode
(JNIEnv *pEnv, jclass, jlong objectId, jint mode) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_mode = (uint32_t) mode;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setOclAcceleration
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setOclAcceleration
(JNIEnv *pEnv, jclass, jlong objectId, jint value) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_oclAcceleration = (uint32_t) value;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setPca
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setPca
(JNIEnv *pEnv, jclass, jlong objectId, jboolean enable) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_pca = (uint32_t) enable;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setPlaneDownsampling
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setPlaneDownsampling
(JNIEnv *pEnv, jclass, jlong objectId, jint granularity) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_planeDownsampling = (uint32_t) granularity;
}

/*
 * Class:     vhacd_VHACDParameters
 * Method:    setResolution
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setResolution
(JNIEnv *pEnv, jclass, jlong objectId, jint maxVoxels) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_resolution = (uint32_t) maxVoxels;
}
