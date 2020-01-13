/*
 * Copyright (c) 2020 jMonkeyEngine
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

#ifdef __cplusplus
extern "C" {
#endif

    using namespace VHACD;

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    create
     * Signature: ()J
     */
    JNIEXPORT jlong JNICALL Java_vhacd_VHACDParameters_create
    (JNIEnv *env, jclass clas) {
        jmeClasses::initJavaClasses(env);

        IVHACD::Parameters * const pParam = new IVHACD::Parameters();
        return reinterpret_cast<jlong> (pParam);
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    finalizeNative
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_finalizeNative
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",)

                delete pParam;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getAlpha
     * Signature: (J)D
     */
    JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getAlpha
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        double result = pParam->m_alpha;
        return (jdouble) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getBeta
     * Signature: (J)D
     */
    JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getBeta
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        double result = pParam->m_beta;
        return (jdouble) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getConcavity
     * Signature: (J)D
     */
    JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getConcavity
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        double result = pParam->m_concavity;
        return (jdouble) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getConvexhullApproximation
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getConvexhullApproximation
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        int result = pParam->m_convexhullApproximation;
        return (jint) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getConvexhullDownsampling
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getConvexhullDownsampling
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        int result = pParam->m_convexhullDownsampling;
        return (jint) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getDepth
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getDepth
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        int result = pParam->m_depth;
        return (jint) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getGamma
     * Signature: (J)D
     */
    JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getGamma
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        double result = pParam->m_gamma;
        return (jdouble) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getMaxNumVerticesPerCH
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getMaxNumVerticesPerCH
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        unsigned int result = pParam->m_maxNumVerticesPerCH;
        return (jint) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getMinVolumePerCH
     * Signature: (J)D
     */
    JNIEXPORT jdouble JNICALL Java_vhacd_VHACDParameters_getMinVolumePerCH
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        double result = pParam->m_minVolumePerCH;
        return (jdouble) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getMode
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getMode
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        int result = pParam->m_mode;
        return (jint) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getOclAcceleration
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getOclAcceleration
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        int result = pParam->m_oclAcceleration;
        return (jint) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getPca
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_vhacd_VHACDParameters_getPca
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        int result = pParam->m_pca;
        return (jboolean) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getPlaneDownsampling
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getPlaneDownsampling
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        int result = pParam->m_planeDownsampling;
        return (jint) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    getResolution
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_vhacd_VHACDParameters_getResolution
    (JNIEnv *env, jclass clas, jlong objectId) {
        const IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.", 0);

        unsigned int result = pParam->m_resolution;
        return (jint) result;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setAlpha
     * Signature: (JD)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setAlpha
    (JNIEnv *env, jclass clas, jlong objectId, jdouble alpha) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_alpha = (double) alpha;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setBeta
     * Signature: (JD)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setBeta
    (JNIEnv *env, jclass clas, jlong objectId, jdouble beta) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_beta = (double) beta;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setConcavity
     * Signature: (JD)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setConcavity
    (JNIEnv *env, jclass clas, jlong objectId, jdouble depth) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_concavity = (double) depth;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setConvexhullApproximation
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setConvexhullApproximation
    (JNIEnv *env, jclass clas, jlong objectId, jint value) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_convexhullApproximation = (int) value;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setConvexhullDownsampling
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setConvexhullDownsampling
    (JNIEnv *env, jclass clas, jlong objectId, jint precision) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_convexhullDownsampling = (int) precision;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setDepth
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setDepth
    (JNIEnv *env, jclass clas, jlong objectId, jint depth) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_depth = (int) depth;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setGamma
     * Signature: (JD)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setGamma
    (JNIEnv *env, jclass clas, jlong objectId, jdouble gamma) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_gamma = (double) gamma;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setMaxNumVerticesPerCH
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setMaxNumVerticesPerCH
    (JNIEnv *env, jclass clas, jlong objectId, jint numVertices) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_maxNumVerticesPerCH = (unsigned int) numVertices;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setMinVolumePerCH
     * Signature: (JD)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setMinVolumePerCH
    (JNIEnv *env, jclass clas, jlong objectId, jdouble volume) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_minVolumePerCH = (double) volume;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setMode
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setMode
    (JNIEnv *env, jclass clas, jlong objectId, jint mode) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_mode = (int) mode;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setOclAcceleration
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setOclAcceleration
    (JNIEnv *env, jclass clas, jlong objectId, jint value) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_oclAcceleration = (int) value;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setPca
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setPca
    (JNIEnv *env, jclass clas, jlong objectId, jboolean enable) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_pca = (int) enable;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setPlaneDownsampling
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setPlaneDownsampling
    (JNIEnv *env, jclass clas, jlong objectId, jint granularity) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_planeDownsampling = (int) granularity;
    }

    /*
     * Class:     vhacd_VHACDParameters
     * Method:    setResolution
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_vhacd_VHACDParameters_setResolution
    (JNIEnv *env, jclass clas, jlong objectId, jint maxVoxels) {
        IVHACD::Parameters * const pParam
                = reinterpret_cast<IVHACD::Parameters *> (objectId);
        NULL_CHECK(pParam, "The parameters do not exist.",);

        pParam->m_resolution = (unsigned int) maxVoxels;
    }

#ifdef __cplusplus
}
#endif

