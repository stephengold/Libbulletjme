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
#include "vhacd4_Vhacd4Parameters.h"
#include "jmeClasses.h"

#define VHACD VHACD4
#include "VHACD4.h"

using namespace VHACD;

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    create
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_vhacd4_Vhacd4Parameters_create
(JNIEnv *pEnv, jclass) {
    jmeClasses::initJavaClasses(pEnv);

    IVHACD::Parameters * const pParam = new IVHACD::Parameters(); //dance023
    return reinterpret_cast<jlong> (pParam);
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_finalizeNative
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    delete pParam; //dance023
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    getFillMode
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd4_Vhacd4Parameters_getFillMode
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    jint result = -1;
    switch (pParam->m_fillMode) {
        case FillMode::FLOOD_FILL:
            result = 0;
            break;
        case FillMode::SURFACE_ONLY:
            result = 1;
            break;
        case FillMode::RAYCAST_FILL:
            result = 2;
            break;
        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The fill mode is out of range.");
    }

    return result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    getMaxHulls
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd4_Vhacd4Parameters_getMaxHulls
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_maxConvexHulls;
    return (jint) result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    getMaxNumVerticesPerCH
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd4_Vhacd4Parameters_getMaxNumVerticesPerCH
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_maxNumVerticesPerCH;
    return (jint) result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    getMaxRecursion
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd4_Vhacd4Parameters_getMaxRecursion
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_maxRecursionDepth;
    return (jint) result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    getMinEdgeLength
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd4_Vhacd4Parameters_getMinEdgeLength
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_minEdgeLength;
    return (jint) result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    getResolution
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd4_Vhacd4Parameters_getResolution
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    uint32_t result = pParam->m_resolution;
    return (jint) result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    getVolumePercentError
 * Signature: (J)D
 */
JNIEXPORT jdouble JNICALL Java_vhacd4_Vhacd4Parameters_getVolumePercentError
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", 0);

    double result = pParam->m_minimumVolumePercentErrorAllowed;
    return (jdouble) result;
}


/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    isAsync
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_vhacd4_Vhacd4Parameters_isAsync
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", JNI_FALSE);

    bool result = pParam->m_asyncACD;
    return (jboolean) result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    isFindBestPlane
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_vhacd4_Vhacd4Parameters_isFindBestPlane
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", JNI_FALSE);

    bool result = pParam->m_findBestPlane;
    return (jboolean) result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    isShrinkWrap
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_vhacd4_Vhacd4Parameters_isShrinkWrap
(JNIEnv *pEnv, jclass, jlong objectId) {
    const IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.", JNI_FALSE);

    bool result = pParam->m_shrinkWrap;
    return (jboolean) result;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setAsync
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setAsync
(JNIEnv *pEnv, jclass, jlong objectId, jboolean setting) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_asyncACD = (bool) setting;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setFillMode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setFillMode
(JNIEnv *pEnv, jclass, jlong objectId, jint ordinal) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    switch (ordinal) {
        case 0:
            pParam->m_fillMode = FillMode::FLOOD_FILL;
            break;
        case 1:
            pParam->m_fillMode = FillMode::SURFACE_ONLY;
            break;
        case 2:
            pParam->m_fillMode = FillMode::RAYCAST_FILL;
            break;
        default:
            pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                    "The fill mode is out of range.");
    }
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setFindBestPlane
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setFindBestPlane
(JNIEnv *pEnv, jclass, jlong objectId, jboolean setting) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_findBestPlane = (bool) setting;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setMaxHulls
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setMaxHulls
(JNIEnv *pEnv, jclass, jlong objectId, jint limit) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_maxConvexHulls = (uint32_t) limit;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setMaxNumVerticesPerCH
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setMaxNumVerticesPerCH
(JNIEnv *pEnv, jclass, jlong objectId, jint limit) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_maxNumVerticesPerCH = (uint32_t) limit;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setMaxRecursion
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setMaxRecursion
(JNIEnv *pEnv, jclass, jlong objectId, jint depth) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_maxRecursionDepth = (uint32_t) depth;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setMinEdgeLength
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setMinEdgeLength
(JNIEnv *pEnv, jclass, jlong objectId, jint length) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_minEdgeLength = (uint32_t) length;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setResolution
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setResolution
(JNIEnv *pEnv, jclass, jlong objectId, jint numVoxels) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_resolution = (uint32_t) numVoxels;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setShrinkWrap
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setShrinkWrap
(JNIEnv *pEnv, jclass, jlong objectId, jboolean setting) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_shrinkWrap = (bool) setting;
}

/*
 * Class:     vhacd4_Vhacd4Parameters
 * Method:    setVolumePercentError
 * Signature: (JD)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Parameters_setVolumePercentError
(JNIEnv *pEnv, jclass, jlong objectId, jdouble percentage) {
    IVHACD::Parameters * const pParam
            = reinterpret_cast<IVHACD::Parameters *> (objectId);
    NULL_CHK(pEnv, pParam, "The parameters do not exist.",);

    pParam->m_minimumVolumePercentErrorAllowed = (double) percentage;
}
