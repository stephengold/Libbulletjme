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
#include "vhacd4_Vhacd4Hull.h"
#include "jmeClasses.h"

#define VHACD VHACD4
#include "VHACD4.h"

using namespace VHACD;

/*
 * Class:     vhacd4_Vhacd4Hull
 * Method:    getNumFloats
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_vhacd4_Vhacd4Hull_getNumFloats
(JNIEnv *pEnv, jclass, jlong hullId) {
    const IVHACD::ConvexHull * const pHull
            = reinterpret_cast<IVHACD::ConvexHull *> (hullId);
    NULL_CHK(pEnv, pHull, "The hull does not exist.", 0);

    uint32_t numFloats = 3 * pHull->m_points.size();

    return (jint) numFloats;
}

/*
 * Class:     vhacd4_Vhacd4Hull
 * Method:    getPositions
 * Signature: (JLjava/nio/FloatBuffer;)V
 */
JNIEXPORT void JNICALL Java_vhacd4_Vhacd4Hull_getPositions
(JNIEnv *pEnv, jclass, jlong hullId, jobject storeBuffer) {
    const IVHACD::ConvexHull * const pHull
            = reinterpret_cast<IVHACD::ConvexHull *> (hullId);
    NULL_CHK(pEnv, pHull, "The hull does not exist.",)

    NULL_CHK(pEnv, storeBuffer, "The positions buffer does not exist.",);
    jfloat * const pPositions
            = (jfloat *) pEnv->GetDirectBufferAddress(storeBuffer);
    NULL_CHK(pEnv, pPositions, "The positions buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);

    const jlong capacity = pEnv->GetDirectBufferCapacity(storeBuffer);
    EXCEPTION_CHK(pEnv,);
    const uint32_t numPoints = pHull->m_points.size();
    for (uint32_t i = 0; i < numPoints; ++i) {
        const uint32_t bp = 3 * i;
        if (bp + 2 >= capacity)
            break;
        pPositions[bp] = pHull->m_points[i].mX;
        pPositions[bp + 1] = pHull->m_points[i].mY;
        pPositions[bp + 2] = pHull->m_points[i].mZ;
    }
}
