/*
 * Copyright (c) 2020-2024 jMonkeyEngine
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
#ifndef NO_DEBUG
#include <iostream>
#endif
#include "vhacd_VHACD.h"
#include "jmeClasses.h"
#include "VHACD.h"

using namespace VHACD;

class Callback : public IVHACD::IUserCallback {
public:
    JNIEnv *pEnv;
    /*
     * constructor:
     */
    Callback(JNIEnv *pJNIEnv) {
        pEnv = pJNIEnv;
    }

    void Update(const double overallPercent, const double stagePercent,
            const double operationPercent, const char* const stageName,
            const char* const operationName) {

        jstring arg4 = pEnv->NewStringUTF(stageName);
        EXCEPTION_CHK(pEnv,);

        jstring arg5 = pEnv->NewStringUTF(operationName);
        EXCEPTION_CHK(pEnv,);

        jfloat arg1 = overallPercent;
        jfloat arg2 = stagePercent;
        jfloat arg3 = operationPercent;
        pEnv->CallStaticVoidMethod(jmeClasses::Vhacd,
                jmeClasses::Vhacd_update, arg1, arg2, arg3, arg4, arg5);
        // no check for exceptions!
    }
};

class Logger : public IVHACD::IUserLogger {
public:
    /*
     * constructor:
     */
    Logger(bool b) {
        log = b;
    }

    void Log(const char* const msg) {
#ifndef NO_DEBUG
        if (log)std::cout << msg << std::endl;
#endif
    }

private:
    bool log;
};

/*
 * Class:     vhacd_VHACD
 * Method:    compute
 * Signature: (Ljava/nio/FloatBuffer;Ljava/nio/IntBuffer;JZ)V
 */
JNIEXPORT void JNICALL Java_vhacd_VHACD_compute
(JNIEnv *pEnv, jclass, jobject positionsBuffer, jobject indicesBuffer,
        jlong paramsId, jboolean debug) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, positionsBuffer, "The positions buffer does not exist.",);
    const jfloat * const pPositions
            = (jfloat *) pEnv->GetDirectBufferAddress(positionsBuffer);
    NULL_CHK(pEnv, pPositions, "The positions buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);
    const jlong capacityFloats = pEnv->GetDirectBufferCapacity(positionsBuffer);
    EXCEPTION_CHK(pEnv,);

    NULL_CHK(pEnv, indicesBuffer, "The indices buffer does not exist.",);
    const jint * const pIndices
            = (jint *) pEnv->GetDirectBufferAddress(indicesBuffer);
    NULL_CHK(pEnv, pIndices, "The indices buffer is not direct.",);
    EXCEPTION_CHK(pEnv,);
    const jlong capacityInts = pEnv->GetDirectBufferCapacity(indicesBuffer);
    EXCEPTION_CHK(pEnv,);

    IVHACD::Parameters * const pParams
            = reinterpret_cast<IVHACD::Parameters *> (paramsId);
    NULL_CHK(pEnv, pParams, "The parameters do not exist.",)

    Callback callback = Callback(pEnv);
    pParams->m_callback = &callback;

    Logger logger(debug);
    pParams->m_logger = &logger;

    // on some platforms, jint != uint32_t
    uint32_t * const pTriangles = new uint32_t[capacityInts]; //dance001
    for (jlong i = 0; i < capacityInts; ++i) {
        pTriangles[i] = (uint32_t) pIndices[i];
    }

    IVHACD * const pIvhacd = CreateVHACD();
    const uint32_t nPoints = capacityFloats / 3;
    const uint32_t nTriangles = capacityInts / 3;
    const bool success = pIvhacd->Compute(pPositions, nPoints, pTriangles,
            nTriangles, *pParams);

    if (success) {
        const uint32_t n_hulls = pIvhacd->GetNConvexHulls();

        for (uint32_t i = 0; i < n_hulls; ++i) {
            IVHACD::ConvexHull * const
                    pHull = new IVHACD::ConvexHull(); //dance002
            pIvhacd->GetConvexHull(i, *pHull);
            const jlong hullId = reinterpret_cast<jlong> (pHull);

            pEnv->CallStaticVoidMethod(jmeClasses::Vhacd,
                    jmeClasses::Vhacd_addHull, hullId);
            EXCEPTION_CHK(pEnv,);
            delete pHull; //dance002
        }
    }

    delete[] pTriangles; //dance001
    pIvhacd->Clean();
    pIvhacd->Release();
}
