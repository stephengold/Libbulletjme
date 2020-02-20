/*
 * Copyright (c) 2019-2020 jMonkeyEngine
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
#include "com_jme3_bullet_util_NativeLibrary.h"
#include "jmeClasses.h"

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    isDebug
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_NativeLibrary_isDebug
(JNIEnv *pEnv, jclass clazz) {
#ifdef _DEBUG
    return JNI_TRUE;
#else
    return JNI_FALSE;
#endif
}

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    isDoublePrecision
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_NativeLibrary_isDoublePrecision
(JNIEnv *pEnv, jclass clazz) {
#ifdef BT_USE_DOUBLE_PRECISION
    return JNI_TRUE;
#else
    return JNI_FALSE;
#endif
}

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    setStartupMessageEnabled
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeLibrary_setStartupMessageEnabled
(JNIEnv *pEnv, jclass clazz, jboolean enable) {
    jmeClasses::printFlag = (int) enable;
}
