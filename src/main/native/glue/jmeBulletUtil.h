/*
 * Copyright (c) 2009-2012 jMonkeyEngine
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
#ifndef JME_BULLET_UTIL_H
#define JME_BULLET_UTIL_H

/*
 * Author: Normen Hansen
 */
#include "jmeClasses.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

class jmeBulletUtil {
public:
    // convert Bullet math objects to jMonkeyEngine/SimMath:
    static void convert(JNIEnv *, const btMatrix3x3 * in, jobject outMatrix3f);
    static void convert(JNIEnv *, const btQuaternion * in, jobject outQuaternion);
    static void convert(JNIEnv *, const btTransform * in, jobject outTransform);
    static void convert(JNIEnv *, const btVector3 * in, jobject outVector3f);
    static void convertDp(JNIEnv *, const btMatrix3x3 * in, jobject outMatrix3d);
    static void convertDp(JNIEnv *, const btQuaternion * in, jobject outQuatd);
    static void convertDp(JNIEnv *, const btVector3 * in, jobject outVec3d);
    static void convertQuat(JNIEnv *, const btMatrix3x3 * in, jobject outQuaternion);
    static void convertQuatDp(JNIEnv *, const btMatrix3x3 * in, jobject outQuatd);

    // convert jMonkeyEngine/SimMath math objects to Bullet:
    static void convert(JNIEnv *, jobject inMatrix3f, btMatrix3x3 * out);
    static void convert(JNIEnv *, jobject inQuaternion, btQuaternion * out);
    static void convert(JNIEnv *, jobject inTransform,
            btTransform * outTransform, btVector3 * outScale);
    static void convert(JNIEnv *, jobject inVector3f, btVector3 * out);
    static void convertDp(JNIEnv *, jobject inMatrix3d, btMatrix3x3 * out);
    static void convertDp(JNIEnv *, jobject inQuatd, btQuaternion * out);
    static void convertDp(JNIEnv *, jobject inVec3d, btVector3 * out);
    static void convertQuat(JNIEnv *, jobject inQuaternion, btMatrix3x3 * out);
    static void convertQuatDp(JNIEnv *, jobject inQuatd, btMatrix3x3 * out);

    static void addRayTestResult(JNIEnv *, jobject resultList,
            const btVector3 *pNormal, btScalar hitFraction,
            const btCollisionObject *, int partIndex, int triangleIndex);
    static void addSweepTestResult(JNIEnv *, jobject resultList,
            const btVector3 *pNormal, btScalar hitFraction,
            const btCollisionObject *, int partIndex, int triangleIndex);

private:
    /*
     * constructor:
     */
    jmeBulletUtil() {
    }

    ~jmeBulletUtil() {
    }
};

#endif // JME_BULLET_UTIL_H
