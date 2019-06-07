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
#include "jmeClasses.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btVector3.h"

/*
 * Author: Normen Hansen
 */
class jmeBulletUtil {
public:
    // convert Bullet math to jMonkeyEngine:
    static void convert(JNIEnv*, const btMatrix3x3* in, jobject outMatrix);
    static void convert(JNIEnv*, const btQuaternion* in, jobject outQuat);
    static void convert(JNIEnv*, const btTransform* in, jobject outTransform);
    static void convert(JNIEnv*, const btVector3* in, jobject outVector);
    static void convertQuat(JNIEnv*, const btMatrix3x3* in, jobject outQuat);

    // convert jMonkeyEngine math to Bullet:
    static void convert(JNIEnv*, jobject inMatrix, btMatrix3x3* out);
    static void convert(JNIEnv*, jobject inQuat, btQuaternion* out);
    static void convert(JNIEnv*, jobject inTransform,
            btTransform* outTransform, btVector3* outScale);
    static void convert(JNIEnv*, jobject inVector, btVector3* out);
    static void convertQuat(JNIEnv*, jobject inQuat, btMatrix3x3* out);

    static void addResult(JNIEnv*, jobject resultlist, btVector3* hitnormal,
            btVector3* m_hitPointWorld, const btScalar m_hitFraction,
            const btCollisionObject* hitobject);
    static void addSweepResult(JNIEnv*v, jobject resultlist,
            btVector3* hitnormal, btVector3* m_hitPointWorld,
            const btScalar m_hitFraction, const btCollisionObject* hitobject);

private:

    jmeBulletUtil() {
    };

    ~jmeBulletUtil() {
    };
};

class jmeUserPointer {
public:
    jobject javaCollisionObject;
    jint group;
    jint groups;
    void *space;
};