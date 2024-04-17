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
#ifndef JME_MOTION_STATE_H
#define JME_MOTION_STATE_H

/*
 * Author: Normen Hansen
 */
#include <jni.h>
#include "btBulletDynamicsCommon.h"

class jmeMotionState : public btMotionState {
private:
    bool dirty;
    btTransform * trans;
public:
    /*
     * constructor:
     */
    jmeMotionState();
    virtual ~jmeMotionState();

    btTransform worldTransform;

    bool applyTransform(JNIEnv *,
            jobject locationVector3fIn, jobject rotationQuaternionIn);
    virtual void getWorldTransform(btTransform& worldTransformOut) const;
    void setKinematicLocation(JNIEnv *, jobject locationVector3fIn);
    void setKinematicLocationDp(JNIEnv *, jobject locationVec3dIn);
    void setKinematicRotation(JNIEnv *, jobject rotationMatrix3fIn);
    void setKinematicRotationMatrix3d(JNIEnv *, jobject rotationMatrix3dIn);
    void setKinematicRotationQuat(JNIEnv *, jobject rotationQuaternionIn);
    void setKinematicRotationQuatd(JNIEnv *, jobject rotationQuatdIn);
    void setKinematicTransform(const btTransform& worldTransformIn);
    virtual void setWorldTransform(const btTransform& worldTransformIn);
};

#endif // JME_MOTION_STATE_H