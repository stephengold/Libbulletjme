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

/*
 * Author: Normen Hansen
 */
#include "jmeMotionState.h"
#include "jmeBulletUtil.h"

jmeMotionState::jmeMotionState() {
    trans = new btTransform(); //dance024
    trans->setIdentity();
    worldTransform = *trans;
    dirty = true;
}

jmeMotionState::~jmeMotionState() {
    delete trans; //dance024
}

bool jmeMotionState::applyTransform(JNIEnv *pEnv,
        jobject locationVector3fIn, jobject rotationQuaternionIn) {
    if (dirty) {
        //        fprintf(stdout, "Apply world translation\n");
        //        fflush(stdout);
        jmeBulletUtil::convert(pEnv, &worldTransform.getOrigin(), locationVector3fIn);
        jmeBulletUtil::convertQuat(pEnv, &worldTransform.getBasis(), rotationQuaternionIn);
        dirty = false;
        return true;
    }
    return false;
}

void jmeMotionState::getWorldTransform(btTransform& worldTransformOut) const {
    worldTransformOut = worldTransform;
}

void jmeMotionState::setKinematicLocation(JNIEnv *pEnv, jobject locationVector3fIn) {
    jmeBulletUtil::convert(pEnv, locationVector3fIn, &worldTransform.getOrigin());
    dirty = true;
}

void jmeMotionState::setKinematicLocationDp(JNIEnv *pEnv, jobject locationVec3dIn) {
    jmeBulletUtil::convertDp(pEnv, locationVec3dIn, &worldTransform.getOrigin());
    dirty = true;
}

void jmeMotionState::setKinematicRotation(JNIEnv *pEnv, jobject rotationMatrix3fIn) {
    jmeBulletUtil::convert(pEnv, rotationMatrix3fIn, &worldTransform.getBasis());
    dirty = true;
}

void jmeMotionState::setKinematicRotationMatrix3d(JNIEnv *pEnv, jobject rotationMatrix3dIn) {
    jmeBulletUtil::convertDp(pEnv, rotationMatrix3dIn, &worldTransform.getBasis());
    dirty = true;
}

void jmeMotionState::setKinematicRotationQuat(JNIEnv *pEnv, jobject rotationQuaternionIn) {
    jmeBulletUtil::convertQuat(pEnv, rotationQuaternionIn, &worldTransform.getBasis());
    dirty = true;
}

void jmeMotionState::setKinematicRotationQuatd(JNIEnv *pEnv, jobject rotationQuatdIn) {
    btQuaternion q;
    jmeBulletUtil::convertDp(pEnv, rotationQuatdIn, &q);
    btMatrix3x3 & worldBasis = worldTransform.getBasis();
    worldBasis.setRotation(q);
    dirty = true;
}

void jmeMotionState::setKinematicTransform(const btTransform& worldTransformIn) {
    worldTransform = worldTransformIn;
    dirty = true;
}

void jmeMotionState::setWorldTransform(const btTransform& worldTransformIn) {
    worldTransform = worldTransformIn;
    dirty = true;
}