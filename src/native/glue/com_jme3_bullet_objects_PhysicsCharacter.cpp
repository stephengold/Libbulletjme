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

#include "com_jme3_bullet_objects_PhysicsCharacter.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    createCharacterObject
     * Signature: (JJF)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_createCharacterObject
    (JNIEnv *env, jobject object, jlong ghostId, jlong shapeId,
            jfloat stepHeight) {
        btPairCachingGhostObject *pGhost
                = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
        NULL_CHECK(pGhost, "The btPairCachingGhostObject does not exist.", 0);
        btAssert(
                pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT);

        btCollisionShape *pShape
                = reinterpret_cast<btCollisionShape *> (shapeId);
        NULL_CHECK(pShape, "The btCollisionShape does not exist.", 0)
        if (!pShape->isConvex()) {
            jclass newExc
                    = env->FindClass("java/lang/IllegalArgumentException");
            env->ThrowNew(newExc, "The btCollisionShape isn't convex.");
            return 0;
        }
        btConvexShape *pConvex = reinterpret_cast<btConvexShape *> (shapeId);

        btKinematicCharacterController *pController
                = new btKinematicCharacterController(pGhost, pConvex,
                stepHeight);

        return reinterpret_cast<jlong> (pController);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    createGhostObject
     * Signature: ()J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_createGhostObject
    (JNIEnv *env, jobject object) {
        jmeClasses::initJavaClasses(env);
        btPairCachingGhostObject *pGhost = new btPairCachingGhostObject();
        return reinterpret_cast<jlong> (pGhost);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    finalizeNativeCharacter
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_finalizeNativeCharacter
    (JNIEnv *env, jobject object, jlong kccId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        delete pController;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getAngularDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getAngularDamping
    (JNIEnv *env, jobject object, jlong kccId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.", 0)

        return pController->getAngularDamping();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getAngularVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getAngularVelocity
    (JNIEnv *env, jobject object, jlong kccId, jobject storeVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        const btVector3& a_vel = pController->getAngularVelocity();

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &a_vel, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getFallSpeed
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getFallSpeed
    (JNIEnv *env, jobject object, jlong kccId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.", 0)

        return pController->getFallSpeed();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getGravity
     * Signature:  (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getGravity
    (JNIEnv *env, jobject object, jlong kccId, jobject storeVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        const btVector3& g = pController->getGravity();

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &g, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getJumpSpeed
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getJumpSpeed
    (JNIEnv *env, jobject object, jlong kccId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.", 0)

        return pController->getJumpSpeed();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getLinearDamping
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getLinearDamping
    (JNIEnv *env, jobject object, jlong kccId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.", 0)

        return pController->getLinearDamping();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getLinearVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getLinearVelocity
    (JNIEnv *env, jobject object, jlong kccId, jobject storeVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        const btVector3& l_vel = pController->getLinearVelocity();

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &l_vel, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getMaxPenetrationDepth
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getMaxPenetrationDepth
    (JNIEnv *env, jobject object, jlong kccId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.", 0);

        return pController->getMaxPenetrationDepth();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getMaxSlope
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getMaxSlope
    (JNIEnv *env, jobject object, jlong kccId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.", 0);

        return pController->getMaxSlope();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    getUpDirection
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getUpDirection
    (JNIEnv *env, jobject object, jlong kccId, jobject storeVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        const btVector3& direction = pController->getUp();

        NULL_CHECK(storeVector, "The store vector does not exist.",)
        jmeBulletUtil::convert(env, &direction, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    jump
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_jump
    (JNIEnv *env, jobject object, jlong kccId, jobject jumpVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);
        NULL_CHECK(jumpVector, "The jump vector does not exist.",);

        btVector3 vec;
        jmeBulletUtil::convert(env, jumpVector, &vec);

        pController->jump(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    onGround
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_onGround
    (JNIEnv *env, jobject object, jlong kccId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.", JNI_FALSE)

        return pController->onGround();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    reset
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_reset
    (JNIEnv *env, jobject object, jlong kccId, jlong spaceId) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        jmePhysicsSpace *pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
        NULL_CHECK(pSpace, "The physics space does not exist.",)

        btDynamicsWorld *pWorld = pSpace->getDynamicsWorld();
        NULL_CHECK(pWorld, "The btDynamicsWorld does not exist.",)

        pController->reset(pWorld);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setAngularDamping
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setAngularDamping
    (JNIEnv *env, jobject object, jlong kccId, jfloat value) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        pController->setAngularDamping(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setAngularVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setAngularVelocity
    (JNIEnv *env, jobject object, jlong kccId, jobject velocityVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, velocityVector, &vec);

        pController->setAngularVelocity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setCharacterFlags
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setCharacterFlags
    (JNIEnv *env, jobject object, jlong ghostId) {
        btPairCachingGhostObject *pGhost
                = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
        NULL_CHECK(pGhost, "The btPairCachingGhostObject does not exist.",)
        btAssert(
                pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT);

        pGhost->setCollisionFlags(/*pGhost->getCollisionFlags() |*/ btCollisionObject::CF_CHARACTER_OBJECT);
        pGhost->setCollisionFlags(pGhost->getCollisionFlags() & ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setFallSpeed
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setFallSpeed
    (JNIEnv *env, jobject object, jlong kccId, jfloat value) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",)

                pController->setFallSpeed(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setGravity
     * Signature:  (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setGravity
    (JNIEnv *env, jobject object, jlong kccId, jobject accelerationVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        NULL_CHECK(accelerationVector,
                "The acceleration vector does not exist.",)
                btVector3 vec;
        jmeBulletUtil::convert(env, accelerationVector, &vec);

        pController->setGravity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setJumpSpeed
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setJumpSpeed
    (JNIEnv *env, jobject object, jlong kccId, jfloat value) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        pController->setJumpSpeed(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setLinearDamping
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setLinearDamping
    (JNIEnv *env, jobject object, jlong kccId, jfloat value) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        pController->setLinearDamping(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setLinearVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setLinearVelocity
    (JNIEnv *env, jobject object, jlong kccId, jobject velocityVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, velocityVector, &vec);

        pController->setLinearVelocity(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setMaxPenetrationDepth
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setMaxPenetrationDepth
    (JNIEnv *env, jobject object, jlong kccId, jfloat value) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        pController->setMaxPenetrationDepth(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setMaxSlope
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setMaxSlope
    (JNIEnv *env, jobject object, jlong kccId, jfloat value) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        pController->setMaxSlope(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setStepHeight
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setStepHeight
    (JNIEnv *env, jobject object, jlong kccId, jfloat value) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        pController->setStepHeight(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setUp
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setUp
    (JNIEnv *env, jobject object, jlong kccId, jobject upVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        NULL_CHECK(upVector, "The vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, upVector, &vec);

        pController->setUp(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setUseGhostSweepTest
     * Signature: (JZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setUseGhostSweepTest
    (JNIEnv *env, jobject object, jlong kccId, jboolean useGhostSweepTest) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        bool flag = (bool)useGhostSweepTest;
        pController->setUseGhostSweepTest(flag);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    setWalkDirection
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setWalkDirection
    (JNIEnv *env, jobject object, jlong kccId, jobject directionVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        NULL_CHECK(directionVector, "The direction vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, directionVector, &vec);

        pController->setWalkDirection(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsCharacter
     * Method:    warp
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_warp
    (JNIEnv *env, jobject object, jlong kccId, jobject locationVector) {
        btKinematicCharacterController *pController
                = reinterpret_cast<btKinematicCharacterController *> (kccId);
        NULL_CHECK(pController,
                "The btKinematicCharacterController does not exist.",);

        NULL_CHECK(locationVector, "The location vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, locationVector, &vec);

        pController->warp(vec);
    }

#ifdef __cplusplus
}
#endif
