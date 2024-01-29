/*
 * Copyright (c) 2009-2024 jMonkeyEngine
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
 *
 * Based on com_jme3_bullet_objects_PhysicsCharacter by Normen Hansen.
 */
#include "com_jme3_bullet_objects_infos_CharacterController.h"
#include "jmeBulletUtil.h"
#include "jmePhysicsSpace.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"

/*
 * Extend btKinematicCharacterController to provide getters for m_walkDirection
 * and m_useGhostObjectSweepTest.
 */
ATTRIBUTE_ALIGNED16(class)
jmeKcc : public btKinematicCharacterController {
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    /*
     * constructor:
     */
    jmeKcc(btPairCachingGhostObject *ghost, btConvexShape *shape,
            btScalar stepHeight)
    : btKinematicCharacterController(ghost, shape, stepHeight) {
    }

    const btVector3 & getWalkOffset() const {
        return m_walkDirection;
    }

    // Use the following method in place of onGround() to solve issue #18.
    bool isOnGround() const {
        return !m_wasJumping && onGround();
    }

    bool isUsingGhostSweepTest() const {
        return m_useGhostObjectSweepTest;
    }
};

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    create
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_create
(JNIEnv *pEnv, jclass, jlong pcoId) {
    btPairCachingGhostObject * const pGhost
            = reinterpret_cast<btPairCachingGhostObject *> (pcoId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.", 0);
    ASSERT_CHK(pEnv, pGhost->getInternalType()
            & btCollisionObject::CO_GHOST_OBJECT, 0);

    btCollisionShape * const pShape = pGhost->getCollisionShape();
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", 0);
    if (!pShape->isConvex()) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "The btCollisionShape isn't convex.");
        return 0;
    }
    btConvexShape * const pConvex = (btConvexShape *) pShape;

    btScalar stepHeight = 0.5;
    jmeKcc * const
            pController = new jmeKcc(pGhost, pConvex, stepHeight); //dance031
    return reinterpret_cast<jlong> (pController);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_finalizeNative
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    delete pController; //dance031
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getAngularDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getAngularDamping
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getAngularDamping();
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getAngularVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getAngularVelocity
(JNIEnv *pEnv, jclass, jlong kccId, jobject storeVector) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& a_vel = pController->getAngularVelocity();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    jmeBulletUtil::convert(pEnv, &a_vel, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getFallSpeed
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getFallSpeed
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getFallSpeed();
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getGravity
(JNIEnv *pEnv, jclass, jlong kccId, jobject storeVector) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& g = pController->getGravity();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    jmeBulletUtil::convert(pEnv, &g, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getJumpSpeed
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getJumpSpeed
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getJumpSpeed();
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getLinearDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getLinearDamping
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getLinearDamping();
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getLinearVelocity
(JNIEnv *pEnv, jclass, jlong kccId, jobject storeVector) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& l_vel = pController->getLinearVelocity();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    jmeBulletUtil::convert(pEnv, &l_vel, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getMaxPenetrationDepth
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getMaxPenetrationDepth
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getMaxPenetrationDepth();
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getMaxSlope
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getMaxSlope
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getMaxSlope();
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getStepHeight
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getStepHeight
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getStepHeight();
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getUpDirection
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getUpDirection
(JNIEnv *pEnv, jclass, jlong kccId, jobject storeVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& direction = pController->getUp();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    jmeBulletUtil::convert(pEnv, &direction, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    getWalkOffset
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_getWalkOffset
(JNIEnv *pEnv, jclass, jlong kccId, jobject storeVector) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& offset = pController->getWalkOffset();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",);
    jmeBulletUtil::convert(pEnv, &offset, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    isUsingGhostSweepTest
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_isUsingGhostSweepTest
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", JNI_FALSE);

    bool result = pController->isUsingGhostSweepTest();

    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    jump
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_jump
(JNIEnv *pEnv, jclass, jlong kccId, jobject jumpVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);
    NULL_CHK(pEnv, jumpVector, "The jump vector does not exist.",);

    btVector3 vec;
    jmeBulletUtil::convert(pEnv, jumpVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pController->jump(vec);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    onGround
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_onGround
(JNIEnv *pEnv, jclass, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", JNI_FALSE);

    return pController->isOnGround();
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    reset
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_reset
(JNIEnv *pEnv, jclass, jlong kccId, jlong spaceId) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    jmePhysicsSpace * const
            pSpace = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",);

    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The physics world does not exist.",);

    pController->reset(pWorld);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setAngularDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setAngularDamping
(JNIEnv *pEnv, jclass, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setAngularDamping(value);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setAngularVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setAngularVelocity
(JNIEnv *pEnv, jclass, jlong kccId, jobject velocityVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",);
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, velocityVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pController->setAngularVelocity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setFallSpeed
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setFallSpeed
(JNIEnv *pEnv, jclass, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setFallSpeed(value);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setGravity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setGravity
(JNIEnv *pEnv, jclass, jlong kccId, jobject accelerationVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, accelerationVector,
            "The acceleration vector does not exist.",);
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, accelerationVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pController->setGravity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setJumpSpeed
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setJumpSpeed
(JNIEnv *pEnv, jclass, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setJumpSpeed(value);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setLinearDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setLinearDamping
(JNIEnv *pEnv, jclass, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setLinearDamping(value);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setLinearVelocity
(JNIEnv *pEnv, jclass, jlong kccId, jobject velocityVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, velocityVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pController->setLinearVelocity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setMaxPenetrationDepth
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setMaxPenetrationDepth
(JNIEnv *pEnv, jclass, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setMaxPenetrationDepth(value);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setMaxSlope
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setMaxSlope
(JNIEnv *pEnv, jclass, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setMaxSlope(value);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setStepHeight
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setStepHeight
(JNIEnv *pEnv, jclass, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setStepHeight(value);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setUp
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setUp
(JNIEnv *pEnv, jclass, jlong kccId, jobject upVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, upVector, "The vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, upVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pController->setUp(vec);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setUseGhostSweepTest
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setUseGhostSweepTest
(JNIEnv *pEnv, jclass, jlong kccId, jboolean useGhostSweepTest) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    bool flag = (bool)useGhostSweepTest;
    pController->setUseGhostSweepTest(flag);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    setWalkDirection
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_setWalkDirection
(JNIEnv *pEnv, jclass, jlong kccId, jobject directionVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, directionVector, "The direction vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, directionVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pController->setWalkDirection(vec);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    warp
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_warp
(JNIEnv *pEnv, jclass, jlong kccId, jobject locationVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, locationVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pController->warp(vec);
}

/*
 * Class:     com_jme3_bullet_objects_infos_CharacterController
 * Method:    warpDp
 * Signature: (JLcom/simsilica/mathd/Vec3d;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_infos_CharacterController_warpDp
(JNIEnv *pEnv, jclass, jlong kccId, jobject locationVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convertDp(pEnv, locationVector, &vec);
    EXCEPTION_CHK(pEnv,);

    pController->warp(vec);
}
