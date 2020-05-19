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

/*
 * Extend btKinematicCharacterController to provide getters for m_walkDirection
 * and m_useGhostObjectSweepTest.
 */
ATTRIBUTE_ALIGNED16(class)
jmeKcc : public btKinematicCharacterController {
    public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    jmeKcc(btPairCachingGhostObject *ghost, btConvexShape *shape,
            btScalar stepHeight)
            : btKinematicCharacterController(ghost, shape, stepHeight) {
    }

    const btVector3 & getWalkOffset() const {
        return m_walkDirection;
    }

    bool isUsingGhostSweepTest() const {
        return m_useGhostObjectSweepTest;
    }
};

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    createCharacterObject
 * Signature: (JJF)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_createCharacterObject
(JNIEnv *pEnv, jobject, jlong ghostId, jlong shapeId, jfloat stepHeight) {
    btPairCachingGhostObject * const pGhost
            = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.", 0);
    btAssert(pGhost->getInternalType() & btCollisionObject::CO_GHOST_OBJECT);

    btCollisionShape * const
            pShape = reinterpret_cast<btCollisionShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The btCollisionShape does not exist.", 0)
    if (!pShape->isConvex()) {
        pEnv->ThrowNew(jmeClasses::IllegalArgumentException,
                "The btCollisionShape isn't convex.");
        return 0;
    }
    btConvexShape * const
            pConvex = reinterpret_cast<btConvexShape *> (shapeId);

    jmeKcc * const
            pController = new jmeKcc(pGhost, pConvex, stepHeight); //dance031
    return reinterpret_cast<jlong> (pController);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    createGhostObject
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_createGhostObject
(JNIEnv *pEnv, jobject) {
    jmeClasses::initJavaClasses(pEnv);

    btPairCachingGhostObject * const
            pGhost = new btPairCachingGhostObject(); //dance014
    return reinterpret_cast<jlong> (pGhost);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    finalizeNativeCharacter
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_finalizeNativeCharacter
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    delete pController; //dance031
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getAngularDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getAngularDamping
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0)

    return pController->getAngularDamping();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getAngularVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getAngularVelocity
(JNIEnv *pEnv, jobject, jlong kccId, jobject storeVector) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& a_vel = pController->getAngularVelocity();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &a_vel, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getFallSpeed
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getFallSpeed
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0)

    return pController->getFallSpeed();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getGravity
 * Signature:  (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getGravity
(JNIEnv *pEnv, jobject, jlong kccId, jobject storeVector) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& g = pController->getGravity();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &g, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getJumpSpeed
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getJumpSpeed
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0)

    return pController->getJumpSpeed();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getLinearDamping
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getLinearDamping
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0)

    return pController->getLinearDamping();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getLinearVelocity
(JNIEnv *pEnv, jobject, jlong kccId, jobject storeVector) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& l_vel = pController->getLinearVelocity();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &l_vel, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getMaxPenetrationDepth
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getMaxPenetrationDepth
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getMaxPenetrationDepth();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getMaxSlope
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getMaxSlope
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getMaxSlope();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getStepHeight
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getStepHeight
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", 0);

    return pController->getStepHeight();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getUpDirection
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getUpDirection
(JNIEnv *pEnv, jobject, jlong kccId, jobject storeVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& direction = pController->getUp();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &direction, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    getWalkOffset
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_getWalkOffset
(JNIEnv *pEnv, jobject, jlong kccId, jobject storeVector) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    const btVector3& offset = pController->getWalkOffset();

    NULL_CHK(pEnv, storeVector, "The store vector does not exist.",)
    jmeBulletUtil::convert(pEnv, &offset, storeVector);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    isUsingGhostSweepTest
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_isUsingGhostSweepTest
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", JNI_FALSE);

    bool result = pController->isUsingGhostSweepTest();

    return (jboolean) result;
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    jump
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_jump
(JNIEnv *pEnv, jobject, jlong kccId, jobject jumpVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);
    NULL_CHK(pEnv, jumpVector, "The jump vector does not exist.",);

    btVector3 vec;
    jmeBulletUtil::convert(pEnv, jumpVector, &vec);

    pController->jump(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    onGround
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_onGround
(JNIEnv *pEnv, jobject, jlong kccId) {
    const jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.", JNI_FALSE)

    return pController->onGround();
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    reset
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_reset
(JNIEnv *pEnv, jobject, jlong kccId, jlong spaceId) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    jmePhysicsSpace * const pSpace
            = reinterpret_cast<jmePhysicsSpace *> (spaceId);
    NULL_CHK(pEnv, pSpace, "The physics space does not exist.",)

    btDynamicsWorld * const pWorld = pSpace->getDynamicsWorld();
    NULL_CHK(pEnv, pWorld, "The btDynamicsWorld does not exist.",)

    pController->reset(pWorld);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setAngularDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setAngularDamping
(JNIEnv *pEnv, jobject, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setAngularDamping(value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setAngularVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setAngularVelocity
(JNIEnv *pEnv, jobject, jlong kccId, jobject velocityVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, velocityVector, &vec);

    pController->setAngularVelocity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setCharacterFlags
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setCharacterFlags
(JNIEnv *pEnv, jobject, jlong ghostId) {
    btPairCachingGhostObject * const pGhost
            = reinterpret_cast<btPairCachingGhostObject *> (ghostId);
    NULL_CHK(pEnv, pGhost, "The btPairCachingGhostObject does not exist.",)
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
(JNIEnv *pEnv, jobject, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setFallSpeed(value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setGravity
 * Signature:  (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setGravity
(JNIEnv *pEnv, jobject, jlong kccId, jobject accelerationVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, accelerationVector,
            "The acceleration vector does not exist.",);
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, accelerationVector, &vec);

    pController->setGravity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setJumpSpeed
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setJumpSpeed
(JNIEnv *pEnv, jobject, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setJumpSpeed(value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setLinearDamping
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setLinearDamping
(JNIEnv *pEnv, jobject, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setLinearDamping(value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setLinearVelocity
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setLinearVelocity
(JNIEnv *pEnv, jobject, jlong kccId, jobject velocityVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, velocityVector, "The velocity vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, velocityVector, &vec);

    pController->setLinearVelocity(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setMaxPenetrationDepth
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setMaxPenetrationDepth
(JNIEnv *pEnv, jobject, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setMaxPenetrationDepth(value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setMaxSlope
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setMaxSlope
(JNIEnv *pEnv, jobject, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setMaxSlope(value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setStepHeight
 * Signature: (JF)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setStepHeight
(JNIEnv *pEnv, jobject, jlong kccId, jfloat value) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    pController->setStepHeight(value);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setUp
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setUp
(JNIEnv *pEnv, jobject, jlong kccId, jobject upVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, upVector, "The vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, upVector, &vec);

    pController->setUp(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setUseGhostSweepTest
 * Signature: (JZ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setUseGhostSweepTest
(JNIEnv *pEnv, jobject, jlong kccId, jboolean useGhostSweepTest) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    bool flag = (bool)useGhostSweepTest;
    pController->setUseGhostSweepTest(flag);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    setWalkDirection
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_setWalkDirection
(JNIEnv *pEnv, jobject, jlong kccId, jobject directionVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, directionVector, "The direction vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, directionVector, &vec);

    pController->setWalkDirection(vec);
}

/*
 * Class:     com_jme3_bullet_objects_PhysicsCharacter
 * Method:    warp
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsCharacter_warp
(JNIEnv *pEnv, jobject, jlong kccId, jobject locationVector) {
    jmeKcc * const pController = reinterpret_cast<jmeKcc *> (kccId);
    NULL_CHK(pEnv, pController, "The controller does not exist.",);

    NULL_CHK(pEnv, locationVector, "The location vector does not exist.",)
    btVector3 vec;
    jmeBulletUtil::convert(pEnv, locationVector, &vec);

    pController->warp(vec);
}
