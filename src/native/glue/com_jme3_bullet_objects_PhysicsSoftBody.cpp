/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
 * Author: Dokthar
 */
#include "com_jme3_bullet_objects_PhysicsSoftBody.h"
#include "jmeBulletUtil.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

static inline btVector3 getBoundingCenter(btSoftBody* body) {
    return (body->m_bounds[0] + body->m_bounds[1]) / 2;
}

#ifdef __cplusplus
extern "C" {
#endif

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addAeroForceToNode
     * Signature: (JLcom/jme3/math/Vector3f;I)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addAeroForceToNode
    (JNIEnv *env, jobject object, jlong bodyId, jobject force, jint nodeId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(force, "The force vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, force, &vec);

        body->addAeroForceToNode(vec, nodeId);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addForce
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addForce__JLcom_jme3_math_Vector3f_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject force) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(force, "The force vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, force, &vec);

        body->addForce(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addForce
     * Signature: (JLcom/jme3/math/Vector3f;I)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addForce__JLcom_jme3_math_Vector3f_2I
    (JNIEnv *env, jobject object, jlong bodyId, jobject force, jint nodeId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(force, "The force vector does not exist.",)
        btVector3 vec;
        jmeBulletUtil::convert(env, force, &vec);

        body->addForce(vec, nodeId);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addVelocity__JLcom_jme3_math_Vector3f_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btVector3 bulletVector;
        jmeBulletUtil::convert(env, velocityVector, &bulletVector);

        body->addVelocity(bulletVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    addVelocity
     * Signature: (JLcom/jme3/math/Vector3f;I)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_addVelocity__JLcom_jme3_math_Vector3f_2I
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector,
            jint nodeId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btVector3 bulletVector;
        jmeBulletUtil::convert(env, velocityVector, &bulletVector);

        body->addVelocity(bulletVector, nodeId);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendAnchor
     * Signature: (JIJLcom/jme3/math/Vector3f;ZF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendAnchor
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeId, jlong rigidId,
            jobject localPivot, jboolean collisionBetweenLinkedBodies,
            jfloat influence) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btRigidBody* rigid = reinterpret_cast<btRigidBody*> (rigidId);
        NULL_CHECK(rigid, "The btRigidBody does not exist.",)

        if (localPivot != NULL) {
            btVector3 vec;
            jmeBulletUtil::convert(env, localPivot, &vec);
            body->appendAnchor(nodeId, rigid, vec,
                    !collisionBetweenLinkedBodies, influence);
        } else {
            body->appendAnchor(nodeId, rigid,
                    !collisionBetweenLinkedBodies, influence);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendFaces
     * Signature: (JLjava/nio/ByteBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendFaces__JLjava_nio_ByteBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject byteBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jbyte* face = (jbyte*) env->GetDirectBufferAddress(byteBuffer);
        const int capacity = env->GetDirectBufferCapacity(byteBuffer) - 2;

        for (int i = 0; i < capacity;) {
            int ni1 = face[i++];
            int ni2 = face[i++];
            int ni3 = face[i++];
            body->appendFace(ni1, ni2, ni3);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendFaces
     * Signature: (JLjava/nio/ShortBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendFaces__JLjava_nio_ShortBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject shortBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jshort* face = (jshort*) env->GetDirectBufferAddress(shortBuffer);
        const int capacity = env->GetDirectBufferCapacity(shortBuffer) - 2;

        for (int i = 0; i < capacity;) {
            int ni1 = face[i++];
            int ni2 = face[i++];
            int ni3 = face[i++];
            body->appendFace(ni1, ni2, ni3);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendFaces
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendFaces__JLjava_nio_IntBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jint* face = (jint*) env->GetDirectBufferAddress(intBuffer);
        const int capacity = env->GetDirectBufferCapacity(intBuffer) - 2;

        for (int i = 0; i < capacity;) {
            int ni1 = face[i++];
            int ni2 = face[i++];
            int ni3 = face[i++];
            body->appendFace(ni1, ni2, ni3);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendLinks
     * Signature: (JLjava/nio/ByteBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendLinks__JLjava_nio_ByteBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject byteBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jbyte* link = (jbyte*) env->GetDirectBufferAddress(byteBuffer);
        const int capacity = env->GetDirectBufferCapacity(byteBuffer) - 1;

        for (int i = 0; i < capacity;) {
            int ni1 = link[i++];
            int ni2 = link[i++];
            body->appendLink(ni1, ni2);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendLinks
     * Signature: (JLjava/nio/ShortBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendLinks__JLjava_nio_ShortBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject shortBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jshort* link = (jshort*) env->GetDirectBufferAddress(shortBuffer);
        const int capacity = env->GetDirectBufferCapacity(shortBuffer) - 1;

        for (int i = 0; i < capacity;) {
            int ni1 = link[i++];
            int ni2 = link[i++];
            body->appendLink(ni1, ni2);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendLinks
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendLinks__JLjava_nio_IntBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jint* link = (jint*) env->GetDirectBufferAddress(intBuffer);
        const int capacity = env->GetDirectBufferCapacity(intBuffer) - 1;

        for (int i = 0; i < capacity;) {
            int ni1 = link[i++];
            int ni2 = link[i++];
            body->appendLink(ni1, ni2);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendNodes
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendNodes
    (JNIEnv *env, jobject object, jlong bodyId, jobject floatBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jfloat* position = (jfloat*) env->GetDirectBufferAddress(floatBuffer);
        const int capacity = env->GetDirectBufferCapacity(floatBuffer) - 2;

        for (int i = 0; i < capacity;) {
            const float x = position[i++];
            const float y = position[i++];
            const float z = position[i++];
            body->appendNode(btVector3(x, y, z), 1);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendTetras
     * Signature: (JLjava/nio/ByteBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendTetras__JLjava_nio_ByteBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject byteBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jbyte* tetra = (jbyte*) env->GetDirectBufferAddress(byteBuffer);
        const int capacity = env->GetDirectBufferCapacity(byteBuffer) - 3;

        for (int i = 0; i < capacity;) {
            int ni1 = tetra[i++];
            int ni2 = tetra[i++];
            int ni3 = tetra[i++];
            int ni4 = tetra[i++];
            body->appendTetra(ni1, ni2, ni3, ni4);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendTetras
     * Signature: (JLjava/nio/ShortBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendTetras__JLjava_nio_ShortBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject shortBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jshort* tetra = (jshort*) env->GetDirectBufferAddress(shortBuffer);
        const int capacity = env->GetDirectBufferCapacity(shortBuffer) - 3;

        for (int i = 0; i < capacity;) {
            int ni1 = tetra[i++];
            int ni2 = tetra[i++];
            int ni3 = tetra[i++];
            int ni4 = tetra[i++];
            body->appendTetra(ni1, ni2, ni3, ni4);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    appendTetras
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_appendTetras__JLjava_nio_IntBuffer_2
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jint* tetra = (jint*) env->GetDirectBufferAddress(intBuffer);
        const int capacity = env->GetDirectBufferCapacity(intBuffer) - 3;

        for (int i = 0; i < capacity;) {
            int ni1 = tetra[i++];
            int ni2 = tetra[i++];
            int ni3 = tetra[i++];
            int ni4 = tetra[i++];
            body->appendTetra(ni1, ni2, ni3, ni4);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    applyPhysicsRotation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsRotation
    (JNIEnv *env, jobject object, jlong bodyId, jobject rotation) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btQuaternion rot = btQuaternion();
        jmeBulletUtil::convert(env, rotation, &rot);
        body->rotate(rot);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    applyPhysicsScale
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsScale
    (JNIEnv *env, jobject object, jlong bodyId, jobject scl) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btVector3 vec;
        jmeBulletUtil::convert(env, scl, &vec);
        body->scale(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    applyPhysicsTransform
     * Signature: (JLcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsTransform
    (JNIEnv *env, jobject object, jlong bodyId, jobject transform) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(transform, "The transform does not exist.",)
        btTransform trs;
        jmeBulletUtil::convert(env, transform, &trs);

        body->transform(trs);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    applyPhysicsTranslate
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_applyPhysicsTranslate
    (JNIEnv *env, jobject object, jlong bodyId, jobject translate) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btVector3 vec;
        jmeBulletUtil::convert(env, translate, &vec);
        body->translate(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    createEmptySoftBody
     * Signature: ()J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_createEmptySoftBody
    (JNIEnv *env, jobject object) {
        jmeClasses::initJavaClasses(env);

        // Create a temporary btSoftBodyWorldInfo for this body,
        // just until it is added to physics space. TODO null?
        btSoftBodyWorldInfo* worldInfo = new btSoftBodyWorldInfo();

        btSoftBody* body = new btSoftBody(worldInfo);

        body->getCollisionShape()->setMargin(0); // TODO bad idea?

        /* Default material */
        btSoftBody::Material* m = body->appendMaterial();
        m->m_kLST = 1;
        m->m_kAST = 1;
        m->m_kVST = 1;
        // The only available flag for Materials is DebugDraw (by Default)
        // we don't want to use Bullet's debug draw, (we use JME instead).
        m->m_flags = 0x0000;

        body->setUserPointer(NULL);
        return reinterpret_cast<jlong> (body);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    cutLink
     * Signature: (JIIF)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_cutLink
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeIndex0,
            jint nodeIndex1, jfloat position) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0);

        bool success = body->cutLink((int) nodeIndex0, (int) nodeIndex1,
                (btScalar) position);
        return (jboolean) success;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    generateBendingConstraints
     * Signature: (JIJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_generateBendingConstraints
    (JNIEnv *env, jobject object, jlong bodyId, jint dist, jlong matId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btSoftBody::Material* mat = reinterpret_cast<btSoftBody::Material*> (matId);
        NULL_CHECK(mat, "The material does not exist.",)

        body->generateBendingConstraints(dist, mat);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    generateClusters
     * Signature: (JII)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_generateClusters
    (JNIEnv *env, jobject object, jlong bodyId, jint k, jint maxIter) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->generateClusters(k, maxIter);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getAnchorCount
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getAnchorCount
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0);

        int count = body->m_anchors.size();
        return (jint) count;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getAnchorInfluence
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getAnchorInfluence
    (JNIEnv *env, jobject object, jlong bodyId, jint anchorIndex) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        btScalar influence = body->m_anchors[anchorIndex].m_influence;
        return (jfloat) influence;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getAnchorNodeIndex
     * Signature: (JI)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getAnchorNodeIndex
    (JNIEnv *env, jobject object, jlong bodyId, jint anchorIndex) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0);

        const btSoftBody::Node* node = body->m_anchors[anchorIndex].m_node;
        int nodeIndex = int(node - &body->m_nodes[0]);

        return (jint) nodeIndex;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getAnchorPivot
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getAnchorPivot
    (JNIEnv *env, jobject object, jlong bodyId, jint anchorIndex, jobject storeVector) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const btVector3* pPivot = &body->m_anchors[anchorIndex].m_local;
        jmeBulletUtil::convert(env, pPivot, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getAnchorRigidId
     * Signature: (JI)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getAnchorRigidId
    (JNIEnv *env, jobject object, jlong bodyId, jint anchorIndex) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0);

        btRigidBody* pRigid = body->m_anchors[anchorIndex].m_body;
        return reinterpret_cast<jlong> (pRigid);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getBounds
     * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getBounds
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeMinima,
            jobject storeMaxima) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const btVector3& minima = body->m_bounds[0];
        jmeBulletUtil::convert(env, &minima, storeMinima);

        const btVector3& maxima = body->m_bounds[1];
        jmeBulletUtil::convert(env, &maxima, storeMaxima);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterCenter
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterCenter
    (JNIEnv *env, jobject object, jlong bodyId, jint clusterIndex,
            jobject storeVector) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",);
        const btSoftBody::Cluster* cluster = body->m_clusters[clusterIndex];
        jmeBulletUtil::convert(env, &cluster->m_com, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClusterCount
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClusterCount
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->clusterCount();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getClustersPositions
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getClustersPositions
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(storeBuffer, "The store buffer does not exist.",);
        jfloat* bufPtr = (jfloat*) env->GetDirectBufferAddress(storeBuffer);
        int numClusters = body->m_clusters.size();
        for (int clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex) {
            const btSoftBody::Cluster* cluster = body->m_clusters[clusterIndex];
            bufPtr[0] = cluster->m_com.getX();
            bufPtr[1] = cluster->m_com.getY();
            bufPtr[2] = cluster->m_com.getZ();
            bufPtr += 3;
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getFacesIndexes
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getFacesIndexes
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        jint* out = (jint*) env->GetDirectBufferAddress(intBuffer);
        const int size = body->m_faces.size();

        btSoftBody::Node* firstNode = &body->m_nodes[0];

        for (int i = 0, buffi = 0; i < size; ++i) {
            const btSoftBody::Face& f = body->m_faces[i];
            out[buffi++] = int(f.m_n[0] - firstNode);
            out[buffi++] = int(f.m_n[1] - firstNode);
            out[buffi++] = int(f.m_n[2] - firstNode);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getLinksIndexes
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getLinksIndexes
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        jint* out = (jint*) env->GetDirectBufferAddress(intBuffer);
        const int size = body->m_links.size();

        btSoftBody::Node* firstNode = &body->m_nodes[0];

        for (int i = 0, buffi = 0; i < size; ++i) {
            const btSoftBody::Link& l = body->m_links[i];
            out[buffi++] = int(l.m_n[0] - firstNode);
            out[buffi++] = int(l.m_n[1] - firstNode);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getMargin
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getMargin
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0);

        const btCollisionShape* shape = body->getCollisionShape();
        btScalar margin = shape->getMargin();
        return (jfloat) margin;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getMass
     * Signature: (JI)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getMass
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->getMass(nodeId);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getMasses
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getMasses
    (JNIEnv *env, jobject object, jlong bodyId, jobject massBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        jfloat* masses = (jfloat*) env->GetDirectBufferAddress(massBuffer);
        const int length = env->GetDirectBufferCapacity(massBuffer);
        for (int i = 0; i < length; ++i) {
            masses[i] = body->getMass(i);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getMaterial
     * Signature: (J)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getMaterial
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return reinterpret_cast<long> (body->m_materials[0]);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNbFaces
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbFaces
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->m_faces.size();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNbLinks
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbLinks
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->m_links.size();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNbNodes
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbNodes
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->m_nodes.size();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNbTetras
     * Signature: (J)I
     */
    JNIEXPORT jint JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNbTetras
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->m_tetras.size();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodeLocation
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodeLocation
    (JNIEnv * env, jobject object, jlong bodyId, jint nodeIndex,
            jobject storeVector) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",);
        const btSoftBody::Node& node = body->m_nodes[nodeIndex];
        jmeBulletUtil::convert(env, &node.m_x, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodeNormal
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodeNormal
    (JNIEnv * env, jobject object, jlong bodyId, jint nodeIndex,
            jobject storeVector) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",);
        const btSoftBody::Node& node = body->m_nodes[nodeIndex];
        jmeBulletUtil::convert(env, &node.m_n, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodesNormals
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodesNormals
    (JNIEnv * env, jobject object, jlong bodyId, jobject storeBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(storeBuffer, "The store buffer does not exist.",);
        jfloat* bufPtr = (jfloat*) env->GetDirectBufferAddress(storeBuffer);
        int numNodes = body->m_nodes.size();
        for (int nodeIndex = 0; nodeIndex < numNodes; ++nodeIndex) {
            const btSoftBody::Node& node = body->m_nodes[nodeIndex];
            bufPtr[0] = node.m_n.getX();
            bufPtr[1] = node.m_n.getY();
            bufPtr[2] = node.m_n.getZ();
            bufPtr += 3;
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodesPositions
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodesPositions
    (JNIEnv *env, jobject object, jlong bodyId, jobject floatBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        jfloat* out = (jfloat*) env->GetDirectBufferAddress(floatBuffer);
        const int size = body->m_nodes.size();

        btVector3 temp_vertex;

        for (int i = 0, buffi = 0; i < size; i++) {
            const btSoftBody::Node& n = body->m_nodes[i];
            out[buffi++] = n.m_x.getX();
            out[buffi++] = n.m_x.getY();
            out[buffi++] = n.m_x.getZ();
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodesVelocities
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodesVelocities
    (JNIEnv *env, jobject object, jlong bodyId, jobject storeBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(storeBuffer, "The store buffer does not exist.",);
        jfloat* bufPtr = (jfloat*) env->GetDirectBufferAddress(storeBuffer);
        int numNodes = body->m_nodes.size();
        for (int nodeIndex = 0; nodeIndex < numNodes; ++nodeIndex) {
            const btSoftBody::Node& node = body->m_nodes[nodeIndex];
            bufPtr[0] = node.m_v.getX();
            bufPtr[1] = node.m_v.getY();
            bufPtr[2] = node.m_v.getZ();
            bufPtr += 3;
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getNodeVelocity
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getNodeVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeIndex,
            jobject storeVector) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(storeVector, "The store vector does not exist.",);
        const btSoftBody::Node& node = body->m_nodes[nodeIndex];
        jmeBulletUtil::convert(env, &node.m_v, storeVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getPhysicsLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getPhysicsLocation
    (JNIEnv *env, jobject object, jlong bodyId, jobject location) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btVector3 vec = getBoundingCenter(body);
        jmeBulletUtil::convert(env, &vec, location);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getPhysicsRotation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getPhysicsRotation
    (JNIEnv *env, jobject object, jlong bodyId, jobject rotation) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        jmeBulletUtil::convertQuat(env, &body->m_initialWorldTransform.getBasis(), rotation);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getPhysicsTransform
     * Signature: (JLcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getPhysicsTransform
    (JNIEnv *env, jobject object, jlong bodyId, jobject transform) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        // scale unavailable with btTransform
        jmeBulletUtil::convert(env, &body->m_initialWorldTransform, transform);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getRestLengthScale
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getRestLengthScale
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->getRestLengthScale();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getSoftBodyWorldInfo
     * Signature: (J)J
     */
    JNIEXPORT jlong JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getSoftBodyWorldInfo
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return reinterpret_cast<jlong> (body->getWorldInfo());
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getTetrasIndexes
     * Signature: (JLjava/nio/IntBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getTetrasIndexes
    (JNIEnv *env, jobject object, jlong bodyId, jobject intBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        jint* out = (jint*) env->GetDirectBufferAddress(intBuffer);
        const int size = body->m_tetras.size();

        btSoftBody::Node* firstNode = &body->m_nodes[0];

        for (int i = 0, buffi = 0; i < size; ++i) {
            const btSoftBody::Tetra& t = body->m_tetras[i];
            out[buffi++] = int(t.m_n[0] - firstNode);
            out[buffi++] = int(t.m_n[1] - firstNode);
            out[buffi++] = int(t.m_n[2] - firstNode);
            out[buffi++] = int(t.m_n[3] - firstNode);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getTotalMass
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getTotalMass
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->getTotalMass();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    getVolume
     * Signature: (J)F
     */
    JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_getVolume
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", 0)

        return body->getVolume();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    initDefault
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_initDefault
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->initDefaults();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    isCollisionAllowed
     * Signature: (JJ)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_isCollisionAllowed
    (JNIEnv *env, jobject object, jlong softId, jlong rigidId) {
        btSoftBody* softBody = reinterpret_cast<btSoftBody*> (softId);
        NULL_CHECK(softBody, "The btSoftBody does not exist.", 0)

        btCollisionObject* rigidBody
                = reinterpret_cast<btCollisionObject*> (rigidId);
        NULL_CHECK(rigidBody, "The btRigidBody does not exist.", 0);

        btAlignedObjectArray<const class btCollisionObject*> cdos = softBody->m_collisionDisabledObjects;
        int cdoIndex = cdos.findLinearSearch(rigidBody);
        bool allowed = (cdoIndex == cdos.size());

        return (jboolean) allowed;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    isInWorld
     * Signature: (J)Z
     */
    JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_isInWorld
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.", false);

        //boolean isInWorld = body->getBroadphaseHandle() != 0; // from bullet RigidBody
        return body->getBroadphaseHandle() != 0;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    randomizeConstraints
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_randomizeConstraints
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->randomizeConstraints();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    releaseCluster
     * Signature: (JI)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_releaseCluster
    (JNIEnv *env, jobject object, jlong bodyId, jint index) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->releaseCluster(index);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    releaseClusters
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_releaseClusters
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->releaseClusters();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    removeAnchor
     * Signature: (JIJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_removeAnchor
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeId, jlong rigidId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btRigidBody* rigid = reinterpret_cast<btRigidBody*> (rigidId);
        NULL_CHECK(rigid, "The btRigidBody does not exist.",);

        const int size = body->m_anchors.size();
        for (int i = 0; i < size; i++) {
            if (body->m_anchors[i].m_node == &body->m_nodes[nodeId] && body->m_anchors[i].m_body == rigid) {
                // body->m_anchors.remove(body->m_anchors[i]);
                // as we can't use remove because of no operator== between Anchors
                // we do the same but we don't have to do a second search
                body->m_anchors.swap(i, size - 1);
                body->m_anchors.pop_back();

                // set to 1 when attached, 0 by default
                body->m_nodes[nodeId].m_battach = 0;
                break;
            }
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    resetLinkRestLengths
     * Signature: (J)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_resetLinkRestLengths
    (JNIEnv *env, jobject object, jlong bodyId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->resetLinkRestLengths();
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setMargin
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setMargin
    (JNIEnv *env, jobject object, jlong bodyId, jfloat margin) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        btCollisionShape* shape = body->getCollisionShape();
        shape->setMargin(margin);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setMass
     * Signature: (JIF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setMass
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeId, jfloat mass) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->setMass(nodeId, mass);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setMasses
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setMasses
    (JNIEnv *env, jobject object, jlong bodyId, jobject massBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jfloat* masses = (jfloat*) env->GetDirectBufferAddress(massBuffer);
        const int length = env->GetDirectBufferCapacity(massBuffer);
        for (int i = 0; i < length; ++i) {
            body->setMass(i, masses[i]);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setNodeVelocity
     * Signature: (JILcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setNodeVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jint nodeIndex,
            jobject velocityVector) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btSoftBody::Node& node = body->m_nodes[nodeIndex];
        jmeBulletUtil::convert(env, velocityVector, &node.m_v);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setNormals
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setNormals
    (JNIEnv *env, jobject object, jlong bodyId, jobject normalBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jfloat* normals
                = (jfloat*) env->GetDirectBufferAddress(normalBuffer);
        int capacity = env->GetDirectBufferCapacity(normalBuffer) - 2;

        for (int nodeIndex = 0, offset = 0; offset < capacity;) {
            btScalar x = normals[offset++];
            btScalar y = normals[offset++];
            btScalar z = normals[offset++];
            btSoftBody::Node& node = body->m_nodes[nodeIndex++];
            node.m_n = btVector3(x, y, z);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setPhysicsLocation
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setPhysicsLocation
    (JNIEnv *env, jobject object, jlong bodyId, jobject location) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btVector3 vec;
        jmeBulletUtil::convert(env, location, &vec);

        vec -= getBoundingCenter(body);
        body->translate(vec);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setPhysicsRotation
     * Signature: (JLcom/jme3/math/Quaternion;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setPhysicsRotation
    (JNIEnv *env, jobject object, jlong bodyId, jobject rotation) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btMatrix3x3 rot;
        jmeBulletUtil::convertQuat(env, rotation, &rot);
        rot = body->m_initialWorldTransform.inverse().getBasis() * rot;
        btQuaternion quat;
        rot.getRotation(quat);
        body->rotate(quat);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setPhysicsTransform
     * Signature: (JLcom/jme3/math/Transform;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setPhysicsTransform
    (JNIEnv *env, jobject object, jlong bodyId, jobject transform) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        // scale unavailable with btTransform
        btTransform trs;
        jmeBulletUtil::convert(env, transform, &trs);
        body->transform(body->m_initialWorldTransform.inverse() * trs);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setPose
     * Signature: (JZZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setPose
    (JNIEnv *env, jobject object, jlong bodyId, jboolean bvolume,
            jboolean bframe) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->setPose(bvolume, bframe);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setRestLengthScale
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setRestLengthScale
    (JNIEnv *env, jobject object, jlong bodyId, jfloat value) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->setRestLengthScale(value);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setSoftBodyWorldInfo
     * Signature: (JJ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setSoftBodyWorldInfo
    (JNIEnv *env, jobject object, jlong bodyId, jlong worldId) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        btSoftBodyWorldInfo* worldInfo = reinterpret_cast<btSoftBodyWorldInfo*> (worldId);
        NULL_CHECK(worldInfo, "The btSoftBodyWorldInfo does not exist.",)

        body->m_worldInfo = worldInfo;
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setTotalDensity
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setTotalDensity
    (JNIEnv *env, jobject object, jlong bodyId, jfloat density) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->setTotalDensity(density);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setTotalMass
     * Signature: (JFZ)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setTotalMass
    (JNIEnv *env, jobject object, jlong bodyId, jfloat mass,
            jboolean fromFaces) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->setTotalMass(mass, fromFaces);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setVelocities
     * Signature: (JLjava/nio/FloatBuffer;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVelocities
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityBuffer) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",);

        const jfloat* velocities
                = (jfloat*) env->GetDirectBufferAddress(velocityBuffer);
        int capacity = env->GetDirectBufferCapacity(velocityBuffer) - 2;

        for (int nodeIndex = 0, offset = 0; offset < capacity;) {
            btScalar x = velocities[offset++];
            btScalar y = velocities[offset++];
            btScalar z = velocities[offset++];
            btSoftBody::Node& node = body->m_nodes[nodeIndex++];
            node.m_v = btVector3(x, y, z);
        }
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setVelocity
     * Signature: (JLcom/jme3/math/Vector3f;)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVelocity
    (JNIEnv *env, jobject object, jlong bodyId, jobject velocityVector) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        NULL_CHECK(velocityVector, "The velocity vector does not exist.",);
        btVector3 bulletVector;
        jmeBulletUtil::convert(env, velocityVector, &bulletVector);

        body->setVelocity(bulletVector);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setVolumeDensity
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVolumeDensity
    (JNIEnv *env, jobject object, jlong bodyId, jfloat density) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->setVolumeDensity(density);
    }

    /*
     * Class:     com_jme3_bullet_objects_PhysicsSoftBody
     * Method:    setVolumeMass
     * Signature: (JF)V
     */
    JNIEXPORT void JNICALL Java_com_jme3_bullet_objects_PhysicsSoftBody_setVolumeMass
    (JNIEnv *env, jobject object, jlong bodyId, jfloat mass) {
        btSoftBody* body = reinterpret_cast<btSoftBody*> (bodyId);
        NULL_CHECK(body, "The btSoftBody does not exist.",)

        body->setVolumeMass(mass);
    }

#ifdef __cplusplus
}
#endif