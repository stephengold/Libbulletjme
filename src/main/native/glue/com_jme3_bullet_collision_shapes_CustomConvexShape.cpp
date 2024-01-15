/*
 * Copyright (c) 2024 jMonkeyEngine
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
#include "com_jme3_bullet_collision_shapes_CustomConvexShape.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "jmeBulletUtil.h"

/*
 * A custom btConvexInternalShape defined in terms of its supporting vertices,
 * which are calculated using a JVM method.
 */
ATTRIBUTE_ALIGNED16(class)
jmeConvexShape : public btConvexInternalShape {
public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    /*
     * constructors:
     */
    jmeConvexShape(
        JNIEnv *pEnv, jweak javaShapeRef, const btVector3& descaledInertia)
    : btConvexInternalShape() {
        m_useSlowAabb = true;

        m_descaledInertia = descaledInertia;
        m_javaShapeRef = javaShapeRef;
        m_pCreateEnv = pEnv;
        m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
    }

    jmeConvexShape(JNIEnv *pEnv, jweak javaShapeRef,
        const btVector3& descaledHalfExtents, const btVector3& descaledInertia)
    : btConvexInternalShape() {
        m_useSlowAabb = false;
        m_descaledHalfExtents = descaledHalfExtents;

        m_descaledInertia = descaledInertia;
        m_javaShapeRef = javaShapeRef;
        m_pCreateEnv = pEnv;
        m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
    }

    virtual ~jmeConvexShape() {}

    void
    batchedUnitVectorGetSupportingVertexWithoutMargin(
        const btVector3 *pLocationsIn, btVector3 *pVerticesOut, int numV) const
    {
        for (int i = 0; i < numV; i++) {
            const btVector3& testVector = pLocationsIn[i];
            pVerticesOut[i] = localGetSupportingVertexWithoutMargin(testVector);
        }
    }

    void
    calculateLocalInertia(btScalar mass, btVector3& storeResult) const {
        btVector3 scaleVector = getLocalScaling();
        btScalar x = scaleVector.x();
        btScalar y = scaleVector.y();
        btScalar z = scaleVector.z();
        btScalar xx = x * x;
        btScalar yy = y * y;
        btScalar zz = z * z;
        btVector3 factor(yy + zz, zz + xx, xx + yy);
        storeResult = m_descaledInertia * mass * factor / 2;
    }

    void
    getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
    {
        if (m_useSlowAabb) {
            getAabbSlow(t, aabbMin, aabbMax);
        } else {
            btVector3 scaleVector = getLocalScaling();
            btVector3 halfExtents = m_descaledHalfExtents * scaleVector;
            btScalar margin = getMargin();
            btTransformAabb(halfExtents, margin, t, aabbMin, aabbMax);
        }
    }

    const char *
    getName() const {
        return "CustomConvexShape";
    }

    btVector3
    localGetSupportingVertexWithoutMargin(const btVector3& testVector) const {
        btVector3 scaleVector = getLocalScaling();
        btVector3 descaled = testVector / scaleVector;

        jfloat testX = descaled.x();
        jfloat testY = descaled.y();
        jfloat testZ = descaled.z();
        jobject javaVertex = m_pCreateEnv->CallObjectMethod(
                m_javaShapeRef, jmeClasses::CustomConvexShape_locateSupport,
                testX, testY, testZ);
        // no check for exceptions!

        btVector3 result;
        jmeBulletUtil::convert(m_pCreateEnv, javaVertex, &result);
        result *= scaleVector;

        return result;
    }

private:
    /*
     * true to calculate bounding boxes using supporting vertices,
     * false to use the descaled half extents:
     */
    bool m_useSlowAabb;
    /*
     * if m_useSlowAabb is false, these are the shape's half extents
     * on each local axis, for scale=(1,1,1) and margin=0:
     */
    btVector3 m_descaledHalfExtents;
    /*
     * moment of (rotational) inertia on each local axis for scale=(1,1,1):
     */
    btVector3 m_descaledInertia;
    /*
     * interface pointer for the thread that created this collision shape:
     */
    JNIEnv *m_pCreateEnv;
    /*
     * weak reference to the corresponding Java instance:
     */
    jweak m_javaShapeRef;
};

/*
 * Class:     com_jme3_bullet_collision_shapes_CustomConvexShape
 * Method:    createShapeNative
 * Signature: (Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_CustomConvexShape_createShapeNative
(JNIEnv *pEnv, jobject javaShape, jobject heVector, jobject inertiaVector) {
    jmeClasses::initJavaClasses(pEnv);

    NULL_CHK(pEnv, inertiaVector, "The inertia vector does not exist.", 0);

    btVector3 descaledInertia;
    jmeBulletUtil::convert(pEnv, inertiaVector, &descaledInertia);
    EXCEPTION_CHK(pEnv, 0);

    jweak javaShapeRef = pEnv->NewWeakGlobalRef(javaShape); // TODO leak
    EXCEPTION_CHK(pEnv, 0);

    jmeConvexShape *pShape;
    if (heVector == NULL) {
        pShape = new jmeConvexShape(
            pEnv, javaShapeRef, descaledInertia); //dance016

    } else {
        btVector3 descaledHalfExtents;
        jmeBulletUtil::convert(pEnv, heVector, &descaledHalfExtents);
        EXCEPTION_CHK(pEnv, 0);

        pShape = new jmeConvexShape(pEnv, javaShapeRef,
            descaledHalfExtents, descaledInertia); //dance016
    }

    return reinterpret_cast<jlong> (pShape);
}
