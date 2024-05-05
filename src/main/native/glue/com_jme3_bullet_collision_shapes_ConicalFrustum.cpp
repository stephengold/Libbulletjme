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
#include "btMinMax.h"
#include "com_jme3_bullet_collision_shapes_ConicalFrustum.h"
#include "jmeBulletUtil.h"

/*
 * A collision shape for a conical frustum with uniform density.
 * By convention, the local Y axis is the height axis,
 * with the "A" base having y<0 and the "B" base having y>0.
 */
ATTRIBUTE_ALIGNED16(class)
ConicalFrustumShape : public btConvexInternalShape {
protected:
    /*
     * scaled radius of the "A" base, excluding margin (in physics-space units):
     */
    btScalar m_scaledA;
    /*
     * scaled radius of the "B" base, excluding margin (in physics-space units):
     */
    btScalar m_scaledB;
    /*
     * scaled height, excluding margin (in physics-space units):
     */
    btScalar m_scaledHeight;
    /*
     * scaled distance between the center of "A" base and the frustum's center
     * of mass (in physics-space units):
     */
    btScalar m_scaledY0;
    /*
     * radius of the "A" base, for scale=(1,1,1) and margin=0:
     */
    btScalar m_unscaledA;
    /*
     * radius of the "B" base, for scale=(1,1,1) and margin=0:
     */
    btScalar m_unscaledB;
    /*
     * height, for scale=(1,1,1) and margin=0:
     */
    btScalar m_unscaledHeight;
    /*
     * the shape's half extents on each local axis,
     * for scale=(1,1,1) and margin=0:
     */
    btVector3 m_descaledHalfExtents;
    /*
     * (rotational) inertia around each local axis, for mass=1:
     */
    btVector3 m_scaledInertia;

public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    /*
     * constructor:
     */
    ConicalFrustumShape(btScalar aRadius, btScalar bRadius, btScalar height)
    : btConvexInternalShape(),
      m_unscaledA(aRadius),
      m_unscaledB(bRadius),
      m_unscaledHeight(height)
    {
        m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;

        // Initialize the descaled half extents:
        btScalar a2 = aRadius * aRadius;
        btScalar ab = aRadius * bRadius;
        btScalar b2 = bRadius * bRadius;
        btScalar denom = a2 + ab + b2;
        /*
         * the distance between the center of "A" base
         * and the frustum's center of mass:
         */
        btScalar y0 = 0.25 * height * (a2 + 2*ab + 3*b2) / denom;

        btScalar maxR = btMax(aRadius, bRadius);
        btScalar maxY = btMax(height - y0, y0);
        m_descaledHalfExtents.setValue(maxR, maxY, maxR);

        setLocalScaling(btVector3(1, 1, 1));
    }

    virtual ~ConicalFrustumShape() {}

    void
    batchedUnitVectorGetSupportingVertexWithoutMargin(
        const btVector3 *pLocationsIn, btVector3 *pVerticesOut, int numVectors) const
    {
        for (int i = 0; i < numVectors; ++i) {
            const btVector3& testVector = pLocationsIn[i];
            pVerticesOut[i] = localGetSupportingVertexWithoutMargin(testVector);
        }
    }

    void
    calculateLocalInertia(btScalar mass, btVector3& storeResult) const {
        storeResult = mass * m_scaledInertia;
    }

    void
    getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
    {
        btVector3 scaleVector = getLocalScaling();
        btVector3 halfExtents = m_descaledHalfExtents * scaleVector;
        btScalar margin = getMargin();
        btTransformAabb(halfExtents, margin, t, aabbMin, aabbMax);
    }

    const char *
    getName() const {
        return "ConicalFrustum";
    }

    btVector3
    localGetSupportingVertexWithoutMargin(const btVector3& dir) const {
        // The supporting vertex lies on the rim of one of the bases:
        btScalar dx = dir.x();
        btScalar dz = dir.z();
        btScalar dxz = btSqrt(dx*dx + dz*dz);

        btVector3 result;
        if (m_scaledHeight * dir.y() < (m_scaledA - m_scaledB) * dxz) {
            // More precisely, the rim of the "A" base:
            if (dxz == 0) {
                result.setValue(m_scaledA, -m_scaledY0, 0);
            } else {
                result.setValue(m_scaledA * (dx / dxz),
                               -m_scaledY0,
                                m_scaledA * (dz / dxz));
            }

        } else { // It lies on the rim of the "B" base:
            if (dxz == 0) {
                result.setValue(m_scaledB, m_scaledHeight - m_scaledY0, 0);
            } else {
                result.setValue(m_scaledB * (dx / dxz),
                                m_scaledHeight - m_scaledY0,
                                m_scaledB * (dz / dxz));
            }
        }
        return result;
    }

    /*
     * Calculate how far the scaled shape extends from its center of mass,
     * including collision margin.
     */
    btScalar maxRadius() const {
        btScalar rimA = btSqrt(m_scaledA*m_scaledA + m_scaledY0*m_scaledY0);
        btScalar y1 = m_scaledHeight - m_scaledY0;
        btScalar rimB = btSqrt(m_scaledB*m_scaledB + y1*y1);
        btScalar result = btMax(rimA, rimB) + getMargin();

        return result;
    }

    void
    setLocalScaling(const btVector3& scale) {
        btConvexInternalShape::setLocalScaling(scale);

        m_scaledHeight = scale.y() * m_unscaledHeight;
        m_scaledA = scale.x() * m_unscaledA;
        m_scaledB = scale.x() * m_unscaledB;

        btScalar a2 = m_scaledA * m_scaledA;
        btScalar ab = m_scaledA * m_scaledB;
        btScalar b2 = m_scaledB * m_scaledB;
        btScalar denom = a2 + ab + b2;
        m_scaledY0 = 0.25 * m_scaledHeight * (a2 + 2*ab + 3*b2) / denom;

        btScalar a3 = a2 * m_scaledA;
        btScalar a4 = a3 * m_scaledA;
        btScalar b3 = b2 * m_scaledB;
        btScalar b4 = b3 * m_scaledB;
        btScalar h2 = m_scaledHeight * m_scaledHeight;
        btScalar mom = 3 * a3 * (m_scaledA + m_scaledB)
                + a2 * (3 * b2 + 2 * h2)
                + 3 * ab * (b2 + 2 * h2)
                + 3 * b2 * (b2 + 4 * h2);
        /*
         * the moments of inertia of a uniformly dense conical frustum
         * with mass=1, around its center of mass:
         */
        btScalar ixz = (0.05 / denom) * mom - m_scaledY0 * m_scaledY0;
        btScalar iy = (0.3 / denom)
                * (a4 + a3 * m_scaledB + a2 * b2 + m_scaledA * b3 + b4);
        m_scaledInertia.setValue(ixz, iy, ixz);
    }

    /*
     * Estimate the volume of the collision shape, including scale and margin.
     */
    btScalar volume() const {
        btScalar a = m_scaledA + getMargin();
        btScalar b = m_scaledB + getMargin();
        btScalar h = m_scaledHeight + 2 * getMargin();
        btScalar denom = a*a + a*b + b*b;
        btScalar result = SIMD_PI * h * denom/3;
        return result;
    }
};

/*
 * Class:     com_jme3_bullet_collision_shapes_ConicalFrustum
 * Method:    createShapeNative
 * Signature: (FFF)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_ConicalFrustum_createShapeNative
(JNIEnv *pEnv, jclass, jfloat aRadius, jfloat bRadius, jfloat height) {
    jmeClasses::initJavaClasses(pEnv);
    ConicalFrustumShape *pShape
            = new ConicalFrustumShape(aRadius, bRadius, height); //dance016
    return reinterpret_cast<jlong> (pShape);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_ConicalFrustum
 * Method:    maxRadius
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_shapes_ConicalFrustum_maxRadius
(JNIEnv *pEnv, jclass, jlong shapeId) {
    ConicalFrustumShape *pShape
            = reinterpret_cast<ConicalFrustumShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The ConicalFrustumShape does not exist.", 0);
    ASSERT_CHK(pEnv, pShape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE, 0);

    btScalar result = pShape->maxRadius();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_ConicalFrustum
 * Method:    scaledVolume
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_shapes_ConicalFrustum_scaledVolume
(JNIEnv *pEnv, jclass, jlong shapeId) {
    ConicalFrustumShape *pShape
            = reinterpret_cast<ConicalFrustumShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The ConicalFrustumShape does not exist.", 0);
    ASSERT_CHK(pEnv, pShape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE, 0);

    btScalar result = pShape->volume();
    return result;
}
