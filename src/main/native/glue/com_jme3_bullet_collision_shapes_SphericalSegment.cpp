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
#include "com_jme3_bullet_collision_shapes_SphericalSegment.h"
#include "jmeBulletUtil.h"

/*
 * A collision shape for a spherical segment with uniform density.
 * By convention, both bases are orthogonal to the local Y axis.
 */
ATTRIBUTE_ALIGNED16(class)
SphericalSegmentShape : public btConvexInternalShape {
protected:
    /*
     * square of m_scaledRadius
     */
    btScalar m_scaledR2;
    /*
     * scaled radius of the parent sphere, excluding margin
     * (in physics-space units)
     */
    btScalar m_scaledRadius;
    /*
     * Y offset of the segment's center of mass
     * from the center of the parent sphere (in physics-space units):
     */
    btScalar m_scaledY0;
    /*
     * Y offset of the upper base from the center of the parent sphere
     * (in physics-space units):
     */
    btScalar m_scaledYMax;
    /*
     * Y offset of the lower base from the center of the parent sphere
     * (in physics-space units):
     */
    btScalar m_scaledYMin;
    /*
     * radius of the parent sphere, for scale=(1,1,1) and margin=0:
     */
    btScalar m_unscaledRadius;
    /*
     * Y offset of the upper base from the center of the parent sphere
     * for scale=(1,1,1) and margin=0:
     */
    btScalar m_unscaledYMax;
    /*
     * Y offset of the lower base from the center of the parent sphere,
     * for scale=(1,1,1) and margin=0:
     */
    btScalar m_unscaledYMin;
    /*
     * the shape's half extents on each local axis,
     * for scale=(1,1,1) and margin=0:
     */
    btVector3 m_descaledHalfExtents;
    /*
     * (rotational) inertia around each local axis, for mass=1:
     */
    btVector3 m_scaledInertia;
    /*
     * Return the Y offset of a segment's center of mass from the center of
     * the parent sphere.
     */
    static btScalar calcY0(btScalar radius, btScalar yMax, btScalar yMin) {
        btAssert(radius > 0);
        btAssert(yMax <= radius);
        btAssert(yMin <= yMax);
        btAssert(-radius <= yMin);

        btScalar max2 = yMax * yMax;
        btScalar min2 = yMin * yMin;
        btScalar r2 = radius * radius;

        btScalar squares = max2 + yMax * yMin + min2;
        btScalar denominator = 3 * r2 - squares;
        btAssert(denominator != 0);

        btScalar sum = yMax + yMin;
        btScalar cubes = (max2 + min2) * sum;
        btScalar numerator = 2 * r2 * sum - cubes;

        btScalar result = 0.75 * numerator / denominator;

        return result;
    }

public:
    BT_DECLARE_ALIGNED_ALLOCATOR();
    /*
     * constructor:
     */
    SphericalSegmentShape(btScalar radius, btScalar yMax, btScalar yMin)
    : btConvexInternalShape(),
      m_unscaledRadius(radius),
      m_unscaledYMax(yMax),
      m_unscaledYMin(yMin)
    {
        m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;

        // Initialize the descaled half extents:
        btScalar y0 = calcY0(radius, yMax, yMin);
        btScalar yyMax = yMax - y0;
        btAssert(yyMax >= 0);
        btScalar yyMin = yMin - y0;
        btAssert(yyMin <= 0);
        btScalar maxAbsYy = btMax(yyMax, -yyMin);
        btScalar maxR;
        if (yMax >= 0 && yMin <= 0) {
            // The segment includes the parent sphere's equator:
            maxR = radius;
        } else { // The segment doesn't include the parent sphere's equator:
            btScalar max2 = yMax * yMax;
            btScalar min2 = yMin * yMin;
            btScalar r2 = radius * radius;
            btScalar minYSquared = btMin(max2, min2);
            btScalar maxRSquared = r2 - minYSquared;
            maxR = btSqrt(maxRSquared);
        }
        m_descaledHalfExtents.setValue(maxR, maxAbsYy, maxR);

        setLocalScaling(btVector3(1, 1, 1));
    }

    virtual ~SphericalSegmentShape() {}

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
        return "SphericalSegment";
    }

    btVector3
    localGetSupportingVertexWithoutMargin(const btVector3& dir) const {
        btScalar y; // relative to the center of the parent sphere
        btScalar dxyz = dir.length();
        if (dxyz == 0) {
            y = 0.5 * (m_scaledYMax + m_scaledYMin);
        } else {
            y = m_scaledRadius * (dir.y() / dxyz);
            y = btClamped(y,  m_scaledYMin,  m_scaledYMax);
        }

        // The distance from the local Y axis, in physics-space units:
        btScalar rxz = btSqrt(m_scaledR2 - y * y);

        btVector3 result;
        btScalar dxz = btSqrt(dir.x() * dir.x() + dir.z() * dir.z());
        if (dxz == 0) {
            result.setValue(rxz, y - m_scaledY0, 0);
        } else {
            result.setValue(rxz * (dir.x() / dxz),
                            y - m_scaledY0,
                            rxz * (dir.z() / dxz));
        }
        return result;
    }

    /*
     * Calculate how far the scaled shape extends from its center of mass,
     * including collision margin.
     */
    btScalar maxRadius() const {
        btScalar yyMax = m_scaledYMax - m_scaledY0;
        btScalar d2YMax = yyMax * yyMax + m_scaledR2 - m_scaledYMax * m_scaledYMax;

        btScalar yyMin = m_scaledYMin - m_scaledY0;
        btScalar d2YMin = yyMin * yyMin + m_scaledR2 - m_scaledYMin * m_scaledYMin;

        btScalar d2 = btMax(d2YMax, d2YMin);
        if (m_scaledYMax > 0 && m_scaledYMin < 0) {
            btScalar d2Equator = m_scaledY0 * m_scaledY0 + m_scaledR2;
            d2 = btMax(d2, d2Equator);
        }

        btScalar result = btSqrt(d2) + getMargin();
        return result;
    }

    void
    setLocalScaling(const btVector3& scale) {
        btConvexInternalShape::setLocalScaling(scale);

        m_scaledRadius = scale.x() * m_unscaledRadius;
        m_scaledR2 = m_scaledRadius * m_scaledRadius;
        m_scaledYMax = scale.x() * m_unscaledYMax;
        m_scaledYMin = scale.x() * m_unscaledYMin;
        m_scaledY0 = calcY0(m_scaledRadius, m_scaledYMax, m_scaledYMin);

        btScalar max2 = m_scaledYMax * m_scaledYMax;
        btScalar max3 = max2 * m_scaledYMax;
        btScalar max5 = max2 * max3;

        btScalar min2 = m_scaledYMin * m_scaledYMin;
        btScalar min3 = min2 * m_scaledYMin;
        btScalar min5 = min2 * min3;

        btScalar height = m_scaledYMax - m_scaledYMin;
        btScalar h3 = max3 - min3; // difference of cubes
        btScalar h5 = max5 - min5;

        btScalar r4h = m_scaledR2 * m_scaledR2 * height;
        btScalar denom = m_scaledR2 * height - h3 / 3; // proportional to volume
        /*
         * the moments of inertia of a uniformly dense spherical segment
         * with mass=1, around its center of mass:
         */
        btScalar ixz = (3 * r4h + 2 * m_scaledR2 * h3 - 1.8 * h5) / (12 * denom) - m_scaledY0 * m_scaledY0;
        btScalar iy = (3 * r4h - 2 * m_scaledR2 * h3 + 0.6 * h5) / (6 * denom);
        m_scaledInertia.setValue(ixz, iy, ixz);
    }

    /*
     * Estimate the volume of the collision shape, including scale and margin.
     */
    btScalar volume() const {
        btScalar margin = getMargin();
        btScalar yMin = m_scaledYMin - margin;
        btScalar yMax = m_scaledYMax + margin;
        btScalar height = yMax - yMin;
        btScalar h3 = yMax * yMax * yMax - yMin * yMin * yMin; // difference of cubes
        btScalar radius = m_scaledRadius + margin;
        btScalar r2 = radius * radius;
        btScalar denom = r2 * height - h3 / 3; // proportional to volume
        btScalar result = SIMD_PI * denom;
        return result;
    }
};

/*
 * Class:     com_jme3_bullet_collision_shapes_SphericalSegment
 * Method:    createShapeNative
 * Signature: (FFF)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_SphericalSegment_createShapeNative
(JNIEnv *pEnv, jclass, jfloat radius, jfloat yMax, jfloat yMin) {
    jmeClasses::initJavaClasses(pEnv);
    SphericalSegmentShape *pShape
            = new SphericalSegmentShape(radius, yMax, yMin); //dance016
    return reinterpret_cast<jlong> (pShape);
}

/*
 * Class:     com_jme3_bullet_collision_shapes_SphericalSegment
 * Method:    maxRadius
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_shapes_SphericalSegment_maxRadius
(JNIEnv *pEnv, jclass, jlong shapeId) {
    SphericalSegmentShape *pShape
            = reinterpret_cast<SphericalSegmentShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The SphericalSegmentShape does not exist.", 0);
    ASSERT_CHK(pEnv, pShape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE, 0);

    btScalar result = pShape->maxRadius();
    return result;
}

/*
 * Class:     com_jme3_bullet_collision_shapes_SphericalSegment
 * Method:    scaledVolume
 * Signature: (J)F
 */
JNIEXPORT jfloat JNICALL Java_com_jme3_bullet_collision_shapes_SphericalSegment_scaledVolume
(JNIEnv *pEnv, jclass, jlong shapeId) {
    SphericalSegmentShape *pShape
            = reinterpret_cast<SphericalSegmentShape *> (shapeId);
    NULL_CHK(pEnv, pShape, "The SphericalSegmentShape does not exist.", 0);
    ASSERT_CHK(pEnv, pShape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE, 0);

    btScalar result = pShape->volume();
    return result;
}
