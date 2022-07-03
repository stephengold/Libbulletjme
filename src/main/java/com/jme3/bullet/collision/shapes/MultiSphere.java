/*
 * Copyright (c) 2018-2022 jMonkeyEngine
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
package com.jme3.bullet.collision.shapes;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Vector3f;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A convex CollisionShape based on Bullet's btMultiSphereShape. Unlike a
 * CapsuleCollisionShape or a SphereCollisionShape, these shapes have margins
 * and can be scaled non-uniformly.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiSphere extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MultiSphere.class.getName());
    // *************************************************************************
    // fields

    /**
     * copies of the unscaled radii (each &ge;0)
     */
    final private float[] radii;
    /**
     * copies of center locations
     */
    final private Vector3f[] centers;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a centered sphere shape with the specified radius.
     *
     * @param radius the desired unscaled radius (&ge;0)
     */
    public MultiSphere(float radius) {
        Validate.nonNegative(radius, "radius");

        centers = new Vector3f[1];
        centers[0] = new Vector3f(0f, 0f, 0f);

        radii = new float[1];
        radii[0] = radius;

        createShape();
    }

    /**
     * Instantiate a centered Y-axis capsule shape with the specified radius and
     * height.
     *
     * @param radius the desired unscaled radius (&ge;0)
     * @param height the desired unscaled height of the cylindrical portion
     * (&ge;0)
     */
    public MultiSphere(float radius, float height) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        float halfHeight = height / 2f;

        centers = new Vector3f[2];
        centers[0] = new Vector3f(0f, halfHeight, 0f);
        centers[1] = new Vector3f(0f, -halfHeight, 0f);

        radii = new float[2];
        radii[0] = radius;
        radii[1] = radius;

        createShape();
    }

    /**
     * Instantiate a centered capsule shape with the specified radius, height,
     * and axis.
     *
     * @param radius the desired unscaled radius (&ge;0)
     * @param height the desired unscaled height of the cylindrical portion
     * (&ge;0)
     * @param axisIndex which local axis to use for the height: 0&rarr;X,
     * 1&rarr;Y, 2&rarr;Z
     */
    public MultiSphere(float radius, float height, int axisIndex) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        float halfHeight = height / 2f;

        centers = new Vector3f[2];
        switch (axisIndex) {
            case PhysicsSpace.AXIS_X:
                centers[0] = new Vector3f(halfHeight, 0f, 0f);
                centers[1] = new Vector3f(-halfHeight, 0f, 0f);
                break;
            case PhysicsSpace.AXIS_Y:
                centers[0] = new Vector3f(0f, halfHeight, 0f);
                centers[1] = new Vector3f(0f, -halfHeight, 0f);
                break;
            case PhysicsSpace.AXIS_Z:
                centers[0] = new Vector3f(0f, 0f, halfHeight);
                centers[1] = new Vector3f(0f, 0f, -halfHeight);
                break;
            default:
                throw new IllegalArgumentException("axisIndex = " + axisIndex);
        }

        radii = new float[2];
        radii[0] = radius;
        radii[1] = radius;

        createShape();
    }

    /**
     * Instantiate a multi-sphere shape with the specified centers and radii.
     *
     * @param centers the list of center offsets (not null, not empty)
     * @param radii the list of radii (not null, not empty, each &ge;0)
     */
    public MultiSphere(List<Vector3f> centers, List<Float> radii) {
        Validate.nonEmpty(centers, "centers");
        Validate.nonEmpty(radii, "radii");

        int numSpheres = radii.size();
        Validate.require(centers.size() == numSpheres, "lists of equal length");

        this.centers = new Vector3f[numSpheres];
        this.radii = new float[numSpheres];
        for (int i = 0; i < numSpheres; ++i) {
            this.centers[i] = centers.get(i).clone();
            float radius = radii.get(i);
            assert radius >= 0f : radius;
            this.radii[i] = radius;
        }

        createShape();
    }

    /**
     * Instantiate an eccentric sphere shape with the specified center and
     * radius.
     *
     * @param center the offset of the center (not null, unaffected)
     * @param radius the desired unscaled radius (&ge;0)
     */
    public MultiSphere(Vector3f center, float radius) {
        Validate.finite(center, "center");
        Validate.nonNegative(radius, "radius");

        centers = new Vector3f[1];
        centers[0] = center.clone();

        radii = new float[1];
        radii[0] = radius;

        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the offset of the center of the indexed sphere.
     *
     * @param sphereIndex which sphere to read (&ge;0)
     * @param storeResult storage for the result (modified if not null)
     * @return the center offset (either storeResult or a new instance, not
     * null)
     */
    public Vector3f copyCenter(int sphereIndex, Vector3f storeResult) {
        Validate.inRange(sphereIndex, "sphere index", 0, radii.length - 1);
        Vector3f result = storeResult == null ? new Vector3f() : storeResult;

        result.set(centers[sphereIndex]);

        return result;
    }

    /**
     * Count the spheres in this shape.
     *
     * @return the count (&gt;0)
     */
    public int countSpheres() {
        int count = radii.length;
        assert count > 0 : count;
        return count;
    }

    /**
     * Read the radius of the indexed sphere.
     *
     * @param sphereIndex which sphere to read (&ge;0)
     * @return the unscaled radius (&ge;0)
     */
    public float getRadius(int sphereIndex) {
        Validate.inRange(sphereIndex, "sphere index", 0, radii.length - 1);
        float radius = radii[sphereIndex];

        assert radius >= 0f : radius;
        return radius;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long shapeId = nativeId();
        recalcAabb(shapeId);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        int numSpheres = radii.length;
        assert centers.length == numSpheres : numSpheres;

        long shapeId = createShape(centers, radii, numSpheres);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(Vector3f[] centers, float[] radii,
            int numSpheres);

    native private static void recalcAabb(long shapeId);
}
