/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.math.Vector3f;
import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * A capsule collision shape based on Bullet's {@code btCapsuleShapeX},
 * {@code btCapsuleShape}, or {@code btCapsuleShapeZ}. These shapes have no
 * margin and can only be scaled uniformly.
 *
 * @author normenhansen
 * @see MultiSphere
 */
public class CapsuleCollisionShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CapsuleCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled height of the cylindrical portion (in shape units,
     * &ge;0)
     */
    final private float height;
    /**
     * copy of the unscaled radius (in shape units, &ge;0)
     */
    final private float radius;
    /**
     * copy of the index of the main (height) axis (0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z)
     */
    final private int axis;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Y-axis capsule shape with the specified radius and height.
     *
     * @param radius the desired unscaled radius (in shape units, &ge;0)
     * @param height the desired unscaled height of the cylindrical portion (in
     * shape units, &ge;0)
     */
    public CapsuleCollisionShape(float radius, float height) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        this.radius = radius;
        this.height = height;
        this.axis = PhysicsSpace.AXIS_Y;
        createShape();
    }

    /**
     * Instantiate a capsule shape around the specified main (height) axis.
     *
     * @param radius the desired radius (in shape units, &ge;0)
     * @param height the desired height of the cylindrical portion (in shape
     * units, &ge;0)
     * @param axisIndex which local axis to use for the height: 0&rarr;X,
     * 1&rarr;Y, 2&rarr;Z (default=1)
     */
    public CapsuleCollisionShape(float radius, float height, int axisIndex) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");
        Validate.axisIndex(axisIndex, "axis index");

        this.radius = radius;
        this.height = height;
        this.axis = axisIndex;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the main (height) axis of the capsule.
     *
     * @return the axis index: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public int getAxis() {
        assert axis == PhysicsSpace.AXIS_X
                || axis == PhysicsSpace.AXIS_Y
                || axis == PhysicsSpace.AXIS_Z : axis;
        return axis;
    }

    /**
     * Return the height of the cylindrical portion.
     *
     * @return the unscaled height (in shape units, &ge;0)
     */
    public float getHeight() {
        assert height >= 0f : height;
        return height;
    }

    /**
     * Return the radius of the capsule.
     *
     * @return the unscaled radius (in shape units, &ge;0)
     */
    public float getRadius() {
        assert radius >= 0f : radius;
        return radius;
    }

    /**
     * Return the unscaled volume of the capsule.
     *
     * @return the volume (in shape units cubed, &ge;0)
     */
    public float unscaledVolume() {
        float result = MyVolume.capsuleVolume(radius, height);

        assert result >= 0f : result;
        return result;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to the shape. For
     * capsule shapes, scaling must be uniform.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale
                = super.canScale(scale) && MyVector3f.isScaleUniform(scale);
        return canScale;
    }

    /**
     * Return the collision margin of the shape.
     *
     * @return 0
     */
    @Override
    public float getMargin() {
        return 0f;
    }

    /**
     * Calculate how far the scaled shape extends from its center.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        float result = scale.x * (radius + height / 2f);
        return result;
    }

    /**
     * Estimate the volume of the shape, including scale and margin.
     *
     * @return the volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        float result = unscaledVolume() * scale.x * scale.y * scale.z;
        return result;
    }

    /**
     * Alter the collision margin of the shape. This feature is disabled for
     * capsule shapes.
     *
     * @param margin the desired margin thickness (in physics-space units)
     */
    @Override
    public void setMargin(float margin) {
        logger2.log(Level.WARNING,
                "Cannot alter the margin of a CapsuleCollisionShape.");
    }

    /**
     * Approximate this shape with a HullCollisionShape.
     *
     * @return a new shape
     */
    @Override
    public HullCollisionShape toHullShape() {
        float defaultMargin = getDefaultMargin();
        float effectiveRadius = scale.x * radius; // in physics-space units

        HullCollisionShape result;
        if (effectiveRadius > defaultMargin) {
            // Use 42 vertices with the default margin.
            CapsuleCollisionShape shrunkenCapsule = new CapsuleCollisionShape(
                    effectiveRadius - defaultMargin, height, axis);
            FloatBuffer buffer = DebugShapeFactory.debugVertices(
                    shrunkenCapsule, DebugShapeFactory.lowResolution);

            // Flip the buffer.
            buffer.rewind();
            buffer.limit(buffer.capacity());

            result = new HullCollisionShape(buffer);

        } else { // Use 2 vertices with a reduced margin.
            Vector3f v1 = new Vector3f();
            v1.set(axis, height / 2f);
            Vector3f v2 = v1.negate();
            result = new HullCollisionShape(v1, v2);
            if (effectiveRadius <= 1e-9f) {
                result.setMargin(1e-9f);
            } else {
                result.setMargin(effectiveRadius);
            }
        }

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape.
     */
    private void createShape() {
        assert axis == PhysicsSpace.AXIS_X
                || axis == PhysicsSpace.AXIS_Y
                || axis == PhysicsSpace.AXIS_Z : axis;
        assert radius >= 0f : radius;
        assert height >= 0f : height;

        long shapeId = createShape(axis, radius, height);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        this.margin = 0f;
    }
    // *************************************************************************
    // native private methods

    native private static long
            createShape(int axisIndex, float radius, float height);
}
