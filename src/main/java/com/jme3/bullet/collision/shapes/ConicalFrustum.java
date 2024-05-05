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
package com.jme3.bullet.collision.shapes;

import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision shape for a conical frustum with uniform density, based on
 * {@code btConvexInternalShape}. By convention, the local Y axis is the height
 * axis, with the "A" base having y&lt;0 and the "B" base having y&gt;0.
 * <p>
 * This is an imprecise shape; margin always expands the shape.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ConicalFrustum extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(ConicalFrustum.class.getName());
    // *************************************************************************
    // fields

    /**
     * radius of the "A" base, for scale=(1,1,1) and margin=0
     */
    final private float unscaledA;
    /**
     * radius of the "B" base, for scale=(1,1,1) and margin=0
     */
    final private float unscaledB;
    /**
     * height, for scale=(1,1,1) and margin=0
     */
    final private float unscaledHeight;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a conical frustum with the specified dimensions.
     *
     * @param a the desired radius of the "A" base for scale=(1,1,1) (&gt;0)
     * @param b the desired radius of the "B" base for scale=(1,1,1) (&gt;0)
     * @param height the desired height for scale=(1,1,1) (&gt;0)
     */
    public ConicalFrustum(float a, float b, float height) {
        Validate.positive(a, "A radius");
        Validate.positive(b, "B radius");
        Validate.positive(height, "height");

        this.unscaledA = a;
        this.unscaledB = b;
        this.unscaledHeight = height;

        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the radius of the "A" base.
     *
     * @return the unscaled radius (&gt;0)
     */
    public float aRadius() {
        assert unscaledA > 0f : unscaledA;
        return unscaledA;
    }

    /**
     * Return the radius of the "B" base.
     *
     * @return the unscaled radius (&gt;0)
     */
    public float bRadius() {
        assert unscaledB > 0f : unscaledB;
        return unscaledB;
    }

    /**
     * Return the height of the frustum.
     *
     * @return the unscaled height (&gt;0)
     */
    public float height() {
        assert unscaledHeight > 0f : unscaledHeight;
        return unscaledHeight;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to the shape. For
     * a conical frustum, scaling must preserve the circular cross section.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean result = super.canScale(scale) && scale.x == scale.z;
        return result;
    }

    /**
     * Calculate how far the scaled shape extends from its center of mass,
     * including collision margin.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        long shapeId = nativeId();
        float result = maxRadius(shapeId);
        return result;
    }

    /**
     * Estimate the volume of the collision shape, including scale and margin.
     *
     * @return the estimated volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        long shapeId = nativeId();
        float result = scaledVolume(shapeId);
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        long shapeId
                = createShapeNative(unscaledA, unscaledB, unscaledHeight);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShapeNative(
            float aRadius, float bRadius, float height);

    native private static float maxRadius(long shapeId);

    native private static float scaledVolume(long shapeId);
}
