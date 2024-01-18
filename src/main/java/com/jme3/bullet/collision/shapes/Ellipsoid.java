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
import jme3utilities.math.MyMath;

/**
 * A ellipsoidal collision shape. Unlike a {@code SphereCollisionShape}, these
 * shapes have margins and can be scaled non-uniformly.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Ellipsoid extends CustomConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * 4*Pi/3 in single precision
     */
    final private static float fourThirdsPi = 4.1887902f;
    /**
     * message logger for this class
     */
    final public static Logger loggerZ
            = Logger.getLogger(Ellipsoid.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the scaled half extents on each local axis, for margin=0
     */
    final private Vector3f scaledHe;
    /**
     * copy of the squared scaled half extents on each local axis, for margin=0
     */
    final private Vector3f squaredScaledHe;
    /**
     * copy of the half extents on each local axis, for scale=(1,1,1) and
     * margin=0
     */
    final private Vector3f unscaledHe;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an ellipsoid collision shape with the specified extent and
     * inertia.
     *
     * @param halfExtents the desired half extents on each local axis, for
     * scale=(1,1,1) and margin=0 (not null, all components &gt;0, unaffected)
     * @param inertia the desired local inertia vector for a shape with mass=1
     * and scale=(1,1,1) (not null, all components &gt;0, unaffected)
     */
    protected Ellipsoid(Vector3f halfExtents, Vector3f inertia) {
        super(halfExtents, inertia);

        this.scaledHe = halfExtents.clone();
        this.squaredScaledHe = scaledHe.mult(scaledHe);
        this.unscaledHe = halfExtents.clone();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Instantiate a triaxial ellipsoid.
     *
     * @param a the desired unscaled half extent on the local X axis (in shape
     * units, &gt;0)
     * @param b the desired unscaled half extent on the local Y axis (in shape
     * units, &gt;0)
     * @param c the desired unscaled half extent on the local Z axis (in shape
     * units, &gt;0)
     * @param inertiaFactor 0.2 for a uniform-density solid, 1/3 for a hollow
     * shell (&ge;0, &le;1)
     * @return a new instance
     */
    final public static Ellipsoid newInstance(
            float a, float b, float c, float inertiaFactor) {
        Validate.positive(a, "a");
        Validate.positive(b, "b");
        Validate.positive(c, "c");
        Validate.fraction(inertiaFactor, "inertia factor");

        // see https://adamheins.com/blog/ellipsoidal-shell-inertia
        float aa = a * a;
        float bb = b * b;
        float cc = c * c;
        float ix = inertiaFactor * (bb + cc);
        float iy = inertiaFactor * (aa + cc);
        float iz = inertiaFactor * (aa + bb);
        Vector3f inertia = new Vector3f(ix, iy, iz);

        Vector3f halfExtents = new Vector3f(a, b, c);
        Ellipsoid result = new Ellipsoid(halfExtents, inertia);

        return result;
    }
    // *************************************************************************
    // CustomConvexShape methods

    /**
     * Locate the ellipsoid's supporting vertex for the specified direction,
     * ignoring collision margin.
     * <p>
     * This method is invoked by native code.
     *
     * @param dirX the X-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @param dirY the Y-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @param dirZ the Z-coordinate of the direction to test (in scaled shape
     * coordinates)
     * @return the location of the supporting vertex (in scaled shape
     * coordinates)
     */
    @Override
    protected Vector3f locateSupport(float dirX, float dirY, float dirZ) {
        Vector3f result = threadTmpVector.get();

        float x = dirX * squaredScaledHe.x;
        float y = dirY * squaredScaledHe.y;
        float z = dirZ * squaredScaledHe.z;

        float dxyz = MyMath.hypotenuse(x, y, z);
        if (dxyz == 0f) {
            result.set(scaledHe.x, 0f, 0f);
        } else {
            result.x = scaledHe.x * (x / dxyz);
            result.y = scaledHe.y * (y / dxyz);
            result.z = scaledHe.z * (z / dxyz);
        }

        return result;
    }

    /**
     * Calculate how far the scaled shape extends from its center.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        float result = MyMath.max(scaledHe.x, scaledHe.y, scaledHe.z);
        return result;
    }

    /**
     * Estimate the volume of the collision shape, including scale and margin.
     *
     * @return the estimated volume (in physics-space units cubed, &ge;0)
     */
    @Override
    public float scaledVolume() {
        float a = scaledHe.x + margin;
        float b = scaledHe.y + margin;
        float c = scaledHe.z + margin;
        float result = fourThirdsPi * a * b * c;

        return result;
    }

    /**
     * Alter the scale of the shape.
     * <p>
     * Note that if shapes are shared (between collision objects and/or compound
     * shapes) changes can have unintended consequences.
     *
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    @Override
    public void setScale(Vector3f scale) {
        super.setScale(scale);

        if (unscaledHe != null) { // The Ellipsoid is fully instanciated:
            scale.mult(unscaledHe, scaledHe);
            scaledHe.mult(scaledHe, squaredScaledHe);
        }
    }
}
