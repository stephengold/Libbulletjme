/*
 * Copyright (c) 2009-2021 jMonkeyEngine
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
package com.jme3.math;

import java.util.logging.Logger;

/**
 * <code>Plane</code> defines a plane where Normal dot (x,y,z) = Constant.
 * This provides methods for calculating a "distance" of a point from this
 * plane. The distance is pseudo due to the fact that it can be negative if the
 * point is on the non-normal side of the plane.
 *
 * @author Mark Powell
 * @author Joshua Slack
 * @author Ian McClean
 */
public class Plane implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;

    private static final Logger logger = Logger
            .getLogger(Plane.class.getName());

    /**
     * Vector normal to the plane.
     */
    protected Vector3f normal = new Vector3f();

    /**
     * Constant of the plane. See formula in class definition.
     */
    protected float constant;

    /**
     * Constructor instantiates a new <code>Plane</code> object. The normal
     * and constant values are set at creation.
     *
     * @param normal
     *            the normal of the plane.
     * @param constant
     *            the constant of the plane.
     */
    public Plane(Vector3f normal, float constant) {
        if (normal == null) {
            throw new IllegalArgumentException("normal cannot be null");
        }

        this.normal.set(normal);
        this.constant = constant;
    }

    /**
     * Constructor instantiates a new <code>Plane</code> object.
     *
     * @param normal      The normal of the plane.
     * @param displacement A vector representing a point on the plane.
     */
    public Plane(Vector3f normal, Vector3f displacement) {
        this(normal, displacement.dot(normal));
    }

    /**
     * <code>getNormal</code> retrieves the normal of the plane.
     *
     * @return the normal of the plane.
     */
    public Vector3f getNormal() {
        return normal;
    }

    /**
     * <code>getConstant</code> returns the constant of the plane.
     *
     * @return the constant of the plane.
     */
    public float getConstant() {
        return constant;
    }

    /**
     * Find the point in this plane that's nearest to the specified point.
     *
     * @param point the location of the input point (not null, unaffected)
     * @param store storage for the result (not null, modified)
     * @return the location of the nearest point (store)
     */
    public Vector3f getClosestPoint(Vector3f point, Vector3f store) {
//        float t = constant - normal.dot(point);
//        return store.set(normal).multLocal(t).addLocal(point);
        float t = (constant - normal.dot(point)) / normal.dot(normal);
        return store.set(normal).multLocal(t).addLocal(point);
    }

    /**
     * <code>pseudoDistance</code> calculates the distance from this plane to
     * a provided point. If the point is on the negative side of the plane the
     * distance returned is negative, otherwise it is positive. If the point is
     * on the plane, it is zero.
     *
     * @param point
     *            the point to check.
     * @return the signed distance from the plane to a point.
     */
    public float pseudoDistance(Vector3f point) {
        return normal.dot(point) - constant;
    }

    /**
     * <code>toString</code> returns a string representation of this plane.
     * It represents the normal as a <code>Vector3f</code>, so the format is:
     *
     * Plane [Normal: (X.XXXX, Y.YYYY, Z.ZZZZ) - Constant: C.CCCC]
     *
     * @return the string representation of this plane.
     */
    @Override
    public String toString() {
        return getClass().getSimpleName() + " [Normal: " + normal + " - Constant: "
                + constant + "]";
    }

    /**
     * Create a copy of this plane.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public Plane clone() {
        try {
            Plane p = (Plane) super.clone();
            p.normal = normal.clone();
            return p;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
