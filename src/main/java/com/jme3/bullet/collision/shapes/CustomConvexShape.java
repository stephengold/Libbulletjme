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
 * The abstract base class for custom collision shapes defined in terms of their
 * supporting vertices, based on Bullet's {@code btConvexInternalShape}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class CustomConvexShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerY
            = Logger.getLogger(CustomConvexShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * temporary storage for one vector per thread
     */
    final protected static ThreadLocal<Vector3f> threadTmpVector
            = new ThreadLocal<Vector3f>() {
        @Override
        protected Vector3f initialValue() {
            return new Vector3f();
        }
    };
    /**
     * copy of the half extents on each local axis, for scale=(1,1,1) and
     * margin=0, or {@code null} to calculate AABBs using the supporting
     * vertices
     */
    final private Vector3f halfExtents;
    /**
     * copy of the rotational inertia for each local axis, for a shape with
     * mass=1 and scale=(1,1,1)
     */
    final private Vector3f inertia;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a custom collision shape with the specified extent and
     * inertia.
     *
     * @param halfExtents the desired half extents on each local axis, for
     * scale=(1,1,1) and margin=0 (unaffected), or {@code null} to calculate
     * AABBs using the supporting vertices
     * @param inertia the desired local inertia vector for a shape with mass=1
     * and scale=(1,1,1) (not null, all components &gt;0, unaffected)
     */
    public CustomConvexShape(Vector3f halfExtents, Vector3f inertia) {
        Validate.positive(inertia, "inertia");

        this.halfExtents = (halfExtents == null) ? null : halfExtents.clone();
        this.inertia = inertia.clone();
        createShape();
    }
    // *************************************************************************
    // new protected methods

    /**
     * Locate the shape's supporting vertex for the specified direction,
     * excluding collision margin.
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
    abstract protected Vector3f locateSupport(
            float dirX, float dirY, float dirZ);
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape in Bullet.
     */
    private void createShape() {
        long shapeId = createShapeNative(halfExtents, inertia);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private long createShapeNative(Vector3f he, Vector3f inertia);
}
