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

import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;

/**
 * A planar collision shape based on Bullet's {@code btStaticPlaneShape}. Not
 * for use in dynamic bodies. Collisions between HeightfieldCollisionShape,
 * MeshCollisionShape, and PlaneCollisionShape objects are never detected.
 *
 * @author normenhansen
 */
public class PlaneCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PlaneCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * defining plane
     */
    final private Plane plane;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a plane shape defined by the specified plane.
     *
     * @param plane the desired plane (not null, unaffected)
     */
    public PlaneCollisionShape(Plane plane) {
        this.plane = plane.clone();
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the defining plane.
     *
     * @return a new instance (not null)
     */
    final public Plane getPlane() {
        Plane result = plane.clone();
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Determine how far this shape extends from its center.
     *
     * @return the distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        return Float.POSITIVE_INFINITY;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code btStaticPlaneShape}.
     */
    private void createShape() {
        long shapeId = createShape(plane.getNormal(), plane.getConstant());
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(Vector3f normal, float constant);
}
