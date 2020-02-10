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
package com.jme3.bullet.util;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A utility class to generate debug meshes for Bullet collision shapes.
 *
 * @author CJ Hare, normenhansen
 */
public class DebugShapeFactory {
    // *************************************************************************
    // constants and loggers

    /**
     * specify high-res debug mesh for convex shapes (up to 256 vertices)
     */
    public static final int highResolution = 1;
    /**
     * specify low-res debug mesh for convex shapes (up to 42 vertices)
     */
    public static final int lowResolution = 0;
    /**
     * number of axes
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DebugShapeFactory.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private DebugShapeFactory() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate vertex locations for triangles to visualize the specified
     * collision shape.
     *
     * @param shape the shape to visualize (not null, not compound, not plane,
     * unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return a new direct buffer containing scaled shape coordinates (capacity
     * a multiple of 9)
     */
    public static FloatBuffer getDebugMeshVertices(CollisionShape shape,
            int meshResolution) {
        assert !(shape instanceof CompoundCollisionShape);
        assert !(shape instanceof PlaneCollisionShape);
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        long shapeId = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(shapeId, meshResolution, callback);
        FloatBuffer result = callback.getVertices();

        assert (result.capacity() % 9) == 0 : result.capacity();
        return result;
    }

    /**
     * Estimate how far the specified (non-compound, non-plane) shape extends
     * from some origin, based on its debug mesh. The shape's scale and margin
     * are taken into account, but not its debug-mesh resolution.
     *
     * @param shape (not null, not compound, not plane, unaffected)
     * @param transform the transform to apply to debug-mesh coordinates (not
     * null, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return the maximum length of the transformed vertex locations (&ge;0)
     */
    public static float maxDistance(CollisionShape shape, Transform transform,
            int meshResolution) {
        assert !(shape instanceof CompoundCollisionShape);
        assert !(shape instanceof PlaneCollisionShape);
        Validate.nonNull(transform, "transform");
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        long shapeId = shape.getObjectId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices2(shapeId, meshResolution, callback);
        float result = callback.maxDistance(transform);

        return result;
    }
    // *************************************************************************
    // native methods

    // TODO add a method to return vertices only (instead of a triangle mesh)
    native private static void getVertices2(long shapeId, int meshResolution,
            DebugMeshCallback buffer);
}
