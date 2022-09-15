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
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.math.Plane;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;

/**
 * A utility class to generate debug meshes for Bullet collision shapes.
 *
 * @author CJ Hare, normenhansen
 */
final public class DebugShapeFactory {
    // *************************************************************************
    // constants and loggers

    /**
     * specify high-res debug meshes for convex shapes (up to 256 vertices)
     */
    final public static int highResolution = 1;
    /**
     * specify low-res debug meshes for convex shapes (up to 42 vertices)
     */
    final public static int lowResolution = 0;
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
     * Determine vertex locations for the specified collision shape. Note:
     * recursive!
     *
     * @param shape the input shape (not null, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    public static FloatBuffer debugVertices(CollisionShape shape,
            int meshResolution) {
        Validate.nonNull(shape, "shape");
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        FloatBuffer result;
        if (shape instanceof CompoundCollisionShape) {
            CompoundCollisionShape ccs = (CompoundCollisionShape) shape;
            result = createCompoundVertices(ccs, meshResolution);

        } else if (shape instanceof PlaneCollisionShape) {
            float halfExt = 1000f;
            result = createPlaneVertices((PlaneCollisionShape) shape, halfExt);

        } else {
            long shapeId = shape.nativeId();
            DebugMeshCallback callback = new DebugMeshCallback();
            getVertices(shapeId, meshResolution, callback);
            result = callback.getVertices();
        }

        assert (result.capacity() % numAxes) == 0 : result.capacity();
        return result;
    }

    /**
     * Generate vertex locations for triangles to visualize the specified
     * collision shape. Note: recursive!
     *
     * @param shape the shape to visualize (not null, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    public static FloatBuffer getDebugTriangles(CollisionShape shape,
            int meshResolution) {
        Validate.nonNull(shape, "shape");
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        FloatBuffer result;
        if (shape instanceof CompoundCollisionShape) {
            CompoundCollisionShape ccs = (CompoundCollisionShape) shape;
            result = createCompoundTriangles(ccs, meshResolution);

        } else if (shape instanceof PlaneCollisionShape) {
            float halfExt = 1000f;
            result = createPlaneTriangles((PlaneCollisionShape) shape, halfExt);

        } else {
            long shapeId = shape.nativeId();
            DebugMeshCallback callback = new DebugMeshCallback();
            getTriangles(shapeId, meshResolution, callback);
            result = callback.getVertices();
        }

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

        long shapeId = shape.nativeId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getVertices(shapeId, meshResolution, callback);
        float result = callback.maxDistance(transform);

        return result;
    }

    /**
     * Calculate the volume of a debug mesh for the specified convex shape. The
     * shape's scale and margin are taken into account, but not its debug-mesh
     * resolution.
     *
     * @param shape (not null, convex, unaffected)
     * @param meshResolution (0=low, 1=high)
     * @return the scaled volume (in physics-space units cubed, &ge;0)
     */
    public static float volumeConvex(ConvexShape shape, int meshResolution) {
        Validate.inRange(meshResolution, "mesh resolution", lowResolution,
                highResolution);

        long shapeId = shape.nativeId();
        DebugMeshCallback callback = new DebugMeshCallback();
        getTriangles(shapeId, meshResolution, callback);
        float volume = callback.volumeConvex();

        assert volume >= 0f : volume;
        return volume;
    }
    // *************************************************************************
    // private methods

    /**
     * Generate vertex locations for triangles to visualize the specified
     * CompoundCollisionShape.
     *
     * @param compoundShape (not null, unaffected)
     * @param meshResolution (0=low, 1=high)
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    private static FloatBuffer createCompoundTriangles(
            CompoundCollisionShape compoundShape, int meshResolution) {
        ChildCollisionShape[] children = compoundShape.listChildren();
        int numChildren = children.length;

        FloatBuffer[] bufferArray = new FloatBuffer[numChildren];
        Transform tmpTransform = new Transform();
        int totalFloats = 0;

        for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
            ChildCollisionShape child = children[childIndex];
            CollisionShape baseShape = child.getShape();
            child.copyTransform(tmpTransform);
            FloatBuffer buffer = getDebugTriangles(baseShape, meshResolution);

            int numFloats = buffer.capacity();
            MyBuffer.transform(buffer, 0, numFloats, tmpTransform);
            bufferArray[childIndex] = buffer;
            totalFloats += numFloats;
        }

        FloatBuffer result = BufferUtils.createFloatBuffer(totalFloats);
        for (FloatBuffer buffer : bufferArray) {
            for (int position = 0; position < buffer.capacity(); ++position) {
                float value = buffer.get(position);
                result.put(value);
            }
        }
        assert result.position() == result.capacity();

        return result;
    }

    /**
     * Determine vertex locations for the specified CompoundCollisionShape.
     *
     * @param compoundShape (not null, unaffected)
     * @param meshResolution (0=low, 1=high)
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    private static FloatBuffer createCompoundVertices(
            CompoundCollisionShape compoundShape, int meshResolution) {
        ChildCollisionShape[] children = compoundShape.listChildren();
        int numChildren = children.length;

        FloatBuffer[] bufferArray = new FloatBuffer[numChildren];
        Transform tmpTransform = new Transform();
        int totalFloats = 0;

        for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
            ChildCollisionShape child = children[childIndex];
            CollisionShape baseShape = child.getShape();
            child.copyTransform(tmpTransform);
            FloatBuffer buffer = debugVertices(baseShape, meshResolution);

            int numFloats = buffer.capacity();
            MyBuffer.transform(buffer, 0, numFloats, tmpTransform);
            bufferArray[childIndex] = buffer;
            totalFloats += numFloats;
        }

        FloatBuffer result = BufferUtils.createFloatBuffer(totalFloats);
        for (FloatBuffer buffer : bufferArray) {
            for (int position = 0; position < buffer.capacity(); ++position) {
                float value = buffer.get(position);
                result.put(value);
            }
        }
        assert result.position() == result.capacity();

        return result;
    }

    /**
     * Generate vertex locations for triangles to visualize the specified
     * PlaneCollisionShape.
     *
     * @param shape (not null, unaffected)
     * @param halfExtent the desired half extent for the result (in scaled shape
     * units, &gt;0)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    private static FloatBuffer
            createPlaneTriangles(PlaneCollisionShape shape, float halfExtent) {
        assert shape != null;
        assert halfExtent > 0f : halfExtent;
        /*
         * Generate vertex locations for a large square in the Y-Z plane.
         */
        FloatBuffer result = BufferUtils.createFloatBuffer(
                0f, 0f, -1f,
                0f, 1f, 0f,
                0f, 0f, 1f,
                0f, -1f, 0f,
                0f, 0f, -1f,
                0f, 0f, 1f
        );
        int numFloats = result.capacity();
        /*
         * Transform vertex locations to the surface of the shape.
         */
        Transform transform = planeTransform(shape);
        transform.setScale(halfExtent);
        MyBuffer.transform(result, 0, numFloats, transform);

        return result;
    }

    /**
     * Generate vertex locations for the specified PlaneCollisionShape.
     *
     * @param shape (not null, unaffected)
     * @param halfExtent the desired half extent for the result (in scaled shape
     * units, &gt;0)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    private static FloatBuffer
            createPlaneVertices(PlaneCollisionShape shape, float halfExtent) {
        assert shape != null;
        assert halfExtent > 0f : halfExtent;
        /*
         * Generate vertex locations for a large square in the Y-Z plane.
         */
        FloatBuffer result = BufferUtils.createFloatBuffer(
                0f, 0f, -1f,
                0f, 1f, 0f,
                0f, 0f, 1f,
                0f, -1f, 0f
        );
        int numFloats = result.capacity();
        /*
         * Transform vertex locations to the surface of the shape.
         */
        Transform transform = planeTransform(shape);
        transform.setScale(halfExtent);
        MyBuffer.transform(result, 0, numFloats, transform);

        return result;
    }

    /**
     * Generate a Transform that maps the Y-Z plane to the surface of the
     * specified PlaneCollisionShape.
     *
     * @param shape (not null, unaffected) units, &gt;0)
     * @return a new Transform with scale=1
     */
    private static Transform planeTransform(PlaneCollisionShape shape) {
        Transform result = new Transform();

        Plane plane = shape.getPlane();
        plane.getClosestPoint(translateIdentity, result.getTranslation());

        Vector3f v1 = plane.getNormal();
        Vector3f v2 = new Vector3f();
        Vector3f v3 = new Vector3f();
        MyVector3f.generateBasis(v1, v2, v3);
        result.getRotation().fromAxes(v1, v2, v3);

        return result;
    }
    // *************************************************************************
    // native private methods

    native private static void getTriangles(
            long shapeId, int meshResolution, DebugMeshCallback buffer);

    native private static void getVertices(
            long shapeId, int meshResolution, DebugMeshCallback buffer);
}
