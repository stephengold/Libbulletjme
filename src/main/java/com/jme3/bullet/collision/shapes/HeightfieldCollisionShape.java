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
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * A collision shape for terrain defined by a matrix of height values, based on
 * Bullet's {@code btHeightfieldTerrainShape}. Should be more efficient than an
 * equivalent {@code MeshCollisionShape}. Not for use in dynamic bodies.
 * Collisions between {@code HeightfieldCollisionShape},
 * {@code MeshCollisionShape}, and {@code PlaneCollisionShape} objects are never
 * detected.
 *
 * @author Brent Owens
 */
public class HeightfieldCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HeightfieldCollisionShape.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // fields

    /**
     * reverse the direction of the first diagonal
     */
    private boolean flipQuadEdges = true;
    /**
     * true&rarr;left-hand winding of triangles
     */
    private boolean flipTriangleWinding = false;
    /**
     * true&rarr;diagonals alternate on both horizontal axes
     */
    private boolean useDiamond = false;
    /**
     * true&rarr;diagonals alternate on one horizontal axis
     */
    private boolean useZigzag = false;
    /**
     * highest sample in the heightfield or -minHeight, whichever is higher
     */
    private float maxHeight;
    /**
     * lowest sample in the heightfield or -maxHeight, whichever is lower
     */
    private float minHeight;
    /**
     * array of heightfield samples
     */
    private float[] heightfieldData;
    /**
     * direct buffer for passing height data to Bullet
     * <p>
     * A Java reference must persist after createShape() completes, or else the
     * buffer might get garbage collected.
     */
    private FloatBuffer directBuffer;
    /**
     * copy of number of columns in the heightfield (&gt;1)
     */
    private int heightStickLength;
    /**
     * copy of number of rows in the heightfield (&gt;1)
     */
    private int heightStickWidth;
    /**
     * copy of the height-axis index (0&rarr;X, 1&rarr;Y, 2&rarr;Z)
     */
    private int upAxis = PhysicsSpace.AXIS_Y;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a square shape for the specified array of heights.
     *
     * @param heightArray (not null, length&ge;4, length a perfect square,
     * unaffected)
     */
    public HeightfieldCollisionShape(float[] heightArray) {
        Validate.nonEmpty(heightArray, "height array");
        Validate.inRange(heightArray.length, "number of heights",
                4, Integer.MAX_VALUE);

        createCollisionHeightfield(heightArray, scaleIdentity);
    }

    /**
     * Instantiate a square shape for the specified height array and scale
     * vector.
     *
     * @param heightArray (not null, length&ge;4, length a perfect square,
     * unaffected)
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public HeightfieldCollisionShape(float[] heightArray, Vector3f scale) {
        Validate.nonEmpty(heightArray, "height array");
        Validate.inRange(heightArray.length, "number of heights",
                4, Integer.MAX_VALUE);
        Validate.nonNegative(scale, "scale");

        createCollisionHeightfield(heightArray, scale);
    }

    /**
     * Instantiate a rectangular shape for the specified parameters.
     *
     * @param stickLength the number of rows in the heightfield (&gt;1)
     * @param stickWidth number of columns in the heightfield (&gt;1)
     * @param heightmap (not null, length&ge;stickLength*stickWidth, unaffected)
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     * @param upAxis the height-axis index (0&rarr;X, 1&rarr;Y, 2&rarr;Z,
     * default=1)
     * @param flipQuadEdges true&rarr;reverse the direction of the first
     * diagonal (default=true)
     * @param flipTriangleWinding true&rarr;left-hand winding of triangles
     * (default=false)
     * @param useDiamond true&rarr;diagonals alternate on both horizontal axes
     * (default=false)
     * @param useZigzag true&rarr;diagonals alternate on one horizontal axis
     * (default=false)
     */
    public HeightfieldCollisionShape(int stickLength, int stickWidth,
            float[] heightmap, Vector3f scale, int upAxis,
            boolean flipQuadEdges, boolean flipTriangleWinding,
            boolean useDiamond, boolean useZigzag) {
        Validate.inRange(stickLength, "stick length", 2, Integer.MAX_VALUE);
        Validate.inRange(stickWidth, "stick width", 2, Integer.MAX_VALUE);
        Validate.nonEmpty(heightmap, "heightmap");
        assert heightmap.length >= stickLength * stickWidth : heightmap.length;
        Validate.nonNegative(scale, "scale");
        Validate.axisIndex(upAxis, "up axis");

        this.heightStickLength = stickLength;
        this.heightStickWidth = stickWidth;
        this.heightfieldData = heightmap.clone();
        this.scale.set(scale);
        this.upAxis = upAxis;
        this.flipQuadEdges = flipQuadEdges;
        this.flipTriangleWinding = flipTriangleWinding;
        this.useDiamond = useDiamond;
        this.useZigzag = useZigzag;

        calculateMinAndMax();
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many columns are in the heightfield.
     *
     * @return the count (&ge;2)
     */
    public int countColumns() {
        assert heightStickWidth >= 2 : heightStickWidth;
        return heightStickWidth;
    }

    /**
     * Count how many heights are in the heightfield.
     *
     * @return the count (&ge;4)
     */
    public int countMeshVertices() {
        int count = heightfieldData.length;

        assert count >= 4 : count;
        return count;
    }

    /**
     * Count how many rows are in the heightfield.
     *
     * @return the count (&ge;2)
     */
    public int countRows() {
        assert heightStickLength >= 2 : heightStickLength;
        return heightStickLength;
    }

    /**
     * Return the index of the height axis.
     *
     * @return the axis index: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public int upAxis() {
        assert upAxis == PhysicsSpace.AXIS_X
                || upAxis == PhysicsSpace.AXIS_Y
                || upAxis == PhysicsSpace.AXIS_Z : upAxis;
        return upAxis;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Approximate this shape with a splittable shape.
     *
     * @return a new splittable shape
     */
    @Override
    public CollisionShape toSplittableShape() {
        // Generate debug triangles.
        FloatBuffer buffer = DebugShapeFactory.getDebugTriangles(
                this, DebugShapeFactory.lowResolution);
        IndexedMesh nativeMesh = new IndexedMesh(buffer);
        MeshCollisionShape result = new MeshCollisionShape(true, nativeMesh);

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Calculate min and max heights for the heightfield data.
     */
    private void calculateMinAndMax() {
        int elements = heightStickLength * heightStickWidth;
        assert elements == heightfieldData.length : heightfieldData.length;

        float min = heightfieldData[0];
        float max = heightfieldData[0];

        // Find the min and max heights in the data.
        for (float height : heightfieldData) {
            if (height < min) {
                min = height;
            }
            if (height > max) {
                max = height;
            }
        }
        /*
         * Center the terrain's bounding box at y=0 by setting the
         * min and max height to have equal magnitudes and opposite signs.
         * Otherwise, the collision shape won't match the rendered heights.
         */
        if (max < 0) {
            max = -min;
        } else if (Math.abs(max) > Math.abs(min)) {
            min = -max;
        } else {
            max = -min;
        }
        this.minHeight = min;
        this.maxHeight = max;
    }

    /**
     * Instantiate a square {@code btHeightfieldTerrainShape}.
     *
     * @param heightArray (not null, length&ge;4, length a perfect square,
     * unaffected)
     * @param worldScale the desired scale factor for each local axis (not null,
     * no negative component, unaffected)
     */
    private void createCollisionHeightfield(
            float[] heightArray, Vector3f worldScale) {
        scale.set(worldScale);

        this.heightfieldData = heightArray.clone();
        this.heightStickWidth = (int) FastMath.sqrt(heightfieldData.length);
        assert heightStickWidth > 1 : heightStickWidth;

        this.heightStickLength = heightStickWidth;

        calculateMinAndMax();
        createShape();
    }

    /**
     * Instantiate the configured {@code btHeightfieldTerrainShape}.
     */
    private void createShape() {
        this.directBuffer
                = BufferUtils.createFloatBuffer(heightfieldData.length);
        for (float height : heightfieldData) {
            if (!MyMath.isFinite(height)) {
                throw new IllegalArgumentException("illegal height: " + height);
            }
            directBuffer.put(height);
        }

        float heightScale = 1f;
        long shapeId = createShape2(heightStickWidth, heightStickLength,
                directBuffer, heightScale, minHeight, maxHeight, upAxis,
                flipQuadEdges, flipTriangleWinding, useDiamond, useZigzag);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param shapeId the native identifier (not zero)
     */
    private static void freeNativeObject(long shapeId) {
        assert shapeId != 0L;
        finalizeNative(shapeId);
    }
    // *************************************************************************
    // native private methods

    native private static long createShape2(int stickWidth, int stickLength,
            FloatBuffer heightfieldData, float heightScale, float minHeight,
            float maxHeight, int upAxis, boolean flipQuadEdges,
            boolean flipTriangleWinding, boolean useDiamond, boolean useZigzag);

    native private static void finalizeNative(long shapeId);
}
