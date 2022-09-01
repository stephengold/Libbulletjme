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
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyVector3f;
import vhacd.VHACDHull;
import vhacd4.Vhacd4Hull;

/**
 * A convex-hull CollisionShape based on Bullet's btConvexHullShape. For a 2-D
 * convex hull, use Convex2dShape.
 */
public class HullCollisionShape extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HullCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * direct buffer for passing vertices to Bullet
     * <p>
     * A Java reference must persist after createShape() completes, or else the
     * buffer might get garbage collected.
     */
    private FloatBuffer directBuffer;
    /**
     * array of mesh coordinates (not null, not empty, length a multiple of 3)
     */
    final private float[] points;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a shape based on the specified collection of locations. For
     * best performance and stability, the convex hull should have no more than
     * 100 vertices.
     *
     * @param locations a collection of location vectors on which to base the
     * shape (not null, not empty, unaffected)
     */
    public HullCollisionShape(Collection<Vector3f> locations) {
        Validate.nonEmpty(locations, "locations");

        int numLocations = locations.size();
        this.points = new float[numAxes * numLocations];
        int j = 0;
        for (Vector3f location : locations) {
            points[j + PhysicsSpace.AXIS_X] = location.x;
            points[j + PhysicsSpace.AXIS_Y] = location.y;
            points[j + PhysicsSpace.AXIS_Z] = location.z;
            j += numAxes;
        }

        createShape();
    }

    /**
     * Instantiate a shape based on an array containing coordinates. For best
     * performance and stability, the convex hull should have no more than 100
     * vertices.
     *
     * @param points an array of coordinates on which to base the shape (not
     * null, not empty, length a multiple of 3, unaffected)
     */
    public HullCollisionShape(float... points) {
        Validate.nonEmpty(points, "points");
        Validate.require(
                points.length % numAxes == 0, "length a multiple of 3");

        this.points = points.clone();
        createShape();
    }

    /**
     * Instantiate a shape based on a flipped buffer containing coordinates. For
     * best performance and stability, the convex hull should have no more than
     * 100 vertices.
     *
     * @param flippedBuffer the coordinates on which to base the shape (not
     * null, limit&gt;0, limit a multiple of 3, unaffected)
     */
    public HullCollisionShape(FloatBuffer flippedBuffer) {
        Validate.nonNull(flippedBuffer, "flipped buffer");
        int numFloats = flippedBuffer.limit();
        Validate.positive(numFloats, "limit");
        Validate.require(numFloats % numAxes == 0, "limit a multiple of 3");

        this.points = new float[numFloats];
        for (int i = 0; i < numFloats; ++i) {
            points[i] = flippedBuffer.get(i);
        }

        createShape();
    }

    /**
     * Instantiate a shape based on a Vhacd4Hull. For best performance and
     * stability, the convex hull should have no more than 100 vertices.
     *
     * @param vhacd4Hull (not null, unaffected)
     */
    public HullCollisionShape(Vhacd4Hull vhacd4Hull) {
        Validate.nonNull(vhacd4Hull, "V-HACD hull");

        this.points = vhacd4Hull.clonePositions();
        createShape();
    }

    /**
     * Instantiate a shape based on a VHACDHull. For best performance and
     * stability, the convex hull should have no more than 100 vertices.
     *
     * @param vhacdHull (not null, unaffected)
     */
    public HullCollisionShape(VHACDHull vhacdHull) {
        Validate.nonNull(vhacdHull, "V-HACD hull");

        this.points = vhacdHull.clonePositions();
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Calculate a quick upper bound for the unscaled volume of the hull, based
     * on its axis-aligned bounding box.
     *
     * @return the volume (in unscaled shape units cubed, &ge;0)
     */
    public float aabbVolume() {
        Vector3f maxima = new Vector3f(Float.NEGATIVE_INFINITY,
                Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
        Vector3f minima = new Vector3f(Float.POSITIVE_INFINITY,
                Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);

        Vector3f location = new Vector3f();
        for (int floatI = 0; floatI < points.length; floatI += numAxes) {
            float x = points[floatI + PhysicsSpace.AXIS_X];
            float y = points[floatI + PhysicsSpace.AXIS_Y];
            float z = points[floatI + PhysicsSpace.AXIS_Z];
            location.set(x, y, z);
            MyVector3f.accumulateMinima(minima, location);
            MyVector3f.accumulateMaxima(maxima, location);
        }

        float dx = maxima.x - minima.x;
        float dy = maxima.y - minima.y;
        float dz = maxima.z - minima.z;
        float volume = dx * dy * dz;

        assert volume >= 0f : volume;
        assert MyMath.isFinite(volume) : volume;
        return volume;
    }

    /**
     * Copy the unscaled vertex locations of the optimized convex hull.
     *
     * @return a new array (not null)
     */
    public float[] copyHullVertices() {
        long shapeId = nativeId();
        int numHullVertices = countHullVertices();
        FloatBuffer buffer
                = BufferUtils.createFloatBuffer(numHullVertices * numAxes);
        getHullVerticesF(shapeId, buffer);

        float[] result = new float[numHullVertices * numAxes];
        for (int floatI = 0; floatI < numHullVertices * numAxes; ++floatI) {
            result[floatI] = buffer.get(floatI);
        }

        return result;
    }

    /**
     * Count the number of vertices in the optimized convex hull.
     *
     * @return the count (&ge;0)
     */
    public int countHullVertices() {
        long shapeId = nativeId();
        int result = countHullVertices(shapeId);

        return result;
    }

    /**
     * Count the vertices used to generate the hull.
     *
     * @return the count (&gt;0)
     */
    public int countMeshVertices() {
        int length = points.length;
        assert (length % numAxes == 0) : length;
        int result = length / numAxes;

        assert result > 0 : result;
        return result;
    }

    /**
     * Calculate the unscaled half extents of the hull.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.zero();
        for (int i = 0; i < points.length; i += numAxes) {
            float x = FastMath.abs(points[i + PhysicsSpace.AXIS_X]);
            if (x > result.x) {
                result.x = x;
            }
            float y = FastMath.abs(points[i + PhysicsSpace.AXIS_Y]);
            if (y > result.y) {
                result.y = y;
            }
            float z = FastMath.abs(points[i + PhysicsSpace.AXIS_Z]);
            if (z > result.z) {
                result.z = z;
            }
        }

        assert MyVector3f.isAllNonNegative(result) : result;
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Calculate how far this shape extends from its center, including margin.
     *
     * @return a distance (in physics-space units, &ge;0)
     */
    @Override
    public float maxRadius() {
        int numHullVertices = countHullVertices();
        FloatBuffer buffer
                = BufferUtils.createFloatBuffer(numHullVertices * numAxes);
        long shapeId = nativeId();
        getHullVerticesF(shapeId, buffer);
        double maxSquaredDistance = 0.0;

        for (int vertexI = 0; vertexI < numHullVertices; ++vertexI) {
            int startOffset = numAxes * vertexI;
            float x = scale.x * buffer.get(startOffset + PhysicsSpace.AXIS_X);
            float y = scale.y * buffer.get(startOffset + PhysicsSpace.AXIS_Y);
            float z = scale.z * buffer.get(startOffset + PhysicsSpace.AXIS_Z);
            double lengthSquared = MyMath.sumOfSquares(x, y, z);
            if (lengthSquared > maxSquaredDistance) {
                maxSquaredDistance = lengthSquared;
            }
        }
        float result = margin + (float) Math.sqrt(maxSquaredDistance);

        return result;
    }

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
        assert directBuffer == null : directBuffer;

        int numFloats = points.length;
        assert numFloats != 0;
        assert (numFloats % numAxes == 0) : numFloats;
        int numVertices = numFloats / numAxes;

        this.directBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (float f : points) {
            if (!MyMath.isFinite(f)) {
                throw new IllegalArgumentException("illegal coordinate: " + f);
            }
            directBuffer.put(f);
        }

        long shapeId = createShapeF(directBuffer, numVertices);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native private methods

    native private static int countHullVertices(long shapeId);

    native private static long createShapeF(
            FloatBuffer vertices, int numVertices);

    native private static void getHullVerticesF(
            long shapeId, FloatBuffer vertices);

    native private static void recalcAabb(long shapeId);
}
