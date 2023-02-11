/*
 * Copyright (c) 2019-2023 jMonkeyEngine
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
package com.jme3.bullet.collision.shapes.infos;

import com.jme3.bullet.NativePhysicsObject;
import com.jme3.math.Plane;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.DistinctVectorValues;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;

/**
 * An indexed triangle mesh based on Bullet's {@code btIndexedMesh}. Immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class IndexedMesh extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * number of bytes in a float
     */
    final private static int floatBytes = 4;
    /**
     * number of bytes in an int
     */
    final private static int intBytes = 4;
    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of vertices per triangle
     */
    final private static int vpt = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(IndexedMesh.class.getName());
    // *************************************************************************
    // fields

    /**
     * configured position data: 3 floats per vertex (not null, direct, never
     * flipped)
     */
    final private FloatBuffer vertexPositions;
    /**
     * configured index data: 3 ints per triangle (not null, direct, never
     * flipped)
     */
    final private IntBuffer indices;
    /**
     * configured bytes per triangle in the index buffer (12)
     */
    final private int indexStride;
    /**
     * configured number of triangles in the mesh (&ge;0)
     */
    final private int numTriangles;
    /**
     * configured number of vertices in the mesh (&ge;0)
     */
    final private int numVertices;
    /**
     * configured bytes per vertex in the position buffer (12)
     */
    final private int vertexStride;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an IndexedMesh based on the specified positions and indices.
     *
     * @param positionArray (not null, unaffected)
     * @param indexArray (not null, unaffected, length a multiple of 3)
     */
    public IndexedMesh(Vector3f[] positionArray, int[] indexArray) {
        Validate.nonNull(positionArray, "position array");
        Validate.nonNull(indexArray, "index array");
        int numIndices = indexArray.length;
        Validate.require(numIndices % vpt == 0, "length a multiple of 3");

        this.numVertices = positionArray.length;
        this.vertexPositions = BufferUtils.createFloatBuffer(positionArray);
        this.vertexStride = numAxes * floatBytes;

        this.numTriangles = numIndices / vpt;
        this.indices = BufferUtils.createIntBuffer(indexArray);
        this.indexStride = vpt * intBytes;

        createMesh();
    }

    /**
     * Instantiate an IndexedMesh based on the specified vertex positions. An
     * index will be assigned to each distinct position.
     *
     * @param buffer the vertex positions of a non-indexed triangle mesh (not
     * null, flipped, limit a multiple of 9, unaffected)
     */
    public IndexedMesh(FloatBuffer buffer) {
        Validate.nonNull(buffer, "buffer");
        int numFloats = buffer.limit();
        Validate.require(numFloats % 9 == 0, "limit a multiple of 9");

        // Assign an index to each distinct vertex position.
        DistinctVectorValues dvv
                = new DistinctVectorValues(buffer, 0, numFloats);

        this.numVertices = dvv.countDistinct();
        this.vertexPositions
                = BufferUtils.createFloatBuffer(numAxes * numVertices);
        this.vertexStride = numAxes * floatBytes;

        int numIndices = numFloats / numAxes;
        this.numTriangles = numIndices / vpt;
        this.indices = BufferUtils.createIntBuffer(numIndices);
        this.indexStride = vpt * intBytes;

        Vector3f tmpVector = new Vector3f();
        for (int oldVi = 0; oldVi < numIndices; ++oldVi) {
            int newVi = dvv.findVvid(oldVi);
            assert newVi >= 0 : newVi;
            indices.put(oldVi, newVi);

            int readPosition = numAxes * oldVi;
            MyBuffer.get(buffer, readPosition, tmpVector);
            int writePosition = numAxes * newVi;
            MyBuffer.put(vertexPositions, writePosition, tmpVector);
            // Some vertex positions may be written multiple times!
        }

        createMesh();
    }

    /**
     * Instantiate an IndexedMesh based on the specified positions and indices.
     *
     * @param positionBuffer (not null, not flipped, length a multiple of 3,
     * alias created)
     * @param indexBuffer (not null, not flipped, length a multiple of 3, alias
     * created)
     */
    public IndexedMesh(FloatBuffer positionBuffer, IntBuffer indexBuffer) {
        Validate.nonNull(positionBuffer, "position buffer");
        Validate.nonNull(indexBuffer, "index buffer");
        int numFloats = positionBuffer.capacity();
        Validate.require(numFloats % numAxes == 0, "capacity a multiple of 3");
        int numIndices = indexBuffer.capacity();
        Validate.require(numIndices % vpt == 0, "capacity a multiple of 3");

        this.numVertices = numFloats / numAxes;
        this.vertexPositions = positionBuffer;
        this.vertexStride = numAxes * floatBytes;

        this.numTriangles = numIndices / vpt;
        this.indices = indexBuffer;
        this.indexStride = vpt * intBytes;

        createMesh();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the triangle indices.
     *
     * @return a new, direct, unflipped buffer
     */
    public IntBuffer copyIndices() {
        int numInts = indices.capacity();
        IntBuffer result = BufferUtils.createIntBuffer(numInts);
        for (int bufPos = 0; bufPos < numInts; ++bufPos) {
            int tmpIndex = indices.get(bufPos);
            result.put(tmpIndex);
        }

        return result;
    }

    /**
     * Copy the vertex positions of the specified triangle.
     *
     * @param triangleIndex the index of the source triangle (&ge;0)
     * @param destination storage for the result (not null, modified)
     */
    public void copyTriangle(int triangleIndex, Triangle destination) {
        Validate.inRange(triangleIndex, "triangle index", 0, numTriangles - 1);
        Validate.nonNull(destination, "destination");

        int startPosition = triangleIndex * vpt; // within the indices buffer
        Vector3f tmpVector = new Vector3f();
        for (int vertexI = 0; vertexI < vpt; ++vertexI) {
            int indexPosition = startPosition + vertexI;
            int vertexIndex = indices.get(indexPosition);
            MyBuffer.get(vertexPositions, vertexIndex * numAxes, tmpVector);
            destination.set(vertexI, tmpVector);
        }
    }

    /**
     * Copy the vertex positions to a new buffer.
     *
     * @return a new, direct, unflipped buffer
     */
    public FloatBuffer copyVertexPositions() {
        int numFloats = numVertices * numAxes;
        FloatBuffer result = BufferUtils.createFloatBuffer(numFloats);
        for (int bufPos = 0; bufPos < numFloats; ++bufPos) {
            float tmpFloat = vertexPositions.get(bufPos);
            result.put(tmpFloat);
        }

        return result;
    }

    /**
     * Count how many triangles are in this mesh.
     *
     * @return the count (&ge;0)
     */
    public int countTriangles() {
        assert numTriangles >= 0 : numTriangles;
        return numTriangles;
    }

    /**
     * Count how many vertices are in this mesh.
     *
     * @return the count (&ge;0)
     */
    public int countVertices() {
        assert numVertices >= 0 : numVertices;
        return numVertices;
    }

    /**
     * Find the maximum and minimum coordinates for each axis among the vertices
     * in this mesh.
     *
     * @param storeMaxima storage for the maxima (not null, modified)
     * @param storeMinima storage for the minima (not null, modified)
     */
    public void maxMin(Vector3f storeMaxima, Vector3f storeMinima) {
        Validate.nonNull(storeMaxima, "store maxima");
        Validate.nonNull(storeMinima, "store minima");

        int numFloats = numVertices * numAxes;
        MyBuffer.maxMin(
                vertexPositions, 0, numFloats, storeMaxima, storeMinima);
    }

    /**
     * Attempt to divide this mesh into 2 meshes.
     *
     * @param splittingPlane the splitting plane (not null, unaffected)
     * @return a pair of meshes, the first mesh generated by the plane's minus
     * side and the 2nd mesh generated by its plus side; either mesh may be
     * null, indicating an empty mesh
     */
    public IndexedMesh[] split(Plane splittingPlane) {
        Validate.nonNull(splittingPlane, "splitting plane");

        FloatBuffer[] buffers = new FloatBuffer[2];
        int cap = 2 * numTriangles * vpt * numAxes;
        buffers[0] = BufferUtils.createFloatBuffer(cap);
        buffers[1] = BufferUtils.createFloatBuffer(cap);
        /*
         * Split each triangle and write the resulting positions to the
         * new buffers, without concern for duplicates.
         */
        Triangle triangle = new Triangle();
        for (int triangleI = 0; triangleI < numTriangles; ++triangleI) {
            copyTriangle(triangleI, triangle);
            splitTriangle(triangle, splittingPlane, buffers);
        }

        IndexedMesh[] result = new IndexedMesh[2];
        int numMinus = buffers[0].position();
        int numPlus = buffers[1].position();
        if (numMinus == 0 || numPlus == 0) {
            // Degenerate case:  all triangles lie to one side of the plane.
            if (numMinus > 0) {
                result[0] = this;
            } else if (numPlus > 0) {
                result[1] = this;
            }

        } else {
            // Convert each buffer to a new mesh.
            for (int sideI = 0; sideI < 2; ++sideI) {
                buffers[sideI].flip();
                result[sideI] = new IndexedMesh(buffers[sideI]);
            }
        }

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create a {@code btIndexedMesh} using the current configuration.
     */
    private void createMesh() {
        assert vertexStride == 12 : vertexStride;
        assert indexStride == 12 : indexStride;

        long meshId = createInt(indices, vertexPositions, numTriangles,
                numVertices, vertexStride, indexStride);
        setNativeId(meshId);

        logger.log(Level.FINE, "Created {0}", this);
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param meshId the native identifier (not zero)
     */
    private static void freeNativeObject(long meshId) {
        assert meshId != 0L;
        finalizeNative(meshId);
    }

    /**
     * Write the specified triangle, but only if all 3 vertices are distinct.
     *
     * @param buffer the buffer to write to (not null)
     * @param v1 the first vertex to write (not null, unaffected)
     * @param v2 the 2nd vertex to write (not null, unaffected)
     * @param v3 the 3rd vertex to write (not null, unaffected)
     */
    private static void putTriangle(
            FloatBuffer buffer, Vector3f v1, Vector3f v2, Vector3f v3) {
        if (v1.equals(v2) || v1.equals(v3) || v2.equals(v3)) {
            // The triangle is obviously degenerate.  Skip it.
            return;
        }

        buffer.put(v1.x).put(v1.y).put(v1.z);
        buffer.put(v2.x).put(v2.y).put(v2.z);
        buffer.put(v3.x).put(v3.y).put(v3.z);
    }

    /**
     * Attempt to divide the specified triangle into (up to) 3 triangles.
     *
     * @param input the triangle to be split (not null, unaffected)
     * @param splittingPlane the splitting plane (not null, unaffected)
     * @param putResults the buffers to write the triangles
     */
    private static void splitTriangle(
            Triangle input, Plane splittingPlane, FloatBuffer[] putResults) {
        Vector3f a = input.get1(); // alias
        Vector3f b = input.get2(); // alias
        Vector3f c = input.get3(); // alias
        float aa = splittingPlane.pseudoDistance(a);
        float bb = splittingPlane.pseudoDistance(b);
        float cc = splittingPlane.pseudoDistance(c);

        if (aa == 0f && bb == 0f && cc == 0f) {
            // both sides
            putTriangle(putResults[0], a, b, c);
            putTriangle(putResults[1], a, b, c);

        } else if (aa <= 0f && bb <= 0f && cc <= 0f) {
            // negative side
            putTriangle(putResults[0], a, b, c);

        } else if (aa >= 0f && bb >= 0f && cc >= 0f) {
            // positive side
            putTriangle(putResults[1], a, b, c);

        } else if (aa >= 0f && bb <= 0f && cc <= 0f) {
            // A on the positive side, split AB and AC.
            float tab = aa / (aa - bb);
            Vector3f ab = MyVector3f.lerp(tab, a, b, null);
            float tac = aa / (aa - cc);
            Vector3f ac = MyVector3f.lerp(tac, a, c, null);
            putTriangle(putResults[0], b, c, ac);
            putTriangle(putResults[0], b, ac, ab);
            putTriangle(putResults[1], a, ab, ac);

        } else if (aa <= 0f && bb >= 0f && cc >= 0f) {
            // A on the negative side, split AB and AC.
            float tab = -aa / (bb - aa);
            Vector3f ab = MyVector3f.lerp(tab, a, b, null);
            float tac = -aa / (cc - aa);
            Vector3f ac = MyVector3f.lerp(tac, a, c, null);
            putTriangle(putResults[1], b, c, ac);
            putTriangle(putResults[1], b, ac, ab);
            putTriangle(putResults[0], a, ab, ac);

        } else if (aa <= 0f && bb >= 0f && cc <= 0f) {
            // B on the positive side, split AB and BC.
            float tab = -aa / (bb - aa);
            Vector3f ab = MyVector3f.lerp(tab, a, b, null);
            float tbc = bb / (bb - cc);
            Vector3f bc = MyVector3f.lerp(tbc, b, c, null);
            putTriangle(putResults[0], a, bc, c);
            putTriangle(putResults[0], a, ab, bc);
            putTriangle(putResults[1], b, bc, ab);

        } else if (aa >= 0f && bb <= 0f && cc >= 0f) {
            // B on the negative side, split AB and BC.
            float tab = aa / (aa - bb);
            Vector3f ab = MyVector3f.lerp(tab, a, b, null);
            float tbc = -bb / (cc - bb);
            Vector3f bc = MyVector3f.lerp(tbc, b, c, null);
            putTriangle(putResults[1], a, bc, c);
            putTriangle(putResults[1], a, ab, bc);
            putTriangle(putResults[0], b, bc, ab);

        } else if (aa <= 0f && bb <= 0f && cc >= 0f) {
            // C on the positive side, split AC and BC.
            float tac = -aa / (cc - aa);
            Vector3f ac = MyVector3f.lerp(tac, a, c, null);
            float tbc = -bb / (cc - bb);
            Vector3f bc = MyVector3f.lerp(tbc, b, c, null);
            putTriangle(putResults[0], a, b, bc);
            putTriangle(putResults[0], a, bc, ac);
            putTriangle(putResults[1], ac, bc, c);

        } else {
            assert aa >= 0f : aa;
            assert bb >= 0f : bb;
            assert cc <= 0f : cc;
            // C on the negative side, split AC and BC.
            float tac = aa / (aa - cc);
            Vector3f ac = MyVector3f.lerp(tac, a, c, null);
            float tbc = bb / (bb - cc);
            Vector3f bc = MyVector3f.lerp(tbc, b, c, null);
            putTriangle(putResults[1], a, b, bc);
            putTriangle(putResults[1], a, bc, ac);
            putTriangle(putResults[0], ac, bc, c);
        }
    }
    // *************************************************************************
    // native private methods

    native private static long createByte(ByteBuffer indices,
            FloatBuffer vertexPositions, int numTriangles, int numVertices,
            int vertexStride, int indexStride);

    native private static long createInt(IntBuffer indices,
            FloatBuffer vertexPositions, int numTriangles, int numVertices,
            int vertexStride, int indexStride);

    native private static long createShort(ShortBuffer indices,
            FloatBuffer vertexPositions, int numTriangles, int numVertices,
            int vertexStride, int indexStride);

    native private static void finalizeNative(long meshId);
}
