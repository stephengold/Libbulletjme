/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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

import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.IntPair;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;

/**
 * A utility class for interfacing with Native Bullet, specifically for soft
 * bodies.
 *
 * @author dokthar
 */
public class NativeSoftBodyUtil {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * number of vertices per edge
     */
    final private static int vpe = 2;
    /**
     * number of vertices per triangle
     */
    final private static int vpt = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NativeSoftBodyUtil.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private NativeSoftBodyUtil() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add the triangles and unique edges in the specified native mesh to the
     * specified soft body.
     *
     * @param mesh the input native mesh (not null)
     * @param softBody the soft body to which faces and links will be added (not
     * null, modified)
     */
    public static void appendFromNativeMesh(IndexedMesh mesh,
            PhysicsSoftBody softBody) {
        Validate.nonNull(softBody, "soft body");

        FloatBuffer positions = mesh.copyVertexPositions();
        assert positions.isDirect();
        softBody.appendNodes(positions);

        IntBuffer triangleIndices = mesh.copyIndices();
        assert triangleIndices.isDirect();
        softBody.appendFaces(triangleIndices);
        /*
         * Enumerate all unique edges among the triangles.
         */
        int size = triangleIndices.capacity();
        Set<IntPair> uniqueEdges = new HashSet<>(vpt * size);
        for (int intOffset = 0; intOffset < size; intOffset += vpt) {
            int ti0 = triangleIndices.get(intOffset);
            int ti1 = triangleIndices.get(intOffset + 1);
            int ti2 = triangleIndices.get(intOffset + 2);

            uniqueEdges.add(new IntPair(ti0, ti1));
            uniqueEdges.add(new IntPair(ti1, ti2));
            uniqueEdges.add(new IntPair(ti0, ti2));
        }

        int numUniqueEdges = uniqueEdges.size();
        int indexCount = vpe * numUniqueEdges;
        IntBuffer links = BufferUtils.createIntBuffer(indexCount);
        int edgeIndex = 0;
        for (IntPair edge : uniqueEdges) {
            links.put(edgeIndex, edge.smaller());
            links.put(edgeIndex + 1, edge.larger());
            edgeIndex += vpe;
        }
        softBody.appendLinks(links);
    }

    /**
     * Create an index map to merge any mesh vertices that share the same
     * position.
     *
     * @param positionBuffer the buffer of mesh-vertex positions (not null,
     * limit a multiple of 3, unaffected)
     * @return a new index map (not null)
     */
    public static IntBuffer generateIndexMap(FloatBuffer positionBuffer) {
        int numFloats = positionBuffer.limit();
        Validate.require(numFloats % numAxes == 0, "limit a multiple of 3");
        int numVertices = numFloats / numAxes;

        IntBuffer result = BufferUtils.createIntBuffer(numVertices);
        Map<Vector3f, Integer> tmpHashMap = new HashMap<>(numVertices);
        int nextMappedIndex = 0;

        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            Vector3f position = new Vector3f();
            MyBuffer.get(positionBuffer, numAxes * vertexIndex, position);
            MyVector3f.standardize(position, position);

            if (!tmpHashMap.containsKey(position)) {
                tmpHashMap.put(position, nextMappedIndex);
                result.put(nextMappedIndex);
                ++nextMappedIndex;
            } else {
                int mappedIndex = tmpHashMap.get(position);
                result.put(mappedIndex);
            }
        }
        result.flip();

        return result;
    }

    /**
     * Copy all vertex data in the specified input buffer, using the specified
     * map buffer to map vertex indices.
     *
     * @param indexMap the buffer to use to map input indices to result indices
     * (not null, unaffected)
     * @param inputBuffer the input buffer to map (not null, length a multiple
     * of numFloatsPerVertex, unaffected)
     * @param numFloatsPerVertex the number of float components per vertex
     * (&gt;0)
     * @return a new buffer containing mapped vertex data
     */
    public static FloatBuffer mapVertexData(IntBuffer indexMap,
            FloatBuffer inputBuffer, int numFloatsPerVertex) {
        Validate.nonNull(indexMap, "index map");
        Validate.positive(numFloatsPerVertex, "number of floats per vertex");
        int numFloats = inputBuffer.limit();
        assert (numFloats % numFloatsPerVertex == 0) : numFloats;
        int numVertices = numFloats / numFloatsPerVertex;

        FloatBuffer result = BufferUtils.createFloatBuffer(numFloats);

        int lastNewVIndex = -1;
        for (int oldVIndex = 0; oldVIndex < numVertices; ++oldVIndex) {
            int newVIndex = indexMap.get(oldVIndex);
            for (int i = 0; i < numFloatsPerVertex; ++i) {
                int oldFloatIndex = numFloatsPerVertex * oldVIndex + i;
                float x = inputBuffer.get(oldFloatIndex);

                int newFloatIndex = numFloatsPerVertex * newVIndex + i;
                result.put(newFloatIndex, x);
            }
            if (newVIndex > lastNewVIndex) {
                lastNewVIndex = newVIndex;
            }
        }

        int newLimit = numFloatsPerVertex * (lastNewVIndex + 1);
        result.limit(newLimit);

        return result;
    }
    // *************************************************************************
    // native methods

    native private static void updateClusterMesh(long softBodyId,
            FloatBuffer outPositionBuffer, boolean meshInLocalSpace);

    native private static void updateMesh(long softBodyId,
            IntBuffer inIndexMapping, FloatBuffer outPositionBuffer,
            FloatBuffer outNormalBuffer, boolean meshInLocalSpace,
            boolean updateNormals);

    native private static void updateMesh(long softBodyId,
            FloatBuffer outPositionBuffer, FloatBuffer outNormalBuffer,
            boolean meshInLocalSpace, boolean updateNormals);

    native private static void updatePinMesh(long softBodyId,
            FloatBuffer outPositionBuffer, boolean meshInLocalSpace);
}
