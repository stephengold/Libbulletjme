/*
 * Copyright (c) 2019-2021 jMonkeyEngine
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
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A scalable mesh that combines multiple indexed meshes. Based on Bullet's
 * {@code btTriangleIndexVertexArray}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CompoundMesh extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CompoundMesh.class.getName());
    // *************************************************************************
    // fields

    /**
     * component meshes
     */
    final private ArrayList<IndexedMesh> submeshes = new ArrayList<>(4);
    /**
     * copy of scale factors: one for each local axis
     */
    final private Vector3f scale = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate an empty mesh.
     */
    public CompoundMesh() {
        createEmpty();
    }

    /**
     * Copy an existing mesh.
     *
     * @param original the mesh to copy (not null, unaffected)
     */
    public CompoundMesh(CompoundMesh original) {
        createEmpty();
        for (IndexedMesh submesh : original.submeshes) {
            add(submesh);
        }
        setScale(original.scale);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a submesh to this mesh.
     *
     * @param submesh (not null, alias created)
     */
    final public void add(IndexedMesh submesh) {
        Validate.nonNull(submesh, "submesh");

        submeshes.add(submesh);

        long compoundMeshId = nativeId();
        long submeshId = submesh.nativeId();
        addIndexedMesh(compoundMeshId, submeshId);
    }

    /**
     * Count how many triangles are in this mesh.
     *
     * @return the count (&ge;0)
     */
    public int countTriangles() {
        int numTriangles = 0;
        for (IndexedMesh submesh : submeshes) {
            numTriangles += submesh.countTriangles();
        }

        assert numTriangles >= 0 : numTriangles;
        return numTriangles;
    }

    /**
     * Count how many vertices are in this mesh.
     *
     * @return the count (&ge;0)
     */
    public int countVertices() {
        int numVertices = 0;
        for (IndexedMesh submesh : submeshes) {
            numVertices += submesh.countVertices();
        }

        assert numVertices >= 0 : numVertices;
        return numVertices;
    }

    /**
     * Copy the scale factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scale factor for each local axis (either storeResult or a new
     * vector, not null)
     */
    public Vector3f getScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert checkScale(result);
        result.set(scale);

        return result;
    }

    /**
     * Alter the scale factors.
     *
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public void setScale(Vector3f scale) {
        long compoundMeshId = nativeId();
        setScaling(compoundMeshId, scale.x, scale.y, scale.z);
        logger.log(Level.FINE, "Scaled {0}", this);
        this.scale.set(scale);
    }

    /**
     * Attempt to divide this mesh into 2 meshes.
     *
     * @param scaledTriangle to define the splitting plane (in scaled
     * coordinates, not null, unaffected)
     * @return a pair of meshes, the first mesh generated by the plane's minus
     * side and the 2nd mesh generated by its plus side; either mesh may be
     * null, indicating an empty mesh
     */
    public CompoundMesh[] split(Triangle scaledTriangle) {
        Validate.nonNull(scaledTriangle, "scaled triangle");

        // Apply inverse scaling to each vertex of the parent triangle.
        Triangle descaledTriangle = new Triangle();
        Vector3f tmpVector = new Vector3f();
        for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            Vector3f inputVector = scaledTriangle.get(vertexIndex); // alias
            tmpVector.set(inputVector);
            tmpVector.divideLocal(scale);
            descaledTriangle.set(vertexIndex, tmpVector);
        }

        Vector3f normal = descaledTriangle.getNormal(); // alias
        Vector3f location = descaledTriangle.get3(); // alias
        Plane splittingPlane = new Plane(normal, location);
        /*
         * Organize the submeshes into (up to) 2 new meshes, based on
         * which side(s) of the splitting plane they are on.
         */
        CompoundMesh[] result = new CompoundMesh[2];
        for (IndexedMesh submesh : submeshes) {
            IndexedMesh[] mp = submesh.split(splittingPlane);
            for (int sideI = 0; sideI < 2; ++sideI) {
                IndexedMesh newMesh = mp[sideI];
                if (newMesh != null) {
                    if (result[sideI] == null) {
                        result[sideI] = new CompoundMesh();
                    }
                    result[sideI].add(newMesh);
                }
            }
        }

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Compare Bullet's scale factors to the local copy.
     *
     * @param tempVector caller-allocated temporary storage (not null)
     * @return true if scale factors are exactly equal, otherwise false
     */
    private boolean checkScale(Vector3f tempVector) {
        assert tempVector != null;

        long compoundMeshId = nativeId();
        getScaling(compoundMeshId, tempVector);
        boolean result = scale.equals(tempVector);

        return result;
    }

    /**
     * Create an empty {@code btTriangleIndexVertexArray}.
     */
    private void createEmpty() {
        long compoundMeshId = createEmptyTiva();
        setNativeId(compoundMeshId);

        logger.log(Level.FINE, "Created {0}", this);
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param compoundMeshId the native identifier (not zero)
     */
    private static void freeNativeObject(long compoundMeshId) {
        assert compoundMeshId != 0L;
        finalizeNative(compoundMeshId);
    }
    // *************************************************************************
    // native private methods

    native private static void
            addIndexedMesh(long compoundMeshId, long submeshId);

    native private static long createEmptyTiva();

    native private static void finalizeNative(long compoundMeshId);

    native private static void
            getScaling(long compoundMeshId, Vector3f storeVector);

    native private static void setScaling(
            long compoundMeshId, float xScale, float yScale, float zScale);
}
