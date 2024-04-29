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

import com.jme3.bullet.collision.shapes.infos.BoundingValueHierarchy;
import com.jme3.bullet.collision.shapes.infos.CompoundMesh;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A mesh collision shape that uses a Bounding Value Hierarchy (BVH), based on
 * Bullet's {@code btBvhTriangleMeshShape}. Not for use in dynamic bodies.
 * Collisions between {@code HeightfieldCollisionShape},
 * {@code MeshCollisionShape}, and {@code PlaneCollisionShape} objects are never
 * detected.
 * <p>
 * TODO add a shape based on {@code btScaledBvhTriangleMeshShape}
 *
 * @author normenhansen
 */
public class MeshCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * maximum number of submeshes when compression is used
     */
    final public static int maxSubmeshes = 1_024;
    /**
     * maximum number of triangles in any submesh when compression is used
     */
    final public static int maxTrianglesInAnySubmesh = 2_097_151;
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MeshCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * if true, use quantized AABB compression
     */
    final private boolean useCompression;
    /**
     * bounding-value hierarchy
     */
    private BoundingValueHierarchy bvh;
    /**
     * native mesh used to construct the shape
     */
    final private CompoundMesh nativeMesh;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a shape from the specified collection of native meshes.
     *
     * @param useCompression true to use quantized AABB compression
     * @param meshes the collection on which to base the shape (must contain at
     * least one triangle)
     */
    public MeshCollisionShape(
            boolean useCompression, Collection<IndexedMesh> meshes) {
        Validate.nonEmpty(meshes, "meshes");
        this.nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : meshes) {
            nativeMesh.add(submesh);
        }
        Validate.require(
                nativeMesh.countTriangles() > 0, "at least one triangle");

        this.useCompression = useCompression;
        createShape();
    }

    /**
     * Instantiate a shape based on the specified CompoundMesh.
     *
     * @param useCompression true to use quantized AABB compression
     * @param mesh the mesh on which to base the shape (not null, must contain
     * at least one triangle, unaffected)
     */
    public MeshCollisionShape(boolean useCompression, CompoundMesh mesh) {
        Validate.require(mesh.countTriangles() > 0, "at least one triangle");

        this.nativeMesh = new CompoundMesh(mesh);
        this.useCompression = useCompression;
        createShape();
    }

    /**
     * Instantiate a shape from the specified native mesh(es).
     *
     * @param useCompression true to use quantized AABB compression
     * @param submeshes the mesh(es) on which to base the shape (must contain at
     * least one triangle)
     */
    public MeshCollisionShape(
            boolean useCompression, IndexedMesh... submeshes) {
        Validate.nonEmpty(submeshes, "submeshes");
        this.nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : submeshes) {
            nativeMesh.add(submesh);
        }
        Validate.require(
                nativeMesh.countTriangles() > 0, "at least one triangle");

        this.useCompression = useCompression;
        createShape();
    }

    /**
     * Instantiate a shape from the specified native mesh(es) and serialized
     * BVH. The submeshes must be equivalent to those used to generate the BVH.
     *
     * @param bvhBytes the serialized BVH (not null, unaffected)
     * @param submeshes the mesh(es) on which to base the shape (must contain at
     * least one triangle)
     */
    public MeshCollisionShape(byte[] bvhBytes, IndexedMesh... submeshes) {
        Validate.nonNull(bvhBytes, "BVH data");
        Validate.nonEmpty(submeshes, "submeshes");
        this.nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : submeshes) {
            nativeMesh.add(submesh);
        }
        Validate.require(
                nativeMesh.countTriangles() > 0, "at least one triangle");

        this.useCompression = true;
        this.bvh = new BoundingValueHierarchy(bvhBytes);
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many triangles are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshTriangles() {
        int result = nativeMesh.countTriangles();
        return result;
    }

    /**
     * Count how many vertices are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshVertices() {
        int numVertices = nativeMesh.countVertices();
        return numVertices;
    }

    /**
     * Count how many submeshes are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countSubmeshes() {
        int result = nativeMesh.countSubmeshes();
        return result;
    }

    /**
     * Access the bounding-value hierarchy.
     *
     * @return the pre-existing instance (not null)
     */
    public BoundingValueHierarchy getBvh() {
        return bvh;
    }

    /**
     * Access the specified submesh.
     *
     * @param index the index of the desired submesh (in the order the submeshes
     * were added, &ge;0)
     * @return the pre-existing instance (not null)
     */
    public IndexedMesh getSubmesh(int index) {
        int numSubmeshes = nativeMesh.countSubmeshes();
        Validate.inRange(index, "submesh index", 0, numSubmeshes - 1);

        IndexedMesh result = nativeMesh.getSubmesh(index);
        return result;
    }

    /**
     * Serialize the BVH to a byte array.
     *
     * @return a new array containing a serialized version of the BVH
     */
    public byte[] serializeBvh() {
        byte[] result = bvh.serialize();
        return result;
    }

    /**
     * Attempt to divide this shape into 2 shapes.
     *
     * @param splittingTriangle to define the splitting plane (in shape
     * coordinates, not null, unaffected)
     * @return a pair of mesh shapes, the first shape generated by the plane's
     * minus side and the 2nd shape generated by its plus side; either shape may
     * be null, indicating an empty shape
     */
    public MeshCollisionShape[] split(Triangle splittingTriangle) {
        Validate.nonNull(splittingTriangle, "splitting triangle");

        CompoundMesh[] mp = nativeMesh.split(splittingTriangle);
        MeshCollisionShape[] result = new MeshCollisionShape[2];
        int numMinus = (mp[0] == null) ? 0 : mp[0].countTriangles();
        int numPlus = (mp[1] == null) ? 0 : mp[1].countTriangles();
        if (numMinus == 0 || numPlus == 0) {
            // Degenerate case:  all triangles lie to one side of the plane.
            if (numMinus > 0) {
                result[0] = this;
            } else if (numPlus > 0) {
                result[1] = this;
            }

        } else {
            result[0] = new MeshCollisionShape(useCompression, mp[0]);
            result[0].setScale(scale);

            result[1] = new MeshCollisionShape(useCompression, mp[1]);
            result[1].setScale(scale);
        }

        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether this shape can be split by an arbitrary plane.
     *
     * @return true
     */
    @Override
    public boolean canSplit() {
        return true;
    }

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long shapeId = nativeId();
        recalcAabb(shapeId);
    }

    /**
     * Alter the scale of this shape.
     * <p>
     * Note that if shapes are shared (between collision objects and/or compound
     * shapes) changes can have unintended consequences.
     *
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    @Override
    public void setScale(Vector3f scale) {
        super.setScale(scale);

        long shapeId = nativeId();
        if (hasBvh(shapeId)) {
            // Since super.setScale() might've caused a rebuild of the BVH:
            this.bvh = new BoundingValueHierarchy(this);
        }
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code btBvhTriangleMeshShape}.
     */
    private void createShape() {
        int numTriangles = nativeMesh.countTriangles();
        assert numTriangles > 0 : numTriangles;

        if (useCompression) {
            int numSubmeshes = nativeMesh.countSubmeshes();
            if (numSubmeshes > maxSubmeshes) {
                throw new IllegalArgumentException(
                        "Too many submeshes: " + numSubmeshes);
            }
            for (int submeshI = 0; submeshI < numSubmeshes; ++submeshI) {
                IndexedMesh submesh = nativeMesh.getSubmesh(submeshI);
                int count = submesh.countTriangles();
                if (count > maxTrianglesInAnySubmesh) {
                    throw new IllegalArgumentException(
                            "Submesh has too many triangles: " + count);
                }
            }
        }
        /*
         * Even if (bvh == null), building the BVH in createShape() is
         * potentially wasteful, since then setScale() might trigger a rebuild.
         */
        boolean buildBvh = false;
        long meshId = nativeMesh.nativeId();
        long shapeId = createShape(useCompression, buildBvh, meshId);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        setMargin(margin);

        assert !hasBvh(shapeId);
        if (bvh == null) { // Create an untracked native object:
            this.bvh = new BoundingValueHierarchy(this);
        } else {
            long bvhId = bvh.nativeId();
            setOptimizedBvh(shapeId, bvhId, scale);
        }
        assert hasBvh(shapeId);
        assert bvh.isCompressed() == useCompression :
                bvh.isCompressed() + " != " + useCompression;
    }
    // *************************************************************************
    // native private methods

    native private static long
            createShape(boolean useCompression, boolean buildBvh, long meshId);

    native private static boolean hasBvh(long shapeId);

    native private static void recalcAabb(long shapeId);

    native private static void setOptimizedBvh(
            long shapeId, long bvhId, Vector3f scaleVector);
}
