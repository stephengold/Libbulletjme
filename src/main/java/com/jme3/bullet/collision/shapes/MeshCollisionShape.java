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

import com.jme3.bullet.collision.shapes.infos.CompoundMesh;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A mesh CollisionShape that uses a Bounding Value Hierarchy (BVH), based on
 * Bullet's btBvhTriangleMeshShape. Not for use in dynamic bodies. TODO add a
 * shape based on btScaledBvhTriangleMeshShape
 *
 * @author normenhansen
 */
public class MeshCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MeshCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * if true, use quantized AABB compression (default=true)
     */
    private boolean useCompression;
    /**
     * native mesh used to construct this shape
     */
    private CompoundMesh nativeMesh;
    /**
     * unique identifier of the native buffer that holds the BVH - TODO rename
     * bvhBufferId
     */
    private long nativeBVHBuffer = 0L;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a shape based on the specified native mesh(es).
     *
     * @param useCompression true to use quantized AABB compression
     * @param submeshes the mesh(es) on which to base the shape (not null, not
     * empty)
     */
    public MeshCollisionShape(boolean useCompression,
            IndexedMesh... submeshes) {
        Validate.nonEmpty(submeshes, "submeshes");

        this.useCompression = useCompression;
        nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : submeshes) {
            nativeMesh.add(submesh);
        }
        createShape(null);
    }

    /**
     * Instantiate a shape based on the specified native mesh(es) and serialized
     * BVH. The submeshes must be equivalent to those used to generate the BVH.
     *
     * @param bvhData the serialized BVH (not null)
     * @param submeshes the mesh(es) on which to base the shape (not null, not
     * empty)
     */
    public MeshCollisionShape(byte[] bvhData, IndexedMesh... submeshes) {
        Validate.nonNull(bvhData, "BVH data");
        Validate.nonEmpty(submeshes, "submeshes");

        useCompression = true;
        nativeMesh = new CompoundMesh();
        for (IndexedMesh submesh : submeshes) {
            nativeMesh.add(submesh);
        }
        createShape(bvhData);
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
     * Serialize the BVH to a byte array.
     *
     * @return a new array containing a serialized version of the BVH
     */
    public byte[] serializeBvh() {
        long shapeId = getObjectId();
        byte[] result = saveBVH(shapeId);
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Finalize this shape just before it is destroyed. Should be invoked only
     * by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        if (nativeBVHBuffer != 0L) {
            finalizeBVH(nativeBVHBuffer);
        }
    }

    /**
     * Recalculate this shape's bounding box if necessary.
     */
    @Override
    protected void recalculateAabb() {
        long shapeId = getObjectId();
        recalcAabb(shapeId);
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate the configured btBvhTriangleMeshShape.
     *
     * @param bvh serialized BVH, or null to build the BVH from scratch
     */
    private void createShape(byte bvh[]) {
        boolean buildBvh = (bvh == null || bvh.length == 0);
        long meshId = nativeMesh.nativeId();
        long shapeId = createShape(useCompression, buildBvh, meshId);
        setNativeId(shapeId);

        if (!buildBvh) {
            nativeBVHBuffer = setBVH(bvh, shapeId);
            assert nativeBVHBuffer != 0L;
        }

        setScale(scale);
        setMargin(margin);
    }
    // *************************************************************************
    // native methods

    native private long createShape(boolean useCompression, boolean buildBvh,
            long meshId);

    native private void finalizeBVH(long nativeBVHBufferId);

    native private void recalcAabb(long shapeId);

    native private byte[] saveBVH(long objectId);

    /**
     * Read the ID of the native buffer used by the in-place de-serialized
     * shape. The buffer should be explicitly freed when no longer needed.
     */
    native private long setBVH(byte[] buffer, long objectid);
}
