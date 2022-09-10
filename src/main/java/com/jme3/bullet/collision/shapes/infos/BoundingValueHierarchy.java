/*
 * Copyright (c) 2020-2021 jMonkeyEngine
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
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A Bounding-Value Hierarchy (BVH) generated for a MeshCollisionShape, based on
 * Bullet's {@code btOptimizedBvh}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BoundingValueHierarchy extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BoundingValueHierarchy.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a reference to the hierarchy of the specified
     * MeshCollisionShape. Used internally.
     *
     * @param meshShape the pre-existing shape (not null)
     */
    public BoundingValueHierarchy(MeshCollisionShape meshShape) {
        Validate.nonNull(meshShape, "mesh shape");

        long shapeId = meshShape.nativeId();
        long bvhId = getOptimizedBvh(shapeId);
        super.setNativeIdNotTracked(bvhId);
    }

    /**
     * Instantiate a hierarchy from serialized bytes.
     *
     * @param bytes the serialized bytes (not null, unaffected)
     */
    public BoundingValueHierarchy(byte[] bytes) {
        Validate.nonNull(bytes, "bytes");

        long bvhId = deSerialize(bytes);
        super.setNativeId(bvhId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Serialize this hierarchy to a byte array.
     *
     * @return a new array containing serialized bytes (not null)
     */
    public byte[] serialize() {
        long bvhId = nativeId();
        byte[] result = serialize(bvhId);

        assert result != null;
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param bvhId the native identifier (not zero)
     */
    private static void freeNativeObject(long bvhId) {
        assert bvhId != 0L;
        finalizeNative(bvhId);
    }
    // *************************************************************************
    // native private methods

    native private static long deSerialize(byte[] buffer);

    native private static void finalizeNative(long bvhId);

    native private static long getOptimizedBvh(long shapeId);

    native private static byte[] serialize(long bvhId);
}
