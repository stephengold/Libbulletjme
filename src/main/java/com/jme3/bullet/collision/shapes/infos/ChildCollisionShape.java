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
package com.jme3.bullet.collision.shapes.infos;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An element of a CompoundCollisionShape, consisting of a (non-compound) child
 * shape, offset and rotated with respect to its parent.
 *
 * @author normenhansen
 */
public class ChildCollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ChildCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * base shape (not null, not a compound shape)
     */
    private CollisionShape shape;
    /**
     * copy of rotation in the parent's coordinate system (not null)
     */
    private Matrix3f rotation;
    /**
     * copy of translation in the parent's coordinate system (not null)
     */
    private Vector3f offset;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a child shape for use in a compound shape.
     *
     * @param offset the desired translation in the parent's coordinate system
     * (not null, unaffected)
     * @param rotation the desired rotation in the parent's coordinate system
     * (not null, unaffected)
     * @param shape the base shape (not null, not a compound shape, alias
     * created)
     */
    public ChildCollisionShape(Vector3f offset, Matrix3f rotation,
            CollisionShape shape) {
        Validate.nonNull(shape, "shape");
        if (shape instanceof CompoundCollisionShape) {
            throw new IllegalArgumentException(
                    "CompoundCollisionShapes cannot be child shapes!");
        }

        this.offset = offset.clone();
        this.rotation = rotation.clone();
        this.shape = shape;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the translation in the parent's coordinate system.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a translation vector (either storeResult or a new vector, not
     * null)
     */
    public Vector3f copyOffset(Vector3f storeResult) {
        if (storeResult == null) {
            return offset.clone();
        } else {
            return storeResult.set(offset);
        }
    }

    /**
     * Copy the rotation in the parent's coordinate system.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a Quaternion (either storeResult or a new Quaternion, not null)
     */
    public Quaternion copyRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;
        result.fromRotationMatrix(rotation);
        return result;
    }

    /**
     * Copy the rotation in the parent's coordinate system.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (either storeResult or a new matrix, not null)
     */
    public Matrix3f copyRotationMatrix(Matrix3f storeResult) {
        if (storeResult == null) {
            return rotation.clone();
        } else {
            return storeResult.set(rotation);
        }
    }

    /**
     * Copy the Transform relative to the parent's coordinate system.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a Transform with scale=1 (either storeResult or a new instance,
     * not null)
     */
    public Transform copyTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        result.setTranslation(offset);
        result.getRotation().fromRotationMatrix(rotation);
        result.setScale(1f);

        return result;
    }

    /**
     * Access the base shape.
     *
     * @return the pre-existing shape (not null)
     */
    public CollisionShape getShape() {
        assert shape != null;
        return shape;
    }

    /**
     * Alter the child's coordinate transform copy. For internal use only.
     *
     * @param offset the desired translation relative to the parent (not null,
     * unaffected)
     * @param rotation the desired rotation relative to the parent (not null,
     * unaffected)
     * @see com.jme3.bullet.collision.shapes.CompoundCollisionShape
     * #setChildTransform(com.jme3.bullet.collision.shapes.CollisionShape,
     * com.jme3.math.Transform)
     */
    public void setTransform(Vector3f offset, Matrix3f rotation) {
        this.offset.set(offset);
        this.rotation.set(rotation);
    }
}
