/*
 * Copyright (c) 2020-2022 jMonkeyEngine
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
package com.jme3.bullet.objects;

import com.jme3.bullet.MultiBody;
import com.jme3.bullet.collision.PcoType;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.simsilica.mathd.Matrix3d;
import com.simsilica.mathd.Vec3d;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision object for a link or base in a MultiBody, based on Bullet's
 * btMultiBodyLinkCollider.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiBodyCollider extends PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MultiBodyCollider.class.getName());
    // *************************************************************************
    // fields

    /**
     * index of the link (&ge;0) or -1 for the base (of the MultiBody)
     */
    final private int linkIndex;
    /**
     * MultiBody that contains this collider (not null)
     */
    final private MultiBody multiBody;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collider for the indexed link in the specified MultiBody.
     * Used internally.
     *
     * @param multiBody (not null, alias created)
     * @param linkIndex the link index, or -1 for the base
     */
    public MultiBodyCollider(MultiBody multiBody, int linkIndex) {
        Validate.nonNull(multiBody, "multibody");
        Validate.inRange(linkIndex, "link index", -1, Integer.MAX_VALUE);

        this.multiBody = multiBody;
        this.linkIndex = linkIndex;

        assert !hasAssignedNativeObject();
        buildObject();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Attach the specified collision shape to this collider. Used internally.
     *
     * @param shape the shape to attach (not null, alias created)
     */
    public void attachShape(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        setCollisionShape(shape);

        long objectId = nativeId();
        long shapeId = shape.nativeId();
        attachCollisionShape(objectId, shapeId);
    }

    /**
     * Access the MultiBody that contains this collider.
     *
     * @return the pre-existing instance (not null)
     */
    public MultiBody getMultiBody() {
        assert multiBody != null;
        return multiBody;
    }

    /**
     * Determine the index of the corresponding MultiBodyLink.
     *
     * @return the index (&ge;0) or -1 if this is the base collider
     */
    public int linkIndex() {
        assert linkIndex >= -1 : linkIndex;
        return linkIndex;
    }

    /**
     * Determine the mass of this collider.
     *
     * @return the mass (&gt;0)
     */
    public float mass() {
        float result;
        if (linkIndex >= 0) {
            result = multiBody.getLink(linkIndex).mass();
        } else {
            result = multiBody.baseMass();
        }

        assert result > 0f : result;
        return result;
    }

    /**
     * Directly alter the location of this collider's center.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, finite, unaffected)
     */
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");

        long objectId = nativeId();
        setPhysicsLocation(objectId, location);
    }

    /**
     * Directly alter the location of this collider's center.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, finite, unaffected)
     */
    public void setPhysicsLocationDp(Vec3d location) {
        Validate.finite(location, "location");

        long objectId = nativeId();
        setPhysicsLocationDp(objectId, location);
    }

    /**
     * Directly alter this collider's orientation.
     *
     * @param orientation the desired orientation (a rotation matrix in
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Matrix3f orientation) {
        Validate.nonNull(orientation, "orientation");

        long objectId = nativeId();
        setPhysicsRotation(objectId, orientation);
    }

    /**
     * Directly alter this collider's orientation.
     *
     * @param orientation the desired orientation (a rotation matrix in
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotationDp(Matrix3d orientation) {
        Validate.nonNull(orientation, "orientation");

        long objectId = nativeId();
        setPhysicsRotationDp(objectId, orientation);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured collider in Bullet.
     */
    private void buildObject() {
        long multiBodyId = multiBody.nativeId();
        long objectId = createCollider(multiBodyId, linkIndex);
        setNativeId(objectId);
        assert getInternalType(objectId) == PcoType.collider :
                getInternalType(objectId);
        logger2.log(Level.FINE, "Created {0}.", this);

        super.initUserPointer();
    }
    // *************************************************************************
    // native private methods

    native private static long createCollider(long multiBodyId, int linkIndex);

    native private static void
            setPhysicsLocation(long colliderId, Vector3f locationVector);

    native private static void
            setPhysicsLocationDp(long colliderId, Vec3d locationVector);

    native private static void
            setPhysicsRotation(long colliderId, Matrix3f rotationMatrix);

    native private static void
            setPhysicsRotationDp(long colliderId, Matrix3d rotationMatrix);
}
