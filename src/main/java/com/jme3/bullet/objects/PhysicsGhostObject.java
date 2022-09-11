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
package com.jme3.bullet.objects;

import com.jme3.bullet.collision.PcoType;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * A collision object for intangibles, based on Bullet's
 * {@code btPairCachingGhostObject}. This is useful for creating a character
 * controller, collision sensors/triggers, explosions etc.
 * <p>
 * Overlap detection skips the narrow-phase collision-detection algorithm and
 * relies entirely on the broad-phase algorithm, which is AABB plus a margin of
 * about 0.06 world units. Precise collision detection is still available via
 * {@code PhysicsSpace.addCollisionListener()}.
 * <p>
 * <i>From Bullet manual:</i><br>
 * btGhostObject is a special btCollisionObject, useful for fast localized
 * collision queries.
 *
 * @author normenhansen
 */
public class PhysicsGhostObject extends PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsGhostObject.class.getName());
    // *************************************************************************
    // fields

    /**
     * TODO reused list
     */
    final private List<PhysicsCollisionObject> overlappingObjects
            = new LinkedList<>();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a ghost object with the specified CollisionShape. The new
     * object is not added to any CollisionSpace.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public PhysicsGhostObject(CollisionShape shape) {
        super.setCollisionShape(shape);
        buildObject();

        assert !isContactResponse();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access an overlapping collision object by its position in the list.
     * Important: {@link #getOverlappingObjects()} must be invoked first!
     *
     * @param index which list position (&ge;0, &lt;count)
     * @return the pre-existing object
     */
    public PhysicsCollisionObject getOverlapping(int index) {
        return overlappingObjects.get(index);
    }

    /**
     * Count how many collision objects this object overlaps.
     *
     * @return count (&ge;0)
     */
    public int getOverlappingCount() {
        long objectId = nativeId();
        int result = getOverlappingCount(objectId);

        return result;
    }

    /**
     * Access a list of overlapping objects.
     *
     * @return an internal list which may get reused (not null)
     */
    public List<PhysicsCollisionObject> getOverlappingObjects() {
        overlappingObjects.clear();

        long objectId = nativeId();
        getOverlappingObjects(objectId);

        return overlappingObjects;
    }

    /**
     * Directly alter the location of this object's center.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    public void setPhysicsLocation(Vector3f location) {
        long objectId = nativeId();
        setPhysicsLocation(objectId, location);
    }

    /**
     * Directly alter this object's orientation.
     *
     * @param rotation the desired orientation (a rotation matrix in
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Matrix3f rotation) {
        long objectId = nativeId();
        setPhysicsRotation(objectId, rotation);
    }

    /**
     * Directly alter this object's orientation.
     *
     * @param rotation the desired orientation (a rotation quaternion in
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Quaternion rotation) {
        long objectId = nativeId();
        setPhysicsRotation(objectId, rotation);
    }
    // *************************************************************************
    // PhysicsCollisionObject methods

    /**
     * Apply the specified CollisionShape to this object. Note that the object
     * should not be in any CollisionSpace while changing shape; the object gets
     * rebuilt on the physics side.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    @Override
    public void setCollisionShape(CollisionShape collisionShape) {
        super.setCollisionShape(collisionShape);
        buildObject();
    }
    // *************************************************************************
    // Java private methods

    /**
     * This method is invoked by native code.
     *
     * @param co the collision object to add (alias created)
     */
    private void addOverlappingObject_native(PhysicsCollisionObject co) {
        overlappingObjects.add(co);
    }

    /**
     * Create the configured object in Bullet.
     */
    private void buildObject() {
        if (!hasAssignedNativeObject()) {
            long objectId = createGhostObject();
            setNativeId(objectId);
            assert getInternalType(objectId) == PcoType.ghost :
                    getInternalType(objectId);
            logger2.log(Level.FINE, "Created {0}.", this);

            setGhostFlags(objectId);
            initUserPointer();
        }

        long objectId = nativeId();
        CollisionShape shape = getCollisionShape();
        attachCollisionShape(objectId, shape.nativeId());
    }
    // *************************************************************************
    // native private methods

    native private static long createGhostObject();

    native private static int getOverlappingCount(long objectId);

    native private void getOverlappingObjects(long objectId);

    native private static void setGhostFlags(long objectId);

    native private static void
            setPhysicsLocation(long objectId, Vector3f location);

    native private static void
            setPhysicsRotation(long objectId, Matrix3f rotation);

    native private static void
            setPhysicsRotation(long objectId, Quaternion rotation);
}
