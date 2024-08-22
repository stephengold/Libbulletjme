/*
 * Copyright (c) 2020-2024 jMonkeyEngine
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
package com.jme3.bullet;

import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.MultiBodyCollider;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An articulated rigid body based on Bullet's {@code btMultiBody}. Uses
 * Featherstone's algorithm.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiBody extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MultiBody.class.getName());
    // *************************************************************************
    // fields

    /**
     * number of links that have been configured (&ge;0)
     */
    private int numConfigured = 0;
    /**
     * collider for the base, or null if none
     */
    private MultiBodyCollider baseCollider = null;
    /**
     * references to configured links
     */
    final private MultiBodyLink[] links;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a MultiBody. Note that "sleep" is synonym for "deactivate".
     *
     * @param numLinks the desired number of links, not including the base
     * (&ge;0)
     * @param baseMass the desired mass of the base (in physics-space units,
     * &gt;0)
     * @param baseInertia the desired rotational inertia of the base (not null,
     * all elements positive)
     * @param fixedBase true &rarr; base is fixed, false &rarr; base can move
     * @param canSleep true &rarr; can sleep, false &rarr; won't sleep
     */
    public MultiBody(int numLinks, float baseMass, Vector3f baseInertia,
            boolean fixedBase, boolean canSleep) {
        Validate.nonNegative(numLinks, "number of links");
        Validate.positive(baseMass, "base mass");
        Validate.positive(baseInertia, "base inertia");

        long nativeId
                = create(numLinks, baseMass, baseInertia, fixedBase, canSleep);
        super.setNativeId(nativeId);
        assert getNumLinks(nativeId) == numLinks;
        finalizeMultiDof(nativeId);

        this.links = new MultiBodyLink[numLinks];
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a collider for the base.
     *
     * @param shape (not null, alias created)
     * @return a new collider
     */
    public MultiBodyCollider addBaseCollider(CollisionShape shape) {
        Validate.nonNull(shape, "shape");
        assert baseCollider == null : baseCollider;

        baseCollider = new MultiBodyCollider(this, -1);
        long multiBodyId = nativeId();
        long colliderId = baseCollider.nativeId();
        setBaseCollider(multiBodyId, colliderId);

        baseCollider.attachShape(shape);
        assert getBaseCollider(multiBodyId) == colliderId;

        return baseCollider;
    }

    /**
     * Add an external force to the base.
     *
     * @param force the force to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addBaseForce(Vector3f force) {
        Validate.finite(force, "force");

        long multiBodyId = nativeId();
        addBaseForce(multiBodyId, force);
    }

    /**
     * Add an external torque to the base.
     *
     * @param torque the torque to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addBaseTorque(Vector3f torque) {
        Validate.finite(torque, "torque");

        long multiBodyId = nativeId();
        addBaseTorque(multiBodyId, torque);
    }

    /**
     * Determine the angular damping.
     *
     * @return the damping
     */
    public float angularDamping() {
        long multiBodyId = nativeId();
        float result = getAngularDamping(multiBodyId);

        return result;
    }

    /**
     * Determine the angular velocity of the base.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the angular-velocity vector (either storeResult or a new vector,
     * not null)
     */
    public Vector3f baseAngularVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long multiBodyId = nativeId();
        getBaseOmega(multiBodyId, result);

        return result;
    }

    /**
     * Determine the net force on the base.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the force vector (either storeResult or a new vector, not null)
     */
    public Vector3f baseForce(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long multiBodyId = nativeId();
        getBaseForce(multiBodyId, result);

        return result;
    }

    /**
     * Determine the rotational inertia of the base.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the principal (diagonal) components of the inertia tensor (in the
     * base's local coordinates, either storeResult or a new vector, not null)
     */
    public Vector3f baseInertia(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long multiBodyId = nativeId();
        getBaseInertia(multiBodyId, result);

        return result;
    }

    /**
     * Determine the location of the base's center of mass.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f baseLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long multiBodyId = nativeId();
        getBasePos(multiBodyId, result);

        return result;
    }

    /**
     * Determine the mass of the base.
     *
     * @return the mass (in physics-space units, &gt;0)
     */
    public float baseMass() {
        long multiBodyId = nativeId();
        float result = getBaseMass(multiBodyId);

        return result;
    }

    /**
     * Determine the orientation of the base.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the orientation (either storeResult or a new instance, not null)
     */
    public Quaternion baseOrientation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        long multiBodyId = nativeId();
        getWorldToBaseRot(multiBodyId, result);

        return result;
    }

    /**
     * Determine the net torque on the base.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the torque vector (either storeResult or a new vector, not null)
     */
    public Vector3f baseTorque(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long multiBodyId = nativeId();
        getBaseTorque(multiBodyId, result);

        return result;
    }

    /**
     * Determine the transform of the base.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the transform from local coordinates to physics-space coordinates
     * (either storeResult or a new instance, not null, scale=1)
     */
    public Transform baseTransform(Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        long multiBodyId = nativeId();
        getBaseWorldTransform(multiBodyId, result);

        return result;
    }

    /**
     * Determine the linear velocity of the base's center of mass.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the velocity vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f baseVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long multiBodyId = nativeId();
        getBaseVel(multiBodyId, result);

        return result;
    }

    /**
     * Test whether this MultiBody can sleep. Note that "sleep" is synonym for
     * "deactivate".
     *
     * @return true if it can sleep, otherwise false
     */
    public boolean canSleep() {
        long multiBodyId = nativeId();
        boolean result = getCanSleep(multiBodyId);

        return result;
    }

    /**
     * Test whether this MultiBody can wake up.
     *
     * @return true if it can wake up, otherwise false
     */
    public boolean canWakeup() {
        long multiBodyId = nativeId();
        boolean result = getCanWakeup(multiBodyId);

        return result;
    }

    /**
     * Clear all constraint forces.
     */
    public void clearConstraintForces() {
        long multiBodyId = nativeId();
        clearConstraintForces(multiBodyId);
    }

    /**
     * Clear all external forces and torques.
     */
    public void clearForcesAndTorques() {
        long multiBodyId = nativeId();
        clearForcesAndTorques(multiBodyId);
    }

    /**
     * Zero out all velocities.
     */
    public void clearVelocities() {
        long multiBodyId = nativeId();
        clearVelocities(multiBodyId);
    }

    /**
     * Return the set of collision groups with which this multibody can collide.
     *
     * @return the bitmask
     */
    public int collideWithGroups() {
        long multiBodyId = nativeId();
        int result = getCollideWithGroups(multiBodyId);

        return result;
    }

    /**
     * Return the collision group of this multibody.
     *
     * @return the collision group (bitmask with exactly one bit set)
     */
    public int collisionGroup() {
        long multiBodyId = nativeId();
        int result = getCollisionGroup(multiBodyId);

        assert Integer.bitCount(result) == 1 : result;
        return result;
    }

    /**
     * Configure a new link that is fixed to its parent.
     *
     * @param mass the desired mass of the link (&gt;0)
     * @param inertia the desired moment of inertia of the link (not null,
     * unaffected)
     * @param parent the parent link, or null if parented by the base
     * @param orientation the orientation of the link relative to its parent
     * (not null, unaffected)
     * @param parent2Pivot the offset of the pivot from the parent's center of
     * mass (not null, unaffected)
     * @param pivot2Link the offset of the child's center of mass from the pivot
     * (not null, unaffected)
     * @return a new link
     */
    public MultiBodyLink configureFixedLink(float mass, Vector3f inertia,
            MultiBodyLink parent, Quaternion orientation, Vector3f parent2Pivot,
            Vector3f pivot2Link) {
        Validate.positive(mass, "mass");
        Validate.positive(inertia, "inertia");
        Validate.nonNull(orientation, "orientation");
        Validate.nonNull(parent2Pivot, "parent to pivot offset");
        Validate.nonNull(pivot2Link, "pivot to link offset");
        assert numConfigured < links.length;

        long multiBodyId = nativeId();
        int parentIndex = (parent == null) ? -1 : parent.index();
        setupFixed(multiBodyId, numConfigured, mass, inertia, parentIndex,
                orientation, parent2Pivot, pivot2Link);
        MultiBodyLink result = configureLink();

        return result;
    }

    /**
     * Configure a link that is joined to its parent with a planar joint.
     *
     * @param mass the desired mass of the link (&gt;0)
     * @param inertia the desired moment of inertia of the link (not null,
     * unaffected)
     * @param parent the parent link, or null if parented by the base
     * @param orientation the orientation of the link relative to its parent
     * (not null, unaffected)
     * @param axis the axis of rotation, which is also the plane's normal vector
     * (not null, unaffected)
     * @param parent2Link the offset of the child's center of mass from the
     * parent's center of mass (not null, unaffected)
     * @param disableCollision true to ignore collisions between the link and
     * its parent
     * @return a new link
     */
    public MultiBodyLink configurePlanarLink(float mass, Vector3f inertia,
            MultiBodyLink parent, Quaternion orientation, Vector3f axis,
            Vector3f parent2Link, boolean disableCollision) {
        Validate.positive(mass, "mass");
        Validate.positive(inertia, "inertia");
        Validate.nonNull(orientation, "orientation");
        Validate.nonNull(axis, "axis");
        Validate.nonNull(parent2Link, "parent to link offset");
        assert numConfigured < links.length;

        long multiBodyId = nativeId();
        int parentIndex = (parent == null) ? -1 : parent.index();
        setupPlanar(multiBodyId, numConfigured, mass, inertia, parentIndex,
                orientation, axis, parent2Link, disableCollision);
        MultiBodyLink result = configureLink();

        return result;
    }

    /**
     * Configure a link that is joined to its parent with a prismatic joint.
     *
     * @param mass the desired mass of the link (&gt;0)
     * @param inertia the desired moment of inertia of the link (not null,
     * unaffected)
     * @param parent the parent link, or null if parented by the base
     * @param orientation the orientation of the link relative to its parent
     * (not null, unaffected)
     * @param axis the axis of rotation (not null, unaffected)
     * @param parent2Pivot the offset of the pivot from the parent's center of
     * mass (not null, unaffected)
     * @param pivot2Link the offset of the child's center of mass from the pivot
     * (not null, unaffected)
     * @param disableCollision true to ignore collisions between the link and
     * its parent
     * @return a new link
     */
    public MultiBodyLink configurePrismaticLink(float mass, Vector3f inertia,
            MultiBodyLink parent, Quaternion orientation, Vector3f axis,
            Vector3f parent2Pivot, Vector3f pivot2Link,
            boolean disableCollision) {
        Validate.positive(mass, "mass");
        Validate.positive(inertia, "inertia");
        Validate.nonNull(orientation, "orientation");
        Validate.nonNull(axis, "axis");
        Validate.nonNull(parent2Pivot, "parent to pivot offset");
        Validate.nonNull(pivot2Link, "pivot to link offset");
        assert numConfigured < links.length;

        long multiBodyId = nativeId();
        int parentIndex = (parent == null) ? -1 : parent.index();
        setupPrismatic(multiBodyId, numConfigured, mass, inertia, parentIndex,
                orientation, axis, parent2Pivot, pivot2Link, disableCollision);
        MultiBodyLink result = configureLink();

        return result;
    }

    /**
     * Configure a link that is joined to its parent with a revolute joint.
     *
     * @param mass the desired mass of the link (&gt;0)
     * @param inertia the desired moment of inertia of the link (not null,
     * unaffected)
     * @param parent the parent link, or null if parented by the base
     * @param orientation the orientation of the link relative to its parent
     * (not null, unaffected)
     * @param axis the axis of rotation (not null, unaffected)
     * @param parent2Pivot the offset of the pivot from the parent's center of
     * mass (not null, unaffected)
     * @param pivot2Link the offset of the child's center of mass from the pivot
     * (not null, unaffected)
     * @param disableCollision true to ignore collisions between the link and
     * its parent
     * @return a new link
     */
    public MultiBodyLink configureRevoluteLink(float mass, Vector3f inertia,
            MultiBodyLink parent, Quaternion orientation, Vector3f axis,
            Vector3f parent2Pivot, Vector3f pivot2Link,
            boolean disableCollision) {
        Validate.positive(mass, "mass");
        Validate.positive(inertia, "inertia");
        Validate.nonNull(orientation, "orientation");
        Validate.nonNull(axis, "axis");
        Validate.nonNull(parent2Pivot, "parent to pivot offset");
        Validate.nonNull(pivot2Link, "pivot to link offset");
        assert numConfigured < links.length;

        long multiBodyId = nativeId();
        int parentIndex = (parent == null) ? -1 : parent.index();
        setupRevolute(multiBodyId, numConfigured, mass, inertia, parentIndex,
                orientation, axis, parent2Pivot, pivot2Link, disableCollision);
        MultiBodyLink result = configureLink();

        return result;
    }

    /**
     * Configure a link that is joined to its parent with a spherical joint.
     *
     * @param mass the desired mass of the link (&gt;0)
     * @param inertia the desired moment of inertia of the link (not null,
     * unaffected)
     * @param parent the parent link, or null if parented by the base
     * @param orientation the orientation of the link relative to its parent
     * (not null, unaffected)
     * @param parent2Pivot the offset of the pivot from the parent's center of
     * mass (not null, unaffected)
     * @param pivot2Link the offset of the child's center of mass from the pivot
     * (not null, unaffected)
     * @param disableCollision true to ignore collisions between the link and
     * its parent
     * @return a new link
     */
    public MultiBodyLink configureSphericalLink(float mass, Vector3f inertia,
            MultiBodyLink parent, Quaternion orientation, Vector3f parent2Pivot,
            Vector3f pivot2Link, boolean disableCollision) {
        Validate.positive(mass, "mass");
        Validate.positive(inertia, "inertia");
        Validate.nonNull(orientation, "orientation");
        Validate.nonNull(parent2Pivot, "parent to pivot offset");
        Validate.nonNull(pivot2Link, "pivot to link offset");
        assert numConfigured < links.length;

        long multiBodyId = nativeId();
        int parentIndex = (parent == null) ? -1 : parent.index();
        setupSpherical(multiBodyId, numConfigured, mass, inertia, parentIndex,
                orientation, parent2Pivot, pivot2Link, disableCollision);
        MultiBodyLink result = configureLink();

        return result;
    }

    /**
     * Test whether the specified collider is part of this MultiBody.
     *
     * @param collider the collider to search for (not null, unaffected)
     * @return true if included, otherwise false
     */
    public boolean contains(MultiBodyCollider collider) {
        boolean result = false;

        for (MultiBodyLink link : links) {
            if (link != null && link.getCollider() == collider) {
                result = true;
                break;
            }
        }

        if (!result && collider == baseCollider) {
            result = true;
        }

        return result;
    }

    /**
     * Count the configured links in this MultiBody.
     *
     * @return the count (&ge;0)
     */
    public int countConfiguredLinks() {
        assert numConfigured >= 0 : numConfigured;
        return numConfigured;
    }

    /**
     * Count the degrees of freedom.
     *
     * @return the count (&ge;0)
     */
    public int countDofs() {
        long multiBodyId = nativeId();
        int result = getNumDofs(multiBodyId);

        return result;
    }

    /**
     * Count the position variables in this MultiBody.
     *
     * @return the count (&ge;0)
     */
    public int countPositionVariables() {
        long multiBodyId = nativeId();
        int result = getNumPosVars(multiBodyId);

        return result;
    }

    /**
     * Access the collider for the base.
     *
     * @return the pre-existing instance, or null if none
     */
    public MultiBodyCollider getBaseCollider() {
        assert baseCollider == null
                ? getBaseCollider(nativeId()) == 0L
                : getBaseCollider(nativeId()) == baseCollider.nativeId();

        return baseCollider;
    }

    /**
     * Access the index link of this MultiBody.
     *
     * @param linkIndex (&ge;0, &lt;numConfigured)
     * @return the pre-existing instance (not null)
     */
    public MultiBodyLink getLink(int linkIndex) {
        Validate.inRange(linkIndex, "link index", 0, numConfigured - 1);
        MultiBodyLink result = links[linkIndex];

        assert result != null;
        return result;
    }

    /**
     * Test whether this MultiBody has a fixed base.
     *
     * @return true &rarr; fixed, false &rarr; movable
     */
    public boolean hasFixedBase() {
        long multiBodyId = nativeId();
        boolean result = hasFixedBase(multiBodyId);

        return result;
    }

    /**
     * Test whether this MultiBody uses global variables.
     *
     * @return true if using global variables, otherwise false
     */
    public boolean isUsingGlobalVelocities() {
        long multiBodyId = nativeId();
        boolean result = isUsingGlobalVelocities(multiBodyId);

        return result;
    }

    /**
     * Test whether this MultiBody uses the gyro term.
     *
     * @return true if using the gyro term, otherwise false
     */
    public boolean isUsingGyroTerm() {
        long multiBodyId = nativeId();
        boolean result = getUseGyroTerm(multiBodyId);

        return result;
    }

    /**
     * Test whether this MultiBody uses RK4 integration.
     *
     * @return true if using RK4, otherwise false
     */
    public boolean isUsingRK4() {
        long multiBodyId = nativeId();
        boolean result = isUsingRK4Integration(multiBodyId);

        return result;
    }

    /**
     * Determine the linear damping.
     *
     * @return the damping
     */
    public float linearDamping() {
        long multiBodyId = nativeId();
        float result = getLinearDamping(multiBodyId);

        return result;
    }

    /**
     * Enumerate the colliders in this MultiBody.
     *
     * @return a new collection of pre-existing instances
     */
    public List<MultiBodyCollider> listColliders() {
        int capacity = numConfigured + 1;
        List<MultiBodyCollider> result = new ArrayList<>(capacity);

        if (baseCollider != null) {
            result.add(baseCollider);
        }
        for (MultiBodyLink link : links) {
            if (link != null) {
                MultiBodyCollider collider = link.getCollider();
                if (collider != null) {
                    result.add(collider);
                }
            }
        }

        return result;
    }

    /**
     * Determine the maximum applied impulse.
     *
     * @return the impulse
     */
    public float maxAppliedImpulse() {
        long multiBodyId = nativeId();
        float result = getMaxAppliedImpulse(multiBodyId);

        return result;
    }

    /**
     * Determine the maximum coordinate velocity.
     *
     * @return the velocity
     */
    public float maxCoordinateVelocity() {
        long multiBodyId = nativeId();
        float result = getMaxCoordinateVelocity(multiBodyId);

        return result;
    }

    /**
     * Alter the angular velocity of the base.
     *
     * @param angularVelocity the desired angular-velocity vector (in
     * physics-space coordinates, not null, unaffected, default=(0,0,0))
     */
    public void setBaseAngularVelocity(Vector3f angularVelocity) {
        Validate.finite(angularVelocity, "angular velocity");

        long multiBodyId = nativeId();
        setBaseOmega(multiBodyId, angularVelocity);
    }

    /**
     * Alter the location of the base's center of mass.
     *
     * @param location the desired location vector (in physics-space
     * coordinates, not null, unaffected, default=(0,0,0))
     */
    public void setBaseLocation(Vector3f location) {
        Validate.finite(location, "location");

        long multiBodyId = nativeId();
        setBasePos(multiBodyId, location);
    }

    /**
     * Alter the orientation of the base.
     *
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setBaseOrientation(Quaternion orientation) {
        Validate.nonNull(orientation, "orientation");

        long multiBodyId = nativeId();
        setWorldToBaseRot(multiBodyId, orientation);
    }

    /**
     * Alter the transform of the base.
     *
     * @param transform the desired transform from local coordinates to
     * physics-space coordinates (not null, unaffected, scale ignored)
     */
    public void setBaseTransform(Transform transform) {
        Validate.nonNull(transform, "transform");

        long multiBodyId = nativeId();
        setBaseWorldTransform(multiBodyId, transform);
    }

    /**
     * Alter the linear velocity of the base.
     *
     * @param velocity the desired velocity vector (in physics-space
     * coordinates, not null, unaffected, default=(0,0,0))
     */
    public void setBaseVelocity(Vector3f velocity) {
        Validate.finite(velocity, "velocity");

        long multiBodyId = nativeId();
        setBaseVel(multiBodyId, velocity);
    }

    /**
     * Directly alter the collision groups with which this MultiBody can
     * collide.
     *
     * @param groups the desired groups, ORed together (bitmask,
     * default=COLLISION_GROUP_01)
     */
    public void setCollideWithGroups(int groups) {
        long multiBodyId = nativeId();
        setCollideWithGroups(multiBodyId, groups);
    }

    /**
     * Alter which collision group this MultiBody belongs to.
     * <p>
     * Groups are represented by bitmasks with exactly one bit set. Manifest
     * constants are defined in PhysicsCollisionObject.
     * <p>
     * Two objects can collide only if one of them has the collisionGroup of the
     * other in its collideWithGroups set.
     *
     * @param group the collisionGroup to apply (bitmask with exactly one bit
     * set, default=COLLISION_GROUP_01)
     */
    public void setCollisionGroup(int group) {
        Validate.require(Integer.bitCount(group) == 1, "exactly one bit set");

        long multiBodyId = nativeId();
        setCollisionGroup(multiBodyId, group);
    }

    /**
     * Alter the primary user index. Applications may use this parameter for any
     * purpose (native field: m_userIndex).
     *
     * @param index the desired value (default=-1)
     */
    public void setUserIndex(int index) {
        long multiBodyId = nativeId();
        setUserIndex(multiBodyId, index);
    }

    /**
     * Alter the secondary user index. Applications may use this parameter for
     * any purpose (native field: m_userIndex2).
     *
     * @param index the desired value (default=-1)
     */
    public void setUserIndex2(int index) {
        long multiBodyId = nativeId();
        setUserIndex2(multiBodyId, index);
    }

    /**
     * Determine the ID of the MultiBodySpace to which this MultiBody is added.
     *
     * @return the ID, or zero if not in any space
     */
    public long spaceId() {
        long multiBodyId = nativeId();
        long spaceId = getSpace(multiBodyId);

        return spaceId;
    }

    /**
     * Alter whether this MultiBody uses global velocities.
     *
     * @param setting true to use global velocities (default=false)
     */
    public void useGlobalVelocities(boolean setting) {
        long multiBodyId = nativeId();
        useGlobalVelocities(multiBodyId, setting);
    }

    /**
     * Alter whether this MultiBody uses RK4 integration.
     *
     * @param setting true to use RK4 (default=false)
     */
    public void useRK4(boolean setting) {
        long multiBodyId = nativeId();
        useRK4Integration(multiBodyId, setting);
    }

    /**
     * Return the primary user index (native field: m_userIndex).
     *
     * @return the value
     */
    public int userIndex() {
        long multiBodyId = nativeId();
        int result = getUserIndex(multiBodyId);
        return result;
    }

    /**
     * Return the secondary user index (native field: m_userIndex2).
     *
     * @return the value
     */
    public int userIndex2() {
        long multiBodyId = nativeId();
        int result = getUserIndex2(multiBodyId);
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Configure a new link that's just been set up.
     *
     * @return the new link (not null)
     */
    private MultiBodyLink configureLink() {
        int linkIndex = numConfigured;
        ++numConfigured;

        long multiBodyId = nativeId();
        finalizeMultiDof(multiBodyId);

        MultiBodyLink result = new MultiBodyLink(this, linkIndex);
        links[linkIndex] = result;

        return result;
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param multiBodyId the native identifier (not zero)
     */
    private static void freeNativeObject(long multiBodyId) {
        assert multiBodyId != 0L;
        finalizeNative(multiBodyId);
    }
    // *************************************************************************
    // native private methods

    native private static void
            addBaseForce(long multiBodyId, Vector3f forceVector);

    native private static void
            addBaseTorque(long multiBodyId, Vector3f torqueVector);

    native private static void clearConstraintForces(long multiBodyId);

    native private static void clearForcesAndTorques(long multiBodyId);

    native private static void clearVelocities(long multiBodyId);

    native private long create(int numLinks, float baseMass,
            Vector3f baseInertiaVector, boolean fixedBase, boolean canSleep);

    native private static void finalizeMultiDof(long multiBodyId);

    native private static void finalizeNative(long multiBodyId);

    native private static float getAngularDamping(long multiBodyId);

    native private static long getBaseCollider(long multiBodyId);

    native private static void
            getBaseForce(long multiBodyId, Vector3f storeVector);

    native private static void
            getBaseInertia(long multiBodyId, Vector3f storeVector);

    native private static float getBaseMass(long multiBodyId);

    native private static void
            getBaseOmega(long multiBodyId, Vector3f storeVector);

    native private static void
            getBasePos(long multiBodyId, Vector3f storeVector);

    native private static void
            getBaseTorque(long multiBodyId, Vector3f storeVector);

    native private static void
            getBaseVel(long multiBodyId, Vector3f storeVector);

    native private static void
            getBaseWorldTransform(long multiBodyId, Transform storeTransform);

    native private static boolean getCanSleep(long multiBodyId);

    native private static boolean getCanWakeup(long multiBodyId);

    native private static int getCollideWithGroups(long multiBodyId);

    native private static int getCollisionGroup(long multiBodyId);

    native private static float getLinearDamping(long multiBodyId);

    native private static float getMaxAppliedImpulse(long multiBodyId);

    native private static float getMaxCoordinateVelocity(long multiBodyId);

    native private static int getNumDofs(long multiBodyId);

    native private static int getNumLinks(long multiBodyId);

    native private static int getNumPosVars(long multiBodyId);

    native private static long getSpace(long multiBodyId);

    native private static boolean getUseGyroTerm(long multiBodyId);

    native private static int getUserIndex(long multiBodyId);

    native private static int getUserIndex2(long multiBodyId);

    native private static void
            getWorldToBaseRot(long multiBodyId, Quaternion storeQuaternion);

    native private static boolean hasFixedBase(long multiBodyId);

    native private static boolean isUsingGlobalVelocities(long multiBodyId);

    native private static boolean isUsingRK4Integration(long multiBodyId);

    native private static void
            setBaseCollider(long multiBodyId, long colliderId);

    native private static void
            setBaseOmega(long multiBodyId, Vector3f angularVelocityVector);

    native private static void
            setBasePos(long multiBodyId, Vector3f positionVector);

    native private static void
            setBaseVel(long multiBodyId, Vector3f velocityVector);

    native private static void
            setBaseWorldTransform(long multiBodyId, Transform transform);

    native private static void
            setCollideWithGroups(long multiBodyId, int collisionGroups);

    native private static void
            setCollisionGroup(long multiBodyId, int collisionGroup);

    native private static void setupFixed(long multiBodyId, int linkIndex,
            float mass, Vector3f inertiaVector, int parentLinkIndex,
            Quaternion parent2LinkQuaternion, Vector3f parent2PivotVector,
            Vector3f pivot2LinkVector);

    native private static void setupPlanar(long multiBodyId, int linkIndex,
            float mass, Vector3f inertiaVector, int parentLinkIndex,
            Quaternion parent2LinkQuaternion, Vector3f axisVector,
            Vector3f parent2LinkVector, boolean disableParentCollision);

    native private static void setupPrismatic(long multiBodyId, int linkIndex,
            float mass, Vector3f inertiaVector, int parentLinkIndex,
            Quaternion parent2LinkQuaternion, Vector3f axisVector,
            Vector3f parent2PivotVector, Vector3f pivot2LinkVector,
            boolean disableParentCollision);

    native private static void setupRevolute(long multiBodyId, int linkIndex,
            float mass, Vector3f inertiaVector, int parentLinkIndex,
            Quaternion parent2LinkQuaternion, Vector3f axisVector,
            Vector3f parent2PivotVector, Vector3f pivot2LinkVector,
            boolean disableParentCollision);

    native private static void setupSpherical(long multiBodyId, int linkIndex,
            float mass, Vector3f inertiaVector, int parentLinkIndex,
            Quaternion parent2LinkQuaternion, Vector3f parent2PivotVector,
            Vector3f pivotToLinkVector, boolean disableParentCollision);

    native private static void setUserIndex(long multiBodyId, int index);

    native private static void setUserIndex2(long multiBodyId, int index);

    native private static void
            setWorldToBaseRot(long multiBodyId, Quaternion quaternion);

    native private static void
            useGlobalVelocities(long multiBodyId, boolean use);

    native private static void useRK4Integration(long multiBodyId, boolean use);
}
