/*
 * Copyright (c) 2020 jMonkeyEngine
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

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An articulated rigid body based on Bullet's btMultiBody. Uses Featherstone's
 * algorithm.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiBody {
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
     * unique identifier of the btMultiBody
     */
    final private long nativeId;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a MultiBody.
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

        nativeId = create(numLinks, baseMass, baseInertia, fixedBase, canSleep);
        assert nativeId != 0L;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add an external force to the base.
     *
     * @param force the force to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addBaseForce(Vector3f force) {
        Validate.finite(force, "force");
        addBaseForce(nativeId, force);
    }

    /**
     * Add an external torque to the base.
     *
     * @param torque the torque to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addBaseTorque(Vector3f torque) {
        Validate.finite(torque, "torque");
        addBaseTorque(nativeId, torque);
    }

    /**
     * Add an external force to the indexed link.
     *
     * @param linkIndex which link to add it to (&ge;0, &lt;numLinks)
     * @param force the force to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addLinkForce(int linkIndex, Vector3f force) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);
        Validate.finite(force, "force");

        addLinkForce(nativeId, linkIndex, force);
    }

    /**
     * Add an external torque to the indexed link.
     *
     * @param linkIndex which link to add it to (&ge;0, &lt;numLinks)
     * @param torque the torque to add (in physics-space coordinates, not null,
     * unaffected)
     */
    public void addLinkTorque(int linkIndex, Vector3f torque) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);
        Validate.finite(torque, "torque");

        addLinkTorque(nativeId, linkIndex, torque);
    }

    /**
     * Determine the total angular momentum of this MultiBody.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the momentum vector (either storeResult or a new vector, not
     * null)
     */
    public Vector3f angularMomentum(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getAngularMomentum(nativeId, result);
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
        getBaseOmega(nativeId, result);
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
        getBaseInertia(nativeId, result);
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
        getBasePos(nativeId, result);
        return result;
    }

    /**
     * Determine the mass of the base.
     *
     * @return the mass (in physics-space units, &gt;0)
     */
    public float baseMass() {
        float result = getBaseMass(nativeId);
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
        getWorldToBaseRot(nativeId, result);
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
        getBaseWorldTransform(nativeId, result);
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
        getBaseVel(nativeId, result);
        return result;
    }

    /**
     * Test whether this MultiBody can sleep.
     *
     * @return true if it can sleep, otherwise false
     */
    public boolean canSleep() {
        boolean result = canSleep(nativeId);
        return result;
    }

    /**
     * Clear all constraint forces.
     */
    public void clearConstraintForces() {
        clearConstraintForces(nativeId);
    }

    /**
     * Clear all external forces and torques.
     */
    public void clearForcesAndTorques() {
        clearForcesAndTorques(nativeId);
    }

    /**
     * Zero out all velocities.
     */
    public void clearVelocities() {
        clearVelocities(nativeId);
    }

    /**
     * Count the degrees of freedom.
     *
     * @return the count (&ge;0)
     */
    public int countDofs() {
        int result = getNumDofs(nativeId);
        return result;
    }

    /**
     * Count the links.
     *
     * @return the count, not including the base (&ge;0)
     */
    public int countLinks() {
        int result = getNumLinks(nativeId);
        return result;
    }

    /**
     * Read the unique identifier of the native object.
     *
     * @return the ID (not zero)
     */
    final public long getNativeId() {
        assert nativeId != 0L;
        return nativeId;
    }

    /**
     * Test whether this MultiBody has a fixed base.
     *
     * @return true &rarr; fixed, false &rarr; movable
     */
    public boolean hasFixedBase() {
        boolean result = hasFixedBase(nativeId);
        return result;
    }

    /**
     * Test whether this MultiBody uses global variables.
     *
     * @return true if using global variables, otherwise false
     */
    public boolean isUsingGlobalVelocities() {
        boolean result = isUsingGlobalVelocities(nativeId);
        return result;
    }

    /**
     * Test whether this MultiBody uses RK4 integration.
     *
     * @return true if using RK4, otherwise false
     */
    public boolean isUsingRK4() {
        boolean result = isUsingRK4Integration(nativeId);
        return result;
    }

    /**
     * Determine the joint position of the indexed link.
     *
     * @param linkIndex which link (&ge;0, &lt;numLinks)
     * @return the position
     */
    public float jointPosition(int linkIndex) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);

        float result = getJointPos(nativeId, linkIndex);
        return result;
    }

    /**
     * Determine the joint velocity of the indexed link.
     *
     * @param linkIndex which link (&ge;0, &lt;numLinks)
     * @return the velocity
     */
    public float jointVelocity(int linkIndex) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);

        float result = getJointVel(nativeId, linkIndex);
        return result;
    }

    /**
     * Determine the total kinetic energy of this MultiBody.
     *
     * @return the energy (&ge;0)
     */
    public float kineticEnergy() {
        float result = getKineticEnergy(nativeId);
        return result;
    }

    /**
     * Determine the rotational inertia of the indexed link.
     *
     * @param linkIndex which link (&ge;0, &lt;numLinks)
     * @param storeResult storage for the result (modified if not null)
     * @return the principal (diagonal) components of the inertia tensor (in the
     * link's local coordinates, either storeResult or a new vector, not null)
     */
    public Vector3f linkInertia(int linkIndex, Vector3f storeResult) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        getLinkInertia(nativeId, linkIndex, result);
        return result;
    }

    /**
     * Determine the mass of the indexed link.
     *
     * @param linkIndex which link (&ge;0, &lt;numLinks)
     * @return the mass (in physics mass units, &gt;0)
     */
    public float linkMass(int linkIndex) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);

        float result = getLinkMass(nativeId, linkIndex);
        return result;
    }

    /**
     * Determine the parent of the indexed link.
     *
     * @param linkIndex which link (&ge;0, &lt;numLinks)
     * @return the parent's link index, or -1 if the link is parented to the
     * base
     */
    public int parent(int linkIndex) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);

        int result = getParent(nativeId, linkIndex);
        return result;
    }

    /**
     * Alter the angular velocity of the base.
     *
     * @param angularVelocity the desired angular-velocity vector (in
     * physics-space coordinates, not null, unaffected)
     */
    public void setBaseAngularVelocity(Vector3f angularVelocity) {
        Validate.finite(angularVelocity, "angular velocity");
        setBaseOmega(nativeId, angularVelocity);
    }

    /**
     * Alter the location of the base's center of mass.
     *
     * @param location the desired location vector (in physics-space
     * coordinates, not null, unaffected)
     */
    public void setBaseLocation(Vector3f location) {
        Validate.finite(location, "location");
        setBasePos(nativeId, location);
    }

    /**
     * Alter the orientation of the base.
     *
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setBaseOrientation(Quaternion orientation) {
        Validate.nonNull(orientation, "orientation");
        setWorldToBaseRot(nativeId, orientation);
    }

    /**
     * Alter the transform of the base.
     *
     * @param transform the desired transform from local coordinates to
     * physics-space coordinates (not null, unaffected, scale ignored)
     */
    public void setBaseTransform(Transform transform) {
        Validate.nonNull(transform, "transform");
        setBaseWorldTransform(nativeId, transform);
    }

    /**
     * Alter the linear velocity of the base.
     *
     * @param velocity the desired velocity vector (in physics-space
     * coordinates, not null, unaffected)
     */
    public void setBaseVelocity(Vector3f velocity) {
        Validate.finite(velocity, "velocity");
        setBaseVel(nativeId, velocity);
    }

    /**
     * Alter the position of the indexed link's joint.
     *
     * @param linkIndex which link (&ge;0, &lt;numLinks)
     * @param position the desired position
     */
    public void setJointPosition(int linkIndex, float position) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);

        setJointPos(nativeId, linkIndex, position);
    }

    /**
     * Alter the velocity of the indexed link's joint.
     *
     * @param linkIndex which link (&ge;0, &lt;numLinks)
     * @param velocity the desired velocity
     */
    public void setJointVelocity(int linkIndex, float velocity) {
        int numLinks = countLinks();
        Validate.inRange(linkIndex, "link index", 0, numLinks - 1);

        setJointVel(nativeId, linkIndex, velocity);
    }

    /**
     * Alter whether this MultiBody uses global velocities.
     *
     * @param setting true to use global velocities
     */
    public void useGlobalVelocities(boolean setting) {
        useGlobalVelocities(nativeId, setting);
    }

    /**
     * Alter whether this MultiBody uses RK4 integration.
     *
     * @param setting true to use RK4
     */
    public void useRK4(boolean setting) {
        useRK4Integration(nativeId, setting);
    }

    // *************************************************************************
    // Object methods
    /**
     * Finalize this MultiBody just before it is destroyed. Should be invoked
     * only by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        logger.log(Level.FINE, "Finalizing {0}.", this);
        finalizeNative(nativeId);
    }

    /**
     * Represent this MultiBody as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = getClass().getSimpleName();
        result += "#" + Long.toHexString(nativeId);

        return result;
    }
    // *************************************************************************
    // native methods

    native private void addBaseForce(long multiBodyId, Vector3f forceVector);

    native private void addBaseTorque(long multiBodyId, Vector3f torqueVector);

    native private void addLinkForce(long multiBodyId, int linkIndex,
            Vector3f forceVector);

    native private void addLinkTorque(long multiBodyId, int linkIndex,
            Vector3f torqueVector);

    native private boolean canSleep(long multiBodyId);

    native private void clearConstraintForces(long multiBodyId);

    native private void clearForcesAndTorques(long multiBodyId);

    native private void clearVelocities(long multiBodyId);

    native private long create(int numLinks, float baseMass,
            Vector3f baseInertiaVector, boolean fixedBase, boolean canSleep);

    native private void finalizeNative(long multiBodyId);

    native private void getAngularMomentum(long multiBodyId,
            Vector3f storeVector);

    native private void getBaseInertia(long multiBodyId, Vector3f storeVector);

    native private float getBaseMass(long multiBodyId);

    native private void getBaseOmega(long multiBodyId, Vector3f storeVector);

    native private void getBasePos(long multiBodyId, Vector3f storeVector);

    native private void getBaseVel(long multiBodyId, Vector3f storeVector);

    native private void getBaseWorldTransform(long multiBodyId,
            Transform storeTransform);

    native private float getJointPos(long multiBodyId, int linkIndex);

    native private float getJointVel(long multiBodyId, int linkIndex);

    native private float getKineticEnergy(long multiBodyId);

    native private void getLinkInertia(long multiBodyId, int linkIndex,
            Vector3f storeVector);

    native private float getLinkMass(long multiBodyId, int linkIndex);

    native private int getNumDofs(long multiBodyId);

    native private int getNumLinks(long multiBodyId);

    native private int getParent(long multiBodyId, int childLinkIndex);

    native private void getWorldToBaseRot(long multiBodyId,
            Quaternion storeQuaternion);

    native private boolean hasFixedBase(long multiBodyId);

    native private boolean isUsingGlobalVelocities(long multiBodyId);

    native private boolean isUsingRK4Integration(long multiBodyId);

    native private void setBaseOmega(long multiBodyId,
            Vector3f angularVelocityVector);

    native private void setBasePos(long multiBodyId, Vector3f positionVector);

    native private void setBaseVel(long multiBodyId, Vector3f velocityVector);

    native private void setBaseWorldTransform(long multiBodyId,
            Transform transform);

    native private void setJointPos(long multiBodyId, int linkIndex,
            float position);

    native private void setJointVel(long multiBodyId, int linkIndex,
            float velocity);

    native private void setWorldToBaseRot(long multiBodyId,
            Quaternion quaternion);

    native private void useGlobalVelocities(long multiBodyId, boolean use);

    native private void useRK4Integration(long multiBodyId, boolean use);
}
