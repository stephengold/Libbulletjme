/*
 * Copyright (c) 2022 jMonkeyEngine
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
package com.jme3.bullet.joints;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A joint that couples the angular velocity for two bodies based on Bullet's
 * btGearConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * <p>
 * The btGearConstraint will couple the angular velocity for two bodies around
 * given local axis and ratio.
 *
 * @author elmfrain
 */
public class GearJoint extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(GearJoint.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the joint axis in A's local coordinates (unit vector)
     */
    final private Vector3f axisA;
    /**
     * copy of the joint axis in B's local coordinates (unit vector)
     */
    final private Vector3f axisB;
    /**
     * copy of the joint ratio; gear ratio
     */
    final private float ratio;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a double-ended GearJoint with a 1:1 ratio.
     * <p>
     * To be effective, the joint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param axisInA the axis of rotation in A's local coordinates (not null,
     * not zero, unaffected)
     * @param axisInB the axis of rotation in B's local coordinates (not null,
     * not zero, unaffected)
     */
    public GearJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f axisInA, Vector3f axisInB) {
        super(rigidBodyA, rigidBodyB, Vector3f.ZERO, Vector3f.ZERO);

        assert axisInA.isUnitVector() : axisInA;
        assert axisInB.isUnitVector() : axisInB;

        axisA = axisInA.clone();
        axisB = axisInB.clone();
        ratio = 1.0f;
        createJoint();
    }

    /**
     * Instantiate a double-ended GearJoint with the specified ratio.
     * <p>
     * To be effective, the joint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param axisInA the axis of rotation in A's local coordinates (not null,
     * not zero, unaffected)
     * @param axisInB the axis of rotation in B's local coordinates (not null,
     * not zero, unaffected)
     * @param ratio the ratio of the rotation rates
     */
    public GearJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f axisInA, Vector3f axisInB, float ratio) {
        super(rigidBodyA, rigidBodyB, Vector3f.ZERO, Vector3f.ZERO);

        assert axisInA.isUnitVector() : axisInA;
        assert axisInB.isUnitVector() : axisInB;

        axisA = axisInA.clone();
        axisB = axisInB.clone();
        this.ratio = ratio;
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the joint's rotation axis in body A.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return The rotation axis in body A (either storeResult or new vector)
     */
    public Vector3f getAxisA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getAxisA(constraintId, result);

        return result;
    }

    /**
     * Copy the joint's rotation axis in body B.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return The rotation axis in body B (either storeResult or new vector)
     */
    public Vector3f getAxisB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getAxisB(constraintId, result);

        return result;
    }

    /**
     * Get the joint's gear ratio.
     *
     * @return The gear ratio
     */
    public float getRatio() {
        long constraintId = nativeId();
        return getRatio(constraintId);
    }

    /**
     * Alter the joint's rotation axis in body A.
     *
     * @param axisA the desired axis (unit vector, not null, unaffected)
     */
    public void setAxisA(Vector3f axisA) {
        Validate.nonZero(axisA, "Axis in body A");

        long constraintId = nativeId();
        setAxisA(constraintId, axisA);
    }

    /**
     * Alter the joint's rotation axis in body B.
     *
     * @param axisB the desired axis (unit vector, not null, unaffected)
     */
    public void setAxisB(Vector3f axisB) {
        Validate.nonZero(axisB, "Axis in body B");

        long constraintId = nativeId();
        setAxisB(constraintId, axisB);
    }

    /**
     * Alter the joint's gear ratio.
     *
     * @param ratio the gear ratio
     */
    public void setRatio(float ratio) {
        long constraintId = nativeId();
        setRatio(constraintId, ratio);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        PhysicsRigidBody a = getBodyA();
        long aId = a.nativeId();
        assert axisA.isUnitVector();

        PhysicsRigidBody b = getBodyB();
        long bId = b.nativeId();
        assert axisB.isUnitVector();

        long constraintId;
        /*
         * Create a double-ended joint.
         */
        constraintId = createJoint(aId, bId, axisA, axisB, ratio);

        assert getConstraintType(constraintId) == 10;
        setNativeId(constraintId);
    }
    // *************************************************************************
    // native private methods

    native private static long createJoint(long objectIdA, long objectIdB,
            Vector3f axisInA, Vector3f axisInB, float ratio);

    native private static void getAxisA(long jointId, Vector3f storeResult);

    native private static void getAxisB(long jointId, Vector3f storeResult);

    native private static float getRatio(long jointId);

    native private static void setAxisA(long jointId, Vector3f axisA);

    native private static void setAxisB(long jointId, Vector3f axisB);

    native private static void setRatio(long jointId, float ratio);
}
