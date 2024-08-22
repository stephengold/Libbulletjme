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
package com.jme3.bullet.joints;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * A 3 degree-of-freedom joint based on Bullet's btPoint2PointConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * Point to point constraint limits the translation so that the local pivot
 * points of 2 rigid bodies match in worldspace. A chain of rigid bodies can be
 * connected using this constraint.
 *
 * @author normenhansen
 */
public class Point2PointJoint extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(Point2PointJoint.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a single-ended Point2PointJoint where the constraint is
     * already satisfied.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyA the body to constrain (not null, alias created)
     * @param pivotInA the pivot location in the body's scaled local coordinates
     * (not null, unaffected)
     */
    public Point2PointJoint(PhysicsRigidBody rigidBodyA, Vector3f pivotInA) {
        super(rigidBodyA, JointEnd.A, pivotInA);
        createJoint();
    }

    /**
     * Instantiate a single-ended Point2PointJoint where the constraint might
     * not be satisfied yet.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyA the body to constrain (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     */
    public Point2PointJoint(PhysicsRigidBody rigidBodyA, Vector3f pivotInA,
            Vector3f pivotInWorld) {
        super(rigidBodyA, JointEnd.A, pivotInA, pivotInWorld);
        createJoint();
    }

    /**
     * Instantiate a double-ended Point2PointJoint.
     * <p>
     * To be effective, the joint must be added to the PhysicsSpace of both
     * bodies. Also, the bodies must be distinct and at least one of them must
     * be dynamic.
     *
     * @param rigidBodyA the body for the A end (not null, alias created)
     * @param rigidBodyB the body for the B end (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     */
    public Point2PointJoint(PhysicsRigidBody rigidBodyA,
            PhysicsRigidBody rigidBodyB, Vector3f pivotInA, Vector3f pivotInB) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the joint's damping ratio.
     *
     * @return the viscous damping ratio (0&rarr;no damping, 1&rarr;critically
     * damped)
     */
    public float getDamping() {
        long constraintId = nativeId();
        float result = getDamping(constraintId);

        return result;
    }

    /**
     * Return the joint's impulse clamp.
     *
     * @return the clamp value
     */
    public float getImpulseClamp() {
        long constraintId = nativeId();
        float result = getImpulseClamp(constraintId);

        return result;
    }

    /**
     * Locate the pivot in A's scaled local coordinates.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the pivot location (either storeResult or a new vector, not null)
     */
    public Vector3f getPivotInA(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        long constraintId = nativeId();
        getPivotInA(constraintId, result);

        return result;
    }

    /**
     * Locate the pivot in B's scaled local coordinates.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the pivot location (either storeResult or a new vector, not null)
     */
    public Vector3f getPivotInB(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        long constraintId = nativeId();
        getPivotInB(constraintId, result);

        return result;
    }

    /**
     * Return the joint's tau value.
     *
     * @return the tau value
     */
    public float getTau() {
        long constraintId = nativeId();
        float result = getTau(constraintId);

        return result;
    }

    /**
     * Alter the joint's damping.
     *
     * @param value the desired viscous damping ratio (0&rarr;no damping,
     * 1&rarr;critically damped, default=1)
     */
    public void setDamping(float value) {
        long constraintId = nativeId();
        setDamping(constraintId, value);
    }

    /**
     * Alter the joint's impulse clamp.
     *
     * @param value the desired impulse clamp value (default=0)
     */
    public void setImpulseClamp(float value) {
        long constraintId = nativeId();
        setImpulseClamp(constraintId, value);
    }

    /**
     * Alter the joint's tau value.
     *
     * @param value the desired tau value (default=0.3)
     */
    public void setTau(float value) {
        long constraintId = nativeId();
        setTau(constraintId, value);
    }
    // *************************************************************************
    // Constraint methods

    /**
     * Alter the pivot location in A's scaled local coordinates.
     *
     * @param location the desired location (not null, unaffected)
     */
    @Override
    public void setPivotInA(Vector3f location) {
        Validate.nonNull(location, "location");

        long constraintId = nativeId();
        setPivotInA(constraintId, location);
        super.setPivotInA(location);
    }

    /**
     * Alter the pivot location in B's scaled local coordinates. Unlike Minie,
     * Libbulletjme implements this method for both single-ended and
     * double-ended joints.
     *
     * @param location the desired location (not null, unaffected)
     */
    @Override
    public void setPivotInB(Vector3f location) {
        Validate.nonNull(location, "location");

        long constraintId = nativeId();
        setPivotInB(constraintId, location);

        if (pivotB != null) {
            super.setPivotInB(location);
        }
    }
    // *************************************************************************
    // Java private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        PhysicsRigidBody a = getBodyA();
        long aId = a.nativeId();
        assert pivotA != null;
        PhysicsRigidBody b = getBodyB();

        long constraintId;
        if (b == null) {
            // Create a single-ended joint.
            if (pivotB == null) {
                constraintId = createJoint1(aId, pivotA);
            } else {
                /*
                 * Bullet assumes single-ended btPoint2PointConstraints are
                 * satisfied at creation, so we temporarily re-position the body
                 * to satisfy the constraint.
                 */
                Vector3f saveLocation = a.getPhysicsLocation(null);

                Transform localToWorld = new Transform();
                localToWorld.setTranslation(saveLocation);
                a.getPhysicsRotation(localToWorld.getRotation());

                Vector3f pivotAWorld
                        = MyMath.transform(localToWorld, pivotA, null);
                Vector3f worldOffset = pivotB.subtract(pivotAWorld);
                Vector3f tempLocation = saveLocation.add(worldOffset);
                a.setPhysicsLocation(tempLocation);
                constraintId = createJoint1(aId, pivotA);

                a.setPhysicsLocation(saveLocation);
            }

        } else {
            assert pivotB != null;

            // Create a double-ended joint.
            long bId = b.nativeId();
            constraintId = createJoint(aId, bId, pivotA, pivotB);
        }

        assert getConstraintType(constraintId) == 3;
        setNativeId(constraintId);
    }
    // *************************************************************************
    // native private methods

    native private static long createJoint(
            long bodyIdA, long bodyIdB, Vector3f pivotInA, Vector3f pivotInB);

    native private static long createJoint1(long bodyIdA, Vector3f pivotInA);

    native private static float getDamping(long jointId);

    native private static float getImpulseClamp(long jointId);

    native private static void getPivotInA(long jointId, Vector3f storeVector);

    native private static void getPivotInB(long jointId, Vector3f storeVector);

    native private static float getTau(long jointId);

    native private static void setDamping(long jointId, float desiredDamping);

    native private static void
            setImpulseClamp(long jointId, float desiredClamp);

    native private static void setPivotInA(long jointId, Vector3f pivotInA);

    native private static void setPivotInB(long jointId, Vector3f pivotInB);

    native private static void setTau(long jointId, float desiredTau);
}
