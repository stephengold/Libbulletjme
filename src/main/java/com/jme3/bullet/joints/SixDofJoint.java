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

import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;

/**
 * A 6 degree-of-freedom joint based on Bullet's btGeneric6DofConstraint. Axis
 * rotations are applied in XYZ order.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * <p>
 * For each axis:<ul>
 * <li>Lowerlimit = Upperlimit &rarr; axis is locked</li>
 * <li>Lowerlimit &gt; Upperlimit &rarr; axis is free</li>
 * <li>Lowerlimit &lt; Upperlimit &rarr; axis is limited in that range</li>
 * </ul>
 * <p>
 * It is recommended to use the btGeneric6DofSpring2Constraint, it has some
 * improvements over the original btGeneric6Dof(Spring)Constraint.
 *
 * @see com.jme3.bullet.joints.New6Dof
 * @author normenhansen
 */
public class SixDofJoint extends Constraint {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SixDofJoint.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the joint orientation: in physics-space coordinates if bodyA is
     * null, or else in A's local coordinates (rotation matrix)
     */
    final private Matrix3f rotA;
    /**
     * copy of the joint orientation in B's local coordinates (rotation matrix)
     */
    final private Matrix3f rotB;
    /**
     * true&rarr;limits give the allowable range of movement of frameB in frameA
     * space, false&rarr;limits give the allowable range of movement of frameA
     * in frameB space
     */
    final private boolean useLinearReferenceFrameA;
    /**
     * rotational motor for each axis
     */
    private RotationalLimitMotor[] rotationalMotors;
    /**
     * translational motor
     */
    private TranslationalLimitMotor translationalMotor;
    /**
     * upper limits for rotation of all 3 axes
     */
    final private Vector3f angularUpperLimit = new Vector3f(0f, 0f, 0f);
    /**
     * lower limits for rotation of all 3 axes
     */
    final private Vector3f angularLowerLimit = new Vector3f(0f, 0f, 0f);
    /**
     * upper limit for translation of all 3 axes
     */
    final private Vector3f linearUpperLimit = new Vector3f(0f, 0f, 0f);
    /**
     * lower limits for translation of all 3 axes
     */
    final private Vector3f linearLowerLimit = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a single-ended SixDofJoint.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyB the body to constrain (not null, alias created)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInWorld the pivot location in physics-space coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates
     * (rotation matrix, not null, unaffected)
     * @param rotInWorld the orientation of the joint in physics-space
     * coordinates (rotation matrix, not null, unaffected)
     * @param linearReferenceFrame which end to use as the linear reference
     * frame (not null)
     */
    public SixDofJoint(PhysicsRigidBody rigidBodyB, Vector3f pivotInB,
            Vector3f pivotInWorld, Matrix3f rotInB, Matrix3f rotInWorld,
            JointEnd linearReferenceFrame) {
        super(rigidBodyB, JointEnd.B, pivotInB, pivotInWorld);

        this.useLinearReferenceFrameA = (linearReferenceFrame == JointEnd.A);
        this.rotA = rotInWorld.clone();
        this.rotB = rotInB.clone();
        createJoint();
    }

    /**
     * Instantiate a double-ended SixDofJoint.
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
     * @param rotInA the joint orientation in A's local coordinates (rotation
     * matrix, unaffected)
     * @param rotInB the joint orientation in B's local coordinates (rotation
     * matrix, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     */
    public SixDofJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB, Matrix3f rotInA,
            Matrix3f rotInB, boolean useLinearReferenceFrameA) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);

        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        this.rotA = rotInA.clone();
        this.rotB = rotInB.clone();
        createJoint();
    }

    /**
     * Instantiate a double-ended SixDofJoint.
     * <p>
     * To be effective, the joint must be added to the body's PhysicsSpace and
     * the body must be dynamic.
     *
     * @param rigidBodyA the first body to constrain (not null, alias created)
     * @param rigidBodyB the 2nd body to constrain (not null, alias created)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     */
    public SixDofJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f pivotInA, Vector3f pivotInB,
            boolean useLinearReferenceFrameA) {
        super(rigidBodyA, rigidBodyB, pivotInA, pivotInB);

        this.useLinearReferenceFrameA = useLinearReferenceFrameA;
        this.rotA = new Matrix3f();
        this.rotB = new Matrix3f();
        createJoint();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the joint's rotation angles.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the rotation angle for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAngles(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getAngles(constraintId, result);

        return result;
    }

    /**
     * Copy the joint's lower limits for rotation on all 3 axes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the lower limit for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAngularLowerLimit(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = angularLowerLimit.clone();
        } else {
            result = storeResult.set(angularLowerLimit);
        }

        return result;
    }

    /**
     * Copy the joint's upper limits for rotation on all 3 axes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the upper limit for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getAngularUpperLimit(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = angularUpperLimit.clone();
        } else {
            result = storeResult.set(angularUpperLimit);
        }

        return result;
    }

    /**
     * Copy the joint's frame transform relative to the specified end.
     *
     * @param end which end (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return the transform of the constraint space relative to the end
     */
    public Transform getFrameTransform(JointEnd end, Transform storeResult) {
        Transform result
                = (storeResult == null) ? new Transform() : storeResult;

        long constraintId = nativeId();
        switch (end) {
            case A:
                getFrameOffsetA(constraintId, result);
                break;
            case B:
                getFrameOffsetB(constraintId, result);
                break;
            default:
                String message = "end = " + end;
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Copy the joint's lower limits for translation on all 3 axes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the lower limit for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getLinearLowerLimit(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = linearLowerLimit.clone();
        } else {
            result = storeResult.set(linearLowerLimit);
        }

        return result;
    }

    /**
     * Copy the joint's upper limits for translation on all 3 axes.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the upper limit for each local axis (in radians, either
     * storeResult or a new vector, not null)
     */
    public Vector3f getLinearUpperLimit(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = linearUpperLimit.clone();
        } else {
            result = storeResult.set(linearUpperLimit);
        }

        return result;
    }

    /**
     * Copy the joint's pivot offset.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the relative pivot position on each local axis (in physics-space
     * units, either storeResult or a new vector, not null)
     */
    public Vector3f getPivotOffset(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long constraintId = nativeId();
        getPivotOffset(constraintId, result);

        return result;
    }

    /**
     * Access the indexed RotationalLimitMotor of this joint, the motor which
     * influences rotation around the indexed axis.
     *
     * @param axisIndex the axis index of the desired motor: 0&rarr;X, 1&rarr;Y,
     * 2&rarr;Z
     * @return the pre-existing instance
     */
    public RotationalLimitMotor getRotationalLimitMotor(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");
        return rotationalMotors[axisIndex];
    }

    /**
     * Access the TranslationalLimitMotor of this joint, the motor which
     * influences translation on all 3 axes.
     *
     * @return the pre-existing instance
     */
    public TranslationalLimitMotor getTranslationalLimitMotor() {
        return translationalMotor;
    }

    /**
     * Alter the joint's lower limits for rotation of all 3 axes.
     *
     * @param limits the desired lower limits (in radians, not null, unaffected)
     */
    public void setAngularLowerLimit(Vector3f limits) {
        Validate.inRange(limits.x, "limits.x", -FastMath.PI, FastMath.PI);
        Validate.inRange(
                limits.y, "limits.y", -FastMath.HALF_PI, FastMath.HALF_PI);
        Validate.inRange(limits.z, "limits.z", -FastMath.PI, FastMath.PI);

        angularLowerLimit.set(limits);
        long constraintId = nativeId();
        setAngularLowerLimit(constraintId, limits);
    }

    /**
     * Alter the joint's upper limits for rotation of all 3 axes.
     *
     * @param limits the desired upper limits (in radians, not null, unaffected)
     */
    public void setAngularUpperLimit(Vector3f limits) {
        Validate.inRange(limits.x, "limits.x", -FastMath.PI, FastMath.PI);
        Validate.inRange(
                limits.y, "limits.y", -FastMath.HALF_PI, FastMath.HALF_PI);
        Validate.inRange(limits.z, "limits.z", -FastMath.PI, FastMath.PI);

        angularUpperLimit.set(limits);
        long constraintId = nativeId();
        setAngularUpperLimit(constraintId, limits);
    }

    /**
     * Alter the joint's lower limits for translation of all 3 axes.
     *
     * @param vector the desired lower limits (not null, unaffected)
     */
    public void setLinearLowerLimit(Vector3f vector) {
        linearLowerLimit.set(vector);
        long constraintId = nativeId();
        setLinearLowerLimit(constraintId, vector);
    }

    /**
     * Alter the joint's upper limits for translation of all 3 axes.
     *
     * @param vector the desired upper limits (not null, unaffected)
     */
    public void setLinearUpperLimit(Vector3f vector) {
        linearUpperLimit.set(vector);
        long constraintId = nativeId();
        setLinearUpperLimit(constraintId, vector);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Create a double-ended {@code btGeneric6DofConstraint}.
     *
     * @param bodyIdA the ID of the body for the A end (not zero)
     * @param bodyIdB the ID of the body for the B end (not zero)
     * @param pivotInA the pivot location in A's scaled local coordinates (not
     * null, unaffected)
     * @param rotInA the orientation of the joint in A's local coordinates (not
     * null, unaffected)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameA true&rarr;use body A, false&rarr;use body
     * B
     * @return the ID of the new joint
     */
    native protected long createJoint(long bodyIdA, long bodyIdB,
            Vector3f pivotInA, Matrix3f rotInA, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameA);

    /**
     * Create a single-ended {@code btGeneric6DofConstraint}.
     *
     * @param bodyIdB the ID of the body for the B end (not zero)
     * @param pivotInB the pivot location in B's scaled local coordinates (not
     * null, unaffected)
     * @param rotInB the orientation of the joint in B's local coordinates (not
     * null, unaffected)
     * @param useLinearReferenceFrameB true&rarr;use body A, false&rarr;use body
     * B
     * @return the ID of the new joint
     */
    native protected long createJoint1(long bodyIdB, Vector3f pivotInB,
            Matrix3f rotInB, boolean useLinearReferenceFrameB);
    // *************************************************************************
    // private methods

    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        PhysicsRigidBody a = getBodyA();
        assert pivotA != null;
        assert rotA != null;

        PhysicsRigidBody b = getBodyB();
        long bId = b.nativeId();
        assert pivotB != null;
        assert rotB != null;

        long constraintId;
        if (a == null) {
            /*
             * Create a single-ended joint.  Bullet assumes single-ended
             * btGeneric6DofConstraints are satisfied at creation, so we
             * temporarily re-position the body to satisfy the constraint.
             */
            Transform jInWorld = new Transform();
            jInWorld.getRotation().fromRotationMatrix(rotA);
            jInWorld.setTranslation(pivotA);

            Transform jInB = new Transform();
            jInB.getRotation().fromRotationMatrix(rotB);
            jInB.setTranslation(pivotB);

            Transform bToWorld = jInB.invert();
            MyMath.combine(bToWorld, jInWorld, bToWorld);

            Vector3f saveLocation = b.getPhysicsLocation(null);
            Quaternion saveRotation = b.getPhysicsRotation(null);

            b.setPhysicsLocation(bToWorld.getTranslation());
            b.setPhysicsRotation(bToWorld.getRotation());

            boolean useLinearReferenceFrameB = !useLinearReferenceFrameA;
            constraintId
                    = createJoint1(bId, pivotB, rotB, useLinearReferenceFrameB);

            b.setPhysicsLocation(saveLocation);
            b.setPhysicsRotation(saveRotation);

        } else { // Create a double-ended joint.
            long aId = a.nativeId();
            constraintId = createJoint(aId, bId, pivotA, rotA, pivotB, rotB,
                    useLinearReferenceFrameA);
        }

        int constraintType = getConstraintType(constraintId);
        assert constraintType == 6 || constraintType == 9;

        setNativeId(constraintId);
        gatherMotors();
    }

    private void gatherMotors() {
        assert rotationalMotors == null;
        assert translationalMotor == null;

        long constraintId = nativeId();
        this.rotationalMotors = new RotationalLimitMotor[numAxes];

        for (int axisIndex = 0; axisIndex < numAxes; ++axisIndex) {
            long motorId = getRotationalLimitMotor(constraintId, axisIndex);
            rotationalMotors[axisIndex] = new RotationalLimitMotor(motorId);
        }

        long motorId = getTranslationalLimitMotor(constraintId);
        this.translationalMotor = new TranslationalLimitMotor(motorId);
    }
    // *************************************************************************
    // native private methods

    native private static void getAngles(long jointId, Vector3f storeVector);

    native private static void
            getFrameOffsetA(long jointId, Transform frameInA);

    native private static void
            getFrameOffsetB(long jointId, Transform frameInB);

    native private static void
            getPivotOffset(long jointId, Vector3f storeVector);

    native private static long getRotationalLimitMotor(long jointId, int index);

    native private static long getTranslationalLimitMotor(long jointId);

    native private static void
            setAngularLowerLimit(long jointId, Vector3f limits);

    native private static void
            setAngularUpperLimit(long jointId, Vector3f limits);

    native private static void
            setLinearLowerLimit(long jointId, Vector3f limits);

    native private static void
            setLinearUpperLimit(long jointId, Vector3f limits);
}
