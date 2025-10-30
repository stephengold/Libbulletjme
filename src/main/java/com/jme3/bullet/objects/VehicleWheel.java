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

import com.jme3.bullet.objects.infos.VehicleTuning;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Information about one wheel of a vehicle, based on Bullet's
 * {@code btWheelInfo}.
 *
 * @author normenhansen
 */
public class VehicleWheel {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VehicleWheel.class.getName());
    // *************************************************************************
    // fields

    /**
     * true &rarr; physics coordinates match local transform, false &rarr;
     * physics coordinates match world transform
     */
    private boolean applyLocal = false;
    /**
     * copy of wheel type: true&rarr;front (steering) wheel,
     * false&rarr;non-front wheel
     */
    private boolean isFront;
    /**
     * copy of wheel radius (in physics-space units, &gt;0)
     */
    private float radius = 0.5f;
    /**
     * copy of rest length of the suspension (in physics-space units)
     */
    private float restLength = 1f;
    /**
     * copy of roll-influence factor (0&rarr;no roll torque, 1&rarr;realistic
     * behavior)
     */
    private float rollInfluence = 1f;
    /**
     * 0-origin index among the vehicle's wheels (&ge;0)
     */
    private int wheelIndex = 0;
    /**
     * unique identifier of the btRaycastVehicle
     */
    private long vehicleId = 0L;
    /**
     * reusable rotation matrix
     */
    final private Matrix3f tmpMatrix = new Matrix3f();
    /**
     * wheel orientation in physics-space coordinates
     */
    final private Quaternion wheelWorldRotation = new Quaternion();
    /**
     * axis direction (in chassis coordinates, typically to the right/-1,0,0)
     */
    final private Vector3f axisDirection = new Vector3f();
    /**
     * location where the suspension connects to the chassis (in chassis
     * coordinates)
     */
    final private Vector3f location = new Vector3f();
    /**
     * suspension direction (in chassis coordinates, typically down/0,-1,0)
     */
    final private Vector3f suspensionDirection = new Vector3f();
    /**
     * wheel location in physics-space coordinates
     */
    final private Vector3f wheelWorldLocation = new Vector3f();
    /**
     * copy of tuning parameters
     */
    final private VehicleTuning tuning = new VehicleTuning();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a wheel.
     *
     * @param location the location where the suspension connects to the chassis
     * (in chassis coordinates, not null, unaffected)
     * @param direction the suspension direction (in chassis coordinates, not
     * null, unaffected, typically down/0,-1,0)
     * @param axle the axis direction (in chassis coordinates, not null,
     * unaffected, typically right/-1,0,0)
     * @param restLength the rest length of the suspension (in physics-space
     * units)
     * @param radius the wheel's radius (in physics-space units, &ge;0)
     * @param frontWheel true&rarr;front (steering) wheel, false&rarr;non-front
     * wheel
     */
    public VehicleWheel(Vector3f location, Vector3f direction,
            Vector3f axle, float restLength, float radius, boolean frontWheel) {
        Validate.positive(radius, "radius");

        this.location.set(location);
        this.suspensionDirection.set(direction);
        this.axisDirection.set(axle);
        this.isFront = frontWheel;
        this.restLength = restLength;
        this.radius = radius;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Compare Bullet's values to the local copies.
     *
     * @return true if the values are exactly equal, otherwise false
     */
    boolean checkCopies() {
        boolean nativeIsFront = isFront(vehicleId, wheelIndex);
        boolean result = (nativeIsFront == isFront);

        if (result) {
            float nativeRadius = getRadius(vehicleId, wheelIndex);
            result = (nativeRadius == radius);
        }

        if (result) {
            float nativeRestLength = getRestLength(vehicleId, wheelIndex);
            result = (nativeRestLength == restLength);
        }

        if (result) {
            float nativeRollInfluence = getRollInfluence(vehicleId, wheelIndex);
            result = (nativeRollInfluence == rollInfluence);
        }

        return result;
    }

    /**
     * Determine this wheel's axis direction.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a new unit vector (in chassis coordinates, either storeResult or
     * a new instance)
     */
    public Vector3f getAxle(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = axisDirection.clone();
        } else {
            result = storeResult.set(axisDirection);
        }

        return result;
    }

    /**
     * Determine this wheel's braking impulse (native field: m_brake).
     *
     * @return the amount of impulse
     */
    public float getBrake() {
        float result = getBrake(vehicleId, wheelIndex);
        return result;
    }

    /**
     * Determine the location where the wheel touches the ground.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getCollisionLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getCollisionLocation(vehicleId, wheelIndex, result);
        return result;
    }

    /**
     * Determine the normal direction where the wheel touches the ground.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector (in physics-space coordinates, either storeResult
     * or a new instance)
     */
    public Vector3f getCollisionNormal(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        getCollisionNormal(vehicleId, wheelIndex, result);
        return result;
    }

    /**
     * Determine how much this wheel has turned since the last simulation step.
     *
     * @return the rotation angle (in radians)
     */
    public float getDeltaRotation() {
        return getDeltaRotation(vehicleId, wheelIndex);
    }

    /**
     * Determine this wheel's suspension direction.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector (in chassis coordinates, either storeResult or a
     * new instance)
     */
    public Vector3f getDirection(Vector3f storeResult) {
        if (storeResult == null) {
            return suspensionDirection.clone();
        } else {
            return storeResult.set(suspensionDirection);
        }
    }

    /**
     * Determine this wheel's engine force (native field: m_engineForce).
     *
     * @return the amount of force applied (TODO positive direction?)
     */
    public float getEngineForce() {
        float result = getEngineForce(vehicleId, wheelIndex);
        return result;
    }

    /**
     * Determine the friction between this wheel's tire and the ground (native
     * field: m_frictionSlip).
     *
     * @return the coefficient of friction
     */
    public float getFrictionSlip() {
        float result = tuning.getFrictionSlip();
        return result;
    }

    /**
     * Determine the index of this wheel, based on creation order.
     *
     * @return the zero-origin index (&ge;0)
     */
    public int getIndex() {
        return wheelIndex;
    }

    /**
     * Determine the location where the suspension connects to the chassis.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a new location vector (in chassis coordinates, either storeResult
     * or a new instance)
     */
    public Vector3f getLocation(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = location.clone();
        } else {
            result = storeResult.set(location);
        }

        return result;
    }

    /**
     * Determine the maximum force exerted by this wheel's suspension (native
     * field: m_maxSuspensionForce).
     *
     * @return the maximum force
     */
    public float getMaxSuspensionForce() {
        float result = tuning.getMaxSuspensionForce();
        return result;
    }

    /**
     * Determine the maximum travel distance for this wheel's suspension (native
     * field: m_maxSuspensionTravelCm).
     *
     * Note that the units are centimeters ONLY if the physics-space unit is
     * exactly one meter.
     *
     * @return the maximum amount the suspension can be compressed or expanded,
     * relative to its rest length (in hundredths of a physics-space unit)
     */
    public float getMaxSuspensionTravelCm() {
        float result = tuning.getMaxSuspensionTravelCm();
        return result;
    }

    /**
     * Determine the radius of this wheel (native field: m_wheelsRadius).
     *
     * @return the radius (in physics-space units, &ge;0)
     */
    public float getRadius() {
        return radius;
    }

    /**
     * Determine the rest length of this wheel (native field:
     * m_suspensionRestLength1).
     *
     * @return the length
     */
    public float getRestLength() {
        return restLength;
    }

    /**
     * Determine this wheel's roll influence (native field: m_rollInfluence).
     *
     * @return the roll-influence factor
     */
    public float getRollInfluence() {
        return rollInfluence;
    }

    /**
     * Determine the total rotation of this wheel (native field: m_rotation).
     *
     * @return the angle (in radians)
     */
    public float getRotationAngle() {
        float result = getRotationAngle(vehicleId, wheelIndex);
        return result;
    }

    /**
     * Determine to what extent the wheel is skidding (for skid sounds/smoke
     * etcetera). Don't bother invoking this if the wheel is unsupported---in
     * other words, if {@link PhysicsVehicle#castRay(int)} is negative!
     *
     * @return the relative amount of traction (0&rarr;wheel is sliding,
     * 1&rarr;wheel has full traction)
     */
    public float getSkidInfo() {
        float result = getSkidInfo(vehicleId, wheelIndex);
        return result;
    }

    /**
     * Determine this wheel's steering angle (native field: m_steering).
     *
     * @return angle (in radians, 0=straight, left is positive)
     */
    public float getSteerAngle() {
        float result = getSteerAngle(vehicleId, wheelIndex);
        return result;
    }

    /**
     * Determine the length of this wheel's suspension (native field:
     * m_suspensionLength).
     *
     * @return the length (in physics-space units)
     */
    public float getSuspensionLength() {
        float result = getSuspensionLength(vehicleId, wheelIndex);
        return result;
    }

    /**
     * Determine the stiffness of this wheel's suspension (native field:
     * m_suspensionStiffness).
     *
     * @return the stiffness constant
     */
    public float getSuspensionStiffness() {
        float result = tuning.getSuspensionStiffness();
        return result;
    }

    /**
     * Determine this wheel's damping when the suspension is compressed (native
     * field: m_wheelsDampingCompression).
     *
     * @return the damping
     */
    public float getWheelsDampingCompression() {
        float result = tuning.getSuspensionCompression();
        return result;
    }

    /**
     * Determine this wheel's damping when the suspension is expanded (native
     * field: m_wheelsDampingRelaxation).
     *
     * @return the damping
     */
    public float getWheelsDampingRelaxation() {
        float result = tuning.getSuspensionDamping();
        return result;
    }

    /**
     * Determine this wheel's location.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new instance)
     */
    public Vector3f getWheelWorldLocation(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = wheelWorldLocation.clone();
        } else {
            result = storeResult.set(wheelWorldLocation);
        }

        return result;
    }

    /**
     * Determine this wheel's orientation.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a Quaternion (in physics-space coordinates, either storeResult or
     * a new instance)
     */
    public Quaternion getWheelWorldRotation(Quaternion storeResult) {
        Quaternion result;
        if (storeResult == null) {
            result = wheelWorldRotation.clone();
        } else {
            result = storeResult.set(wheelWorldRotation);
        }

        return result;
    }

    /**
     * Test whether physics coordinates should match the local transform of the
     * Spatial.
     *
     * @return true if matching local transform, false if matching world
     * transform
     */
    public boolean isApplyLocal() {
        return applyLocal;
    }

    /**
     * Test whether this wheel is a front (steering) wheel (native field:
     * m_bIsFrontWheel).
     *
     * @return true if front wheel, otherwise false
     */
    public boolean isFrontWheel() {
        return isFront;
    }

    /**
     * Alter whether physics coordinates should match the local transform of the
     * Spatial.
     *
     * @param applyLocal true&rarr;match local transform, false&rarr;match world
     * transform (default=false)
     */
    public void setApplyLocal(boolean applyLocal) {
        this.applyLocal = applyLocal;
    }

    /**
     * Alter the friction between this wheel's tire and the ground (native
     * field: m_frictionSlip).
     * <p>
     * Should be about 0.8 for realistic cars, but can be increased for better
     * handling. Set large (10000) for kart racers.
     *
     * @param coeff the desired coefficient of friction (default=10.5)
     */
    public void setFrictionSlip(float coeff) {
        tuning.setFrictionSlip(coeff);
        applyInfo();
    }

    /**
     * Alter whether this wheel is a front (steering) wheel (native field:
     * m_bIsFrontWheel).
     *
     * @param frontWheel true&rarr;front wheel, false&rarr;non-front wheel
     */
    public void setFrontWheel(boolean frontWheel) {
        this.isFront = frontWheel;
        applyInfo();
    }

    /**
     * Alter the maximum force exerted by this wheel's suspension (native field:
     * m_maxSuspensionForce).
     * <p>
     * Increase this if your suspension cannot handle the weight of your
     * vehicle.
     *
     * @param maxForce the desired maximum force (default=6000)
     */
    public void setMaxSuspensionForce(float maxForce) {
        tuning.setMaxSuspensionForce(maxForce);
        applyInfo();
    }

    /**
     * Alter the maximum travel distance for this wheel's suspension (native
     * field: m_maxSuspensionTravelCm).
     *
     * Note that the units are centimeters ONLY if the physics-space unit is
     * exactly one meter.
     *
     * @param travelCm the desired maximum amount the suspension can be
     * compressed or expanded, relative to its rest length (in hundredths of a
     * physics-space unit, default=500)
     */
    public void setMaxSuspensionTravelCm(float travelCm) {
        tuning.setMaxSuspensionTravelCm(travelCm);
        applyInfo();
    }

    /**
     * Alter the radius of this wheel (native field: m_wheelsRadius).
     *
     * @param radius the desired radius (in physics-space units, &ge;0,
     * default=0.5)
     */
    public void setRadius(float radius) {
        this.radius = radius;
        applyInfo();
    }

    /**
     * Alter the rest length of the suspension of this wheel (native field:
     * m_suspensionRestLength1).
     *
     * @param restLength the desired length (default=1)
     */
    public void setRestLength(float restLength) {
        this.restLength = restLength;
        applyInfo();
    }

    /**
     * Alter this wheel's roll influence (native field: m_rollInfluence).
     * <p>
     * The roll-influence factor reduces (or magnifies) the torque contributed
     * by this wheel that tends to cause the vehicle to roll over. This is a bit
     * of a hack, but it's quite effective.
     * <p>
     * If the friction between the tires and the ground is too high, you may
     * reduce this factor to prevent the vehicle from rolling over. You should
     * also try lowering the vehicle's center of mass.
     *
     * @param rollInfluence the desired roll-influence factor (0&rarr;no roll
     * torque, 1&rarr;realistic behavior, default=1)
     */
    public void setRollInfluence(float rollInfluence) {
        this.rollInfluence = rollInfluence;
        applyInfo();
    }

    /**
     * Alter the total rotation of this wheel (native field: m_rotation).
     *
     * @param angle the desired angle (in radians)
     */
    public void setRotationAngle(float angle) {
        setRotationAngle(vehicleId, wheelIndex, angle);
    }

    /**
     * Alter the length of this wheel's suspension (native field:
     * m_suspensionLength). Bullet updates the length during every simulation
     * step.
     *
     * @param length the desired length (in physics-space units)
     */
    public void setSuspensionLength(float length) {
        setSuspensionLength(vehicleId, wheelIndex, length);
    }

    /**
     * Alter the stiffness of this wheel's suspension (native field:
     * m_suspensionStiffness).
     *
     * @param stiffness the desired stiffness constant (10&rarr;off-road buggy,
     * 50&rarr;sports car, 200&rarr;Formula-1 race car, default=5.88)
     */
    public void setSuspensionStiffness(float stiffness) {
        tuning.setSuspensionStiffness(stiffness);
        applyInfo();
    }

    /**
     * Assign this wheel to a vehicle.
     *
     * @param vehicleId the ID of the btRaycastVehicle (not zero)
     * @param wheelIndex index among the vehicle's wheels (&ge;0)
     */
    public void setVehicleId(long vehicleId, int wheelIndex) {
        Validate.nonZero(vehicleId, "vehicle ID");
        Validate.nonNegative(wheelIndex, "wheel index");

        this.vehicleId = vehicleId;
        this.wheelIndex = wheelIndex;
        applyInfo();
    }

    /**
     * Alter this wheel's damping when the suspension is compressed (native
     * field: m_wheelsDampingCompression).
     * <p>
     * Set to k * 2 * FastMath.sqrt(m_suspensionStiffness) where k is the
     * damping ratio:
     * <p>
     * k = 0.0 undamped and bouncy, k = 1.0 critical damping, k between 0.1 and
     * 0.3 are good values
     *
     * @param damping the desired damping (0&rarr;no damping, default=0.83)
     */
    public void setWheelsDampingCompression(float damping) {
        tuning.setSuspensionCompression(damping);
        applyInfo();
    }

    /**
     * Alter this wheel's damping when the suspension is expanded (native field:
     * m_wheelsDampingRelaxation).
     * <p>
     * Set to k * 2 * FastMath.sqrt(m_suspensionStiffness) where k is the
     * damping ratio:
     * <p>
     * k=0: undamped and bouncy, k=1: critical damping, k between 0.1 and 0.3
     * are good values
     *
     * @param wheelsDampingRelaxation the desired damping (default=0.88)
     */
    public void setWheelsDampingRelaxation(float wheelsDampingRelaxation) {
        tuning.setSuspensionDamping(wheelsDampingRelaxation);
        applyInfo();
    }

    /**
     * Update this wheel's location and orientation.
     */
    public void updatePhysicsState() {
        getWheelLocation(vehicleId, wheelIndex, wheelWorldLocation);
        getWheelRotation(vehicleId, wheelIndex, tmpMatrix);
        wheelWorldRotation.fromRotationMatrix(tmpMatrix);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Apply the cached tuning parameters to the native object.
     */
    private void applyInfo() {
        if (vehicleId != 0L) {
            applyInfo(vehicleId, wheelIndex,
                    getSuspensionStiffness(),
                    getWheelsDampingRelaxation(),
                    getWheelsDampingCompression(),
                    getFrictionSlip(),
                    rollInfluence,
                    getMaxSuspensionTravelCm(),
                    getMaxSuspensionForce(),
                    radius,
                    isFront,
                    restLength);
            assert checkCopies();
        }
    }
    // *************************************************************************
    // native private methods

    native private static void applyInfo(long vehicleId, int wheelIndex,
            float suspensionStiffness,
            float wheelsDampingRelaxation,
            float wheelsDampingCompression,
            float frictionSlip,
            float rollInfluence,
            float maxSuspensionTravelCm,
            float maxSuspensionForce,
            float wheelsRadius,
            boolean frontWheel,
            float suspensionRestLength);

    native private static float getBrake(long vehicleId, int wheelIndex);

    native private static void getCollisionLocation(
            long vehicleId, int wheelIndex, Vector3f vector);

    native private static void
            getCollisionNormal(long vehicleId, int wheelIndex, Vector3f vector);

    native private static float
            getDeltaRotation(long vehicleId, int wheelIndex);

    native private static float getEngineForce(long vehicleId, int wheelIndex);

    native private static float getRadius(long vehicleId, int wheelIndex);

    native private static float getRestLength(long vehicleId, int wheelIndex);

    native private static float
            getRollInfluence(long vehicleId, int wheelIndex);

    native private static float
            getRotationAngle(long vehicleId, int wheelIndex);

    native private static float getSkidInfo(long vehicleId, int wheelIndex);

    native private static float getSteerAngle(long vehicleId, int wheelIndex);

    native private static float
            getSuspensionLength(long vehicleId, int wheelIndex);

    native private static void
            getWheelLocation(long vehicleId, int wheelIndex, Vector3f vector);

    native private static void
            getWheelRotation(long vehicleId, int wheelIndex, Matrix3f matrix);

    native private static boolean isFront(long vehicleId, int wheelIndex);

    native private static void
            setRotationAngle(long vehicleId, int wheelIndex, float angle);

    native private static void
            setSuspensionLength(long vehicleId, int wheelIndex, float length);
}
