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
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.NativePhysicsObject;
import java.util.logging.Logger;

/**
 * Tuning parameters for a PhysicsVehicle, based on Bullet's btVehicleTuning.
 *
 * @author normenhansen
 */
public class VehicleTuning extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VehicleTuning.class.getName());
    // *************************************************************************
    // fields

    /**
     * coefficient of friction between tires and ground (0.8&rarr;realistic car,
     * 10000&rarr;kart racer)
     */
    private float frictionSlip = 10.5f;
    /**
     * maximum force exerted by each wheel's suspension
     */
    private float maxSuspensionForce = 6000f;
    /**
     * maximum suspension travel distance (in hundredths of a PSU)
     */
    private float maxSuspensionTravelCm = 500f;
    /**
     * suspension damping when compressed (0&rarr;no damping)
     */
    private float suspensionCompression = 0.83f;
    /**
     * suspension damping when expanded (0&rarr;no damping)
     */
    private float suspensionDamping = 0.88f;
    /**
     * suspension stiffness constant (10&rarr;off-road buggy, 50&rarr;sports
     * car, 200&rarr;Formula-1 race car)
     */
    private float suspensionStiffness = 5.88f;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an instance with the default parameter values.
     */
    public VehicleTuning() {
        create();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the friction between tires and ground (native field:
     * m_frictionSlip).
     *
     * @return the coefficient of friction
     */
    public float getFrictionSlip() {
        return frictionSlip;
    }

    /**
     * Return the maximum force exerted by each wheel's suspension (native
     * field: m_maxSuspensionForce).
     *
     * @return the maximum force
     */
    public float getMaxSuspensionForce() {
        return maxSuspensionForce;
    }

    /**
     * Determine the maximum travel distance for each wheel's suspension (native
     * field: m_maxSuspensionTravelCm).
     * <p>
     * Note that the units are centimeters ONLY if the physics-space unit is
     * exactly one meter.
     *
     * @return the maximum amount the suspension can be compressed or expanded,
     * relative to its rest length (in hundredths of a physics-space unit)
     */
    public float getMaxSuspensionTravelCm() {
        return maxSuspensionTravelCm;
    }

    /**
     * Return the suspension damping when compressed (native field:
     * m_suspensionCompression).
     *
     * @return the damping amount
     */
    public float getSuspensionCompression() {
        return suspensionCompression;
    }

    /**
     * Return the suspension damping when expanded (native field:
     * m_suspensionDamping).
     *
     * @return the damping amount (0&rarr;no damping, default=0.88)
     */
    public float getSuspensionDamping() {
        return suspensionDamping;
    }

    /**
     * Return the suspension stiffness (native field: m_suspensionStiffness).
     *
     * @return the stiffness constant
     */
    public float getSuspensionStiffness() {
        return suspensionStiffness;
    }

    /**
     * Alter the friction between tires and ground (native field:
     * m_frictionSlip).
     *
     * @param coeff the desired coefficient of friction (0.8&rarr;realistic car,
     * 10000&rarr;kart racer, default=10.5)
     */
    public void setFrictionSlip(float coeff) {
        this.frictionSlip = coeff;
        long tuningId = nativeId();
        setFrictionSlip(tuningId, coeff);
    }

    /**
     * Alter the force exerted by each wheel's suspension (native field:
     * m_maxSuspensionForce).
     *
     * @param maxForce the desired maximum force (default=6000)
     */
    public void setMaxSuspensionForce(float maxForce) {
        this.maxSuspensionForce = maxForce;
        long tuningId = nativeId();
        setMaxSuspensionForce(tuningId, maxForce);
    }

    /**
     * Alter the maximum travel distance for the suspension (native field:
     * m_maxSuspensionTravelCm).
     *
     * Note that the units are centimeters ONLY if the physics-space unit is
     * exactly one meter.
     *
     * @param travelCm the desired maximum amount the suspension can be
     * compressed or expanded, relative to its rest length (in hundredths of a
     * physics-space unit, default=500)
     */
    public void setMaxSuspensionTravelCm(float travelCm) {
        this.maxSuspensionTravelCm = travelCm;
        long tuningId = nativeId();
        setMaxSuspensionTravelCm(tuningId, travelCm);
    }

    /**
     * Alter the suspension damping when compressed (native field:
     * m_suspensionCompression).
     *
     * @param damping the desired damping amount (0&rarr;no damping,
     * default=0.83)
     */
    public void setSuspensionCompression(float damping) {
        this.suspensionCompression = damping;
        long tuningId = nativeId();
        setSuspensionCompression(tuningId, damping);
    }

    /**
     * Alter the suspension damping when expanded (native field:
     * m_suspensionDamping).
     *
     * @param damping the desired damping (0&rarr;no damping, default=0.88)
     */
    public void setSuspensionDamping(float damping) {
        this.suspensionDamping = damping;
        long tuningId = nativeId();
        setSuspensionDamping(tuningId, damping);
    }

    /**
     * Alter the stiffness of the suspension (native field:
     * m_suspensionStiffness).
     *
     * @param stiffness the desired stiffness constant (10&rarr;off-road buggy,
     * 50&rarr;sports car, 200&rarr;Formula-1 race car, default=5.88)
     */
    public void setSuspensionStiffness(float stiffness) {
        this.suspensionStiffness = stiffness;
        long tuningId = nativeId();
        setSuspensionStiffness(tuningId, stiffness);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured tuning in Bullet.
     */
    private void create() {
        long tuningId = createNative();
        setNativeId(tuningId);

        setFrictionSlip(tuningId, frictionSlip);
        setMaxSuspensionForce(tuningId, maxSuspensionForce);
        setMaxSuspensionTravelCm(tuningId, maxSuspensionTravelCm);
        setSuspensionCompression(tuningId, suspensionCompression);
        setSuspensionDamping(tuningId, suspensionDamping);
        setSuspensionStiffness(tuningId, suspensionStiffness);
    }

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param tuningId the native identifier (not zero)
     */
    private static void freeNativeObject(long tuningId) {
        assert tuningId != 0L;
        finalizeNative(tuningId);
    }
    // *************************************************************************
    // native private methods

    native private static long createNative();

    native private static void finalizeNative(long tuningId);

    native private static void setFrictionSlip(long tuningId, float slip);

    native private static void
            setMaxSuspensionForce(long tuningId, float maxForce);

    native private static void
            setMaxSuspensionTravelCm(long tuningId, float travel);

    native private static void
            setSuspensionCompression(long tuningId, float damping);

    native private static void
            setSuspensionDamping(long tuningId, float damping);

    native private static void
            setSuspensionStiffness(long tuningId, float stiff);
}
