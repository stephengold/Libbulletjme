/*
 * Copyright (c) 2009-2015 jMonkeyEngine
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

import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Physics-simulation parameters that can be customized for each
 * PhysicsSoftBody, based on Bullet's {@code btSoftBodyWorldInfo}.
 * <p>
 * NOTE: When a PhysicsSoftBody is added to a PhysicsSoftSpace, it acquires the
 * SoftBodyWorldInfo of that space. To customize a body, assign it a new info
 * after adding it to the space.
 *
 * @author dokthar
 */
public class SoftBodyWorldInfo extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SoftBodyWorldInfo.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate an info that refers to a new btSoftBodyWorldInfo with the
     * default parameters.
     */
    public SoftBodyWorldInfo() {
        long infoId = createSoftBodyWorldInfo();
        super.setNativeId(infoId);
    }

    /**
     * Instantiate an info that refers to the identified native object. Used
     * internally.
     *
     * @param nativeId the ID of a pre-existing btSoftBodyWorldInfo (not zero)
     */
    public SoftBodyWorldInfo(long nativeId) {
        Validate.nonZero(nativeId, "native ID");
        super.setNativeIdNotTracked(nativeId);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the air density.
     *
     * @return the density
     */
    public float airDensity() {
        long infoId = nativeId();
        float result = getAirDensity(infoId);

        return result;
    }

    /**
     * Copy all parameter values from the specified info.
     *
     * @param source the info to copy from (not null, unaffected)
     */
    public void copyAll(SoftBodyWorldInfo source) {
        long thisId = nativeId();
        long sourceId = source.nativeId();
        setSoftBodyWorldInfo(thisId, sourceId);
    }

    /**
     * Copy the gravitational acceleration.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return an acceleration vector in physics-space coordinates (either
     * storeResult or a new vector, not null)
     */
    public Vector3f copyGravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long infoId = nativeId();
        getGravity(infoId, result);

        return result;
    }

    /**
     * Copy the normal direction of the water surface.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a direction vector (in physics-space coordinates, either
     * storeResult or a new vector, not null)
     */
    public Vector3f copyWaterNormal(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long infoId = nativeId();
        getWaterNormal(infoId, result);

        return result;
    }

    /**
     * Return the maximum distance a node can travel in a simulation step.
     *
     * @return the distance (in physics-space units)
     */
    public float maxDisplacement() {
        long infoId = nativeId();
        float result = getMaxDisplacement(infoId);

        return result;
    }

    /**
     * Alter the air density.
     *
     * @param density the desired density (default=1.2)
     */
    public void setAirDensity(float density) {
        long infoId = nativeId();
        setAirDensity(infoId, density);
    }

    /**
     * Alter the gravitational acceleration.
     *
     * @param acceleration the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected, default=(0,-10,0))
     */
    public void setGravity(Vector3f acceleration) {
        long infoId = nativeId();
        setGravity(infoId, acceleration);
    }

    /**
     * Alter the maximum distance a node can travel per simulation step.
     *
     * @param maxDisplacement the desired value (&gt;0, default=1000)
     */
    public void setMaxDisplacement(float maxDisplacement) {
        Validate.positive(maxDisplacement, "max displacement");

        long infoId = nativeId();
        setMaxDisplacement(infoId, maxDisplacement);
    }

    /**
     * Alter the water density.
     *
     * @param density the desired density (default=0)
     */
    public void setWaterDensity(float density) {
        long infoId = nativeId();
        setWaterDensity(infoId, density);
    }

    /**
     * Alter the water normal.
     *
     * @param normalDirection the desired normal direction (not null,
     * unaffected, default=(0,0,0))
     */
    public void setWaterNormal(Vector3f normalDirection) {
        long infoId = nativeId();
        setWaterNormal(infoId, normalDirection);
    }

    /**
     * Alter the water offset.
     *
     * @param offset the desired offset distance (in physics-space units,
     * default=0)
     */
    public void setWaterOffset(float offset) {
        long infoId = nativeId();
        setWaterOffset(infoId, offset);
    }

    /**
     * Return the water density.
     *
     * @return the density
     */
    public float waterDensity() {
        long infoId = nativeId();
        float result = getWaterDensity(infoId);

        return result;
    }

    /**
     * Return the water offset.
     *
     * @return the offset distance (in physics-space units)
     */
    public float waterOffset() {
        long infoId = nativeId();
        float result = getWaterOffset(infoId);

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param infoId the native identifier (not zero)
     */
    private static void freeNativeObject(long infoId) {
        assert infoId != 0L;
        finalizeNative(infoId);
    }
    // *************************************************************************
    // native private methods

    native private static long createSoftBodyWorldInfo();

    native private static void finalizeNative(long infoId);

    native private static float getAirDensity(long infoId);

    native private static void getGravity(long infoId, Vector3f storeVector);

    native private static float getMaxDisplacement(long infoId);

    native private static float getWaterDensity(long infoId);

    native private static void
            getWaterNormal(long infoId, Vector3f storeVector);

    native private static float getWaterOffset(long infoId);

    native private static void setAirDensity(long infoId, float density);

    native private static void setGravity(long infoId, Vector3f gravityVector);

    native private static void
            setMaxDisplacement(long infoId, float displacement);

    native private static void
            setSoftBodyWorldInfo(long targetId, long sourceId);

    native private static void setWaterDensity(long infoId, float density);

    native private static void
            setWaterNormal(long infoId, Vector3f normalVector);

    native private static void setWaterOffset(long infoId, float offset);
}
