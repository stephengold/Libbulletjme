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
package com.jme3.bullet.objects;

import com.jme3.bullet.SoftBodyWorldInfo;
import com.jme3.bullet.collision.PcoType;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.math.Vector3f;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A simplified soft body embedded in a rigid frame, based on Bullet's
 * btReducedDeformableBody.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ReducedDeformableBody extends PhysicsSoftBody {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(ReducedDeformableBody.class.getName());
    // *************************************************************************
    // fields

    /**
     * copies of the initial node masses (each &ge;0)
     */
    final private float[] masses;
    /**
     * copies of the initial node locations (each non-null)
     */
    final private Vector3f[] locations;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a deformable body with the specified nodes. The new body is
     * not added to any physics space.
     *
     * @param locations the initial node locations (not null, alias created)
     * @param masses the initial node masses (not null, each &ge;0, alias
     * created)
     */
    public ReducedDeformableBody(Vector3f[] locations, float[] masses) {
        super(false);
        Validate.nonNull(locations, "locations");
        int numNodes = masses.length;
        Validate.require(locations.length == numNodes,
                "arrays of equal length");

        this.locations = new Vector3f[numNodes];
        this.masses = new float[numNodes];
        for (int i = 0; i < numNodes; ++i) {
            this.locations[i] = locations[i].clone();
            assert masses[i] >= 0f : i;
            this.masses[i] = masses[i];
        }

        SoftBodyWorldInfo worldInfo = new SoftBodyWorldInfo();
        super.setWorldInfoInternal(worldInfo);

        long infoId = worldInfo.nativeId();
        long bodyId = create(infoId, locations, masses, numNodes);
        super.setNativeId(bodyId);
        assert getInternalType(bodyId) == PcoType.soft :
                getInternalType(bodyId);
        logger3.log(Level.FINE, "Created {0}.", this);

        SoftBodyConfig config = new SoftBodyConfig(this);
        super.setConfig(config);

        super.initUserPointer();

        float defaultMargin = CollisionShape.getDefaultMargin();
        setMargin(defaultMargin);

        assert !isInWorld();
        assert isEmpty();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the rigid velocity.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (physics-space units per second in
     * physics-space coordinates, either storeResult or a new vector)
     */
    public Vector3f getLinearVelocity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        long bodyId = nativeId();
        getLinearVelocity(bodyId, result);

        return result;
    }

    /**
     * Test whether reduced modes are enabled.
     *
     * @return true if enabled, false if rigid-only
     */
    public boolean isReducedModesEnabled() {
        long bodyId = nativeId();
        boolean result = isReducedModesEnabled(bodyId);

        return result;
    }

    /**
     * Pin a node.
     *
     * @param nodeIndex the index of the node (&ge;0)
     */
    public void pinNode(int nodeIndex) {
        Validate.nonNegative(nodeIndex, "node index");

        long bodyId = nativeId();
        pinNode(bodyId, nodeIndex);
    }

    /**
     * Alter the Rayleigh damping parameters.
     *
     * @param alpha the desired alpha value (default=0)
     * @param beta the desired beta value (default=0)
     */
    public void setDamping(float alpha, float beta) {
        long bodyId = nativeId();
        setDamping(bodyId, alpha, beta);
    }

    /**
     * Alter the rigid velocity.
     *
     * @param velocity the desired value (not null, unaffected)
     */
    public void setLinearVelocity(Vector3f velocity) {
        Validate.nonNull(velocity, "velocity");

        long bodyId = nativeId();
        setLinearVelocity(bodyId, velocity);
    }

    /**
     * Alter the number of modes.
     *
     * @param numReduced the number of reduced modes (default=0)
     * @param numFull the number of full modes (default=0)
     */
    public void setReducedModes(int numReduced, int numFull) {
        long bodyId = nativeId();
        setReducedModes(bodyId, numReduced, numFull);
    }

    /**
     * Enable/disable reduced modes.
     *
     * @param enable true to enabled reduced modes, false for rigid-only
     * (default=true)
     */
    public void setReducedModesEnabled(boolean enable) {
        long bodyId = nativeId();
        setReducedModesEnabled(bodyId, enable);
    }

    /**
     * Alter the stiffness scale.
     *
     * @param scale the desired scale (default=1)
     */
    public void setStiffnessScale(float scale) {
        long bodyId = nativeId();
        setStiffnessScale(bodyId, scale);
    }
    // *************************************************************************
    // PhysicsSoftBody methods

    /**
     * Create a new, empty btReducedDeformableBody for this
     * ReducedDeformableBody, using the pre-existing worldInfo. The pre-existing
     * btReducedDeformableBody (if any) will be destroyed.
     */
    @Override
    protected void newEmptySoftBody() {
        throw new UnsupportedOperationException("Not yet implemented.");
    }
    // *************************************************************************
    // native private methods

    native private static long create(
            long infoId, Vector3f[] locations, float[] masses, int numNodes);

    native private static void getLinearVelocity(
            long bodyId, Vector3f storeResult);

    native private static boolean isReducedModesEnabled(long bodyId);

    native private static void pinNode(long bodyId, int nodeIndex);

    native private static void setDamping(long bodyId, float alpha, float beta);

    native private static void setLinearVelocity(
            long bodyId, Vector3f velocity);

    native private static void setReducedModes(
            long bodyId, int numReduced, int numFull);

    native private static void setReducedModesEnabled(
            long bodyId, boolean enable);

    native private static void setStiffnessScale(long bodyId, float scale);
}
