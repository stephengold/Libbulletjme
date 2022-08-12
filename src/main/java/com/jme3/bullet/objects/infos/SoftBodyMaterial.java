/*
 * Copyright (c) 2009-2016 jMonkeyEngine
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
import com.jme3.bullet.objects.PhysicsSoftBody;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Provide access to 3 fields of the native btSoftBody::Material struct.
 */
public class SoftBodyMaterial extends NativePhysicsObject {
    // *********************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(SoftBodyMaterial.class.getName());
    // *********************************************************************
    // constructors

    /**
     * Instantiate a material with the default properties.
     *
     * @param body the body to which this material will apply (not null)
     */
    public SoftBodyMaterial(PhysicsSoftBody body) {
        long softBodyId = body.nativeId();
        long materialId = getMaterialId(softBodyId);
        super.setNativeIdNotTracked(materialId);
    }
    // *********************************************************************
    // new methods exposed

    /**
     * Read the angular-stiffness coefficient (native field: m_kAST).
     *
     * @return the coefficient (&ge;0, &le;1)
     */
    public float angularStiffness() {
        long materialId = nativeId();
        float result = getAngularStiffnessFactor(materialId);

        return result;
    }

    /**
     * Read the linear-stiffness coefficient (native field: m_kLST).
     *
     * @return the coefficient (&ge;0, &le;1)
     */
    public float linearStiffness() {
        long materialId = nativeId();
        float result = getLinearStiffnessFactor(materialId);

        return result;
    }

    /**
     * Alter the angular-stiffness coefficient (native field: m_kAST).
     *
     * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
     */
    public void setAngularStiffness(float coefficient) {
        Validate.fraction(coefficient, "stiffness coefficient");

        long materialId = nativeId();
        setAngularStiffnessFactor(materialId, coefficient);
    }

    /**
     * Alter the linear-stiffness coefficient (native field: m_kLST).
     *
     * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
     */
    public void setLinearStiffness(float coefficient) {
        Validate.fraction(coefficient, "stiffness coefficient");

        long materialId = nativeId();
        setLinearStiffnessFactor(materialId, coefficient);
    }

    /**
     * Alter the volume-stiffness coefficient (native field: m_kVST).
     *
     * @param coefficient the desired coefficient (&ge;0, &le;1, default=1)
     */
    public void setVolumeStiffness(float coefficient) {
        Validate.fraction(coefficient, "stiffness coefficient");

        long materialId = nativeId();
        setVolumeStiffnessFactor(materialId, coefficient);
    }

    /**
     * Read the volume-stiffness coefficient (native field: m_kVST).
     *
     * @return the coefficient (&ge;0, &le;1)
     */
    public float volumeStiffness() {
        long materialId = nativeId();
        float result = getVolumeStiffnessFactor(materialId);

        return result;
    }
    // *********************************************************************
    // native private methods

    native private static float getAngularStiffnessFactor(long materialId);

    native private static float getLinearStiffnessFactor(long materialId);

    native private static long getMaterialId(long bodyId);

    native private static float getVolumeStiffnessFactor(long materialId);

    native private static void setAngularStiffnessFactor(long materialId,
            float stiffness);

    native private static void setLinearStiffnessFactor(long materialId,
            float stiffness);

    native private static void setVolumeStiffnessFactor(long materialId,
            float stiffness);
}
