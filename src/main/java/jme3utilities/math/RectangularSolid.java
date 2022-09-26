/*
 Copyright (c) 2018-2022, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.math;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A rectangular solid whose axes might not be aligned with the world axes.
 * Immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RectangularSolid {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RectangularSolid.class.getName());
    // *************************************************************************
    // fields

    /**
     * orientation of the local (principal) axes (default=identity)
     */
    final private Quaternion localToWorld = new Quaternion();
    /**
     * maximum coordinate value for each local axis
     */
    final private Vector3f maxima = new Vector3f();
    /**
     * minimum coordinate value for each local axis
     */
    final private Vector3f minima = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a zero-size rectangular solid at the origin.
     */
    public RectangularSolid() {
    }

    /**
     * Instantiate a centered solid with the specified half extents.
     *
     * @param halfExtents half extents the axis-aligned bounding box (not null,
     * unaffected)
     */
    public RectangularSolid(Vector3f halfExtents) {
        maxima.set(halfExtents);
        halfExtents.mult(-1f, minima);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Determine the half extents of the solid.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the half extent of each local axis (either storeResult or a new
     * vector, not null, all components non-negative)
     */
    public Vector3f halfExtents(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        maxima.subtract(minima, result);
        result.divideLocal(2f);

        assert result.x >= 0f : result.x;
        assert result.y >= 0f : result.y;
        assert result.z >= 0f : result.z;
        return result;
    }

    /**
     * Rotate from local coordinates to world coordinates.
     *
     * @param local the input coordinates (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return the corresponding world coordinates (either storeResult or a new
     * vector, not null)
     */
    public Vector3f localToWorld(Vector3f local, Vector3f storeResult) {
        Validate.finite(local, "local");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        localToWorld.mult(local, result);
        return result;
    }

    /**
     * Copy the maximum coordinate value for each local axis.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the maxima (either storeResult or a new vector, not null)
     */
    public Vector3f maxima(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = maxima.clone();
        } else {
            result = storeResult.set(maxima);
        }
        return result;
    }

    /**
     * Copy minimum coordinate value for each local axis.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the minima (either storeResult or a new vector, not null)
     */
    public Vector3f minima(Vector3f storeResult) {
        Vector3f result;
        if (storeResult == null) {
            result = minima.clone();
        } else {
            result = storeResult.set(minima);
        }
        return result;
    }
    // *************************************************************************
    // Object methods

    /**
     * Represent this solid as a text string.
     *
     * @return descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String description = "RectangularSolid[" + localToWorld
                + ", min=" + minima + ", max=" + maxima + "]";
        return description;
    }
}
