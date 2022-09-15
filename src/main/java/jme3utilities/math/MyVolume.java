/*
 Copyright (c) 2014-2022, Stephen Gold
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

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Utility methods for computing volumes of shapes. All methods should be public
 * and static.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MyVolume {
    // *************************************************************************
    // constants and loggers

    /**
     * The value 4/3, as a float.
     */
    final private static float FOUR_THIRDS = 1.3333333f;
    /**
     * The value 4*Pi/3, as a float. (240 degrees)
     */
    final private static float FOUR_THIRDS_PI = 4.1887902f;
    /**
     * The value Pi/3, as a float. (60 degrees)
     */
    final private static float ONE_THIRD_PI = 1.0471976f;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MyVolume.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyVolume() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Compute the volume of a box with the specified half extents.
     *
     * @param halfExtents the half extents on each local axis (not null, all
     * components &ge;0, unaffected)
     * @return volume (ge;0)
     */
    public static float boxVolume(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");
        float volume = 8f * halfExtents.x * halfExtents.y * halfExtents.z;
        return volume;
    }

    /**
     * Compute the volume of a capsule with the specified radius and height.
     *
     * @param radius the radius of the capsule (&ge;0)
     * @param height the height of the cylindrical portion (&ge;0)
     * @return the volume (&ge;0)
     */
    public static float capsuleVolume(float radius, float height) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        float crossSection = FastMath.PI * radius * radius;
        float volume = crossSection * (height + FOUR_THIRDS * radius);

        assert volume >= 0f : volume;
        return volume;
    }

    /**
     * Compute the volume of a cone with the specified radius and height.
     *
     * @param radius the radius of the cone (&ge;0)
     * @param height the height of the cone (&ge;0)
     * @return the volume (&ge;0)
     */
    public static float coneVolume(float radius, float height) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        float volume = ONE_THIRD_PI * radius * radius * height;

        assert volume >= 0f : volume;
        return volume;
    }

    /**
     * Compute the volume of a cylinder with specified half extents.
     *
     * @param halfExtents the half extents on each local axis (not null, all
     * components &ge;0, unaffected)
     * @return the volume (&ge;0)
     */
    public static float cylinderVolume(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");
        float volume = FastMath.TWO_PI
                * halfExtents.x * halfExtents.y * halfExtents.z;

        assert volume >= 0f : volume;
        return volume;
    }

    /**
     * Compute the volume of a sphere with the specified radius.
     *
     * @param radius the radius of the sphere (&ge;0)
     * @return the volume (&ge;0)
     */
    public static float sphereVolume(float radius) {
        Validate.nonNegative(radius, "radius");
        float volume = FOUR_THIRDS_PI * MyMath.cube(radius);

        assert volume >= 0f : volume;
        return volume;
    }

    /**
     * Determine the volume of the specified tetrahedron.
     *
     * @param v1 the location of the first vertex (not null, unaffected)
     * @param v2 the location of the 2nd vertex (not null, unaffected)
     * @param v3 the location of the 3rd vertex (not null, unaffected)
     * @param v4 the location of the 4th vertex (not null, unaffected)
     * @return the volume (&ge;0)
     */
    public static double tetrahedronVolume(Vector3f v1, Vector3f v2,
            Vector3f v3, Vector3f v4) {
        Validate.finite(v1, "first vertex");
        Validate.finite(v2, "2nd vertex");
        Validate.finite(v3, "3rd vertex");
        Validate.finite(v4, "4th vertex");

        // Set up a 3x3 matrix of offset components relative to v4.
        double m00 = v1.x - v4.x;
        double m01 = v1.y - v4.y;
        double m02 = v1.z - v4.z;
        double m10 = v2.x - v4.x;
        double m11 = v2.y - v4.y;
        double m12 = v2.z - v4.z;
        double m20 = v3.x - v4.x;
        double m21 = v3.y - v4.y;
        double m22 = v3.z - v4.z;

        // Compute the determinant.
        double co00 = m11 * m22 - m12 * m21;
        double co10 = m12 * m20 - m10 * m22;
        double co20 = m10 * m21 - m11 * m20;
        double determinant = m00 * co00 + m01 * co10 + m02 * co20;

        // The volume is 1/6 the absolute value of the determinant.
        double result = Math.abs(determinant) / 6.0;

        return result;
    }
}
