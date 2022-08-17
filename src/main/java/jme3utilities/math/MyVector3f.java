/*
 Copyright (c) 2013-2022, Stephen Gold
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

import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Utility methods for 3-D vectors.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MyVector3f { // TODO finalize the class
    // *************************************************************************
    // constants and loggers

    /**
     * index of the first (X) axis
     */
    final public static int firstAxis = 0;
    /**
     * number of axes in the coordinate system
     */
    final public static int numAxes = 3;
    /**
     * index of the X axis
     */
    final public static int xAxis = 0;
    /**
     * index of the Y axis
     */
    final public static int yAxis = 1;
    /**
     * index of the Z axis
     */
    final public static int zAxis = 2;
    /**
     * index of the final (Z) axis
     */
    final public static int lastAxis = numAxes - 1;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MyVector3f.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyVector3f() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Accumulate maximum coordinates.
     *
     * @param maxima the highest coordinate so far for each axis (not null,
     * modified)
     * @param input vector to compare (not null, unaffected)
     */
    public static void accumulateMaxima(Vector3f maxima, Vector3f input) {
        if (input.x > maxima.x) {
            maxima.x = input.x;
        }
        if (input.y > maxima.y) {
            maxima.y = input.y;
        }
        if (input.z > maxima.z) {
            maxima.z = input.z;
        }
    }

    /**
     * Accumulate minimum coordinates.
     *
     * @param minima the lowest coordinate so far for each axis (not null,
     * modified)
     * @param input vector to compare (not null, unaffected)
     */
    public static void accumulateMinima(Vector3f minima, Vector3f input) {
        if (input.x < minima.x) {
            minima.x = input.x;
        }
        if (input.y < minima.y) {
            minima.y = input.y;
        }
        if (input.z < minima.z) {
            minima.z = input.z;
        }
    }

    /**
     * Accumulate a linear combination of vectors.
     *
     * @param total sum of the scaled inputs so far (not null, modified)
     * @param input the vector to scale and add (not null, unaffected)
     * @param scale scale factor to apply to the input
     */
    public static void accumulateScaled(Vector3f total, Vector3f input,
            float scale) {
        assert Validate.nonNull(total, "total");
        assert Validate.nonNull(input, "input");

        total.x += input.x * scale;
        total.y += input.y * scale;
        total.z += input.z * scale;
    }

    /**
     * Determine the dot (scalar) product of 2 vectors. Unlike
     * {@link com.jme3.math.Vector3f#dot(Vector3f)}, this method returns a
     * double-precision value for precise calculation of angles.
     *
     * @param vector1 the first input vector (not null, unaffected)
     * @param vector2 the 2nd input vector (not null, unaffected)
     * @return the dot product
     */
    public static double dot(Vector3f vector1, Vector3f vector2) {
        double x1 = vector1.x;
        double x2 = vector2.x;
        double y1 = vector1.y;
        double y2 = vector2.y;
        double z1 = vector1.z;
        double z2 = vector2.z;
        double product = x1 * x2 + y1 * y2 + z1 * z2;

        return product;
    }

    /**
     * Generate an orthonormal basis that includes the specified vector.
     *
     * @param in1 input direction for the first basis vector (not null, not
     * zero, modified)
     * @param store2 storage for the 2nd basis vector (not null, modified)
     * @param store3 storage for the 3rd basis vector (not null, modified)
     */
    public static void generateBasis(Vector3f in1, Vector3f store2,
            Vector3f store3) {
        assert Validate.nonZero(in1, "starting direction");
        assert Validate.nonNull(store2, "2nd basis vector");
        assert Validate.nonNull(store3, "3nd basis vector");

        normalizeLocal(in1);
        /*
         * Pick a direction that's not parallel (or anti-parallel) to
         * the input direction.
         */
        float x = Math.abs(in1.x);
        float y = Math.abs(in1.y);
        float z = Math.abs(in1.z);
        if (x <= y && x <= z) {
            store3.set(1f, 0f, 0f);
        } else if (y <= z) {
            store3.set(0f, 1f, 0f);
        } else {
            store3.set(0f, 0f, 1f);
        }
        /*
         * Use cross products to generate unit vectors orthogonal
         * to the input vector.
         */
        in1.cross(store3, store2);
        normalizeLocal(store2);
        in1.cross(store2, store3);
        normalizeLocal(store3);
    }

    /**
     * Test whether all components of a vector are all non-negative: in other
     * words, whether it's in the first octant or the boundaries thereof.
     *
     * @param vector input (not null, unaffected)
     * @return true if all components are non-negative, false otherwise
     */
    public static boolean isAllNonNegative(Vector3f vector) {
        if (vector.x >= 0f && vector.y >= 0f && vector.z >= 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test whether all components of a vector are all positive: in other words,
     * whether it's strictly inside the first octant.
     *
     * @param vector input (not null, unaffected)
     * @return true if all components are positive, false otherwise
     */
    public static boolean isAllPositive(Vector3f vector) {
        if (vector.x > 0f && vector.y > 0f && vector.z > 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test for a scale identity.
     *
     * @param vector input (not null, unaffected)
     * @return true if the vector equals
     * {@link com.jme3.math.Vector3f#UNIT_XYZ}, false otherwise
     */
    public static boolean isScaleIdentity(Vector3f vector) {
        if (vector.x == 1f && vector.y == 1f && vector.z == 1f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test for a uniform scaling vector.
     *
     * @param vector input (not null, unaffected)
     * @return true if all 3 components are equal, false otherwise
     */
    public static boolean isScaleUniform(Vector3f vector) {
        if (vector.x == vector.y && vector.y == vector.z) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test for a zero vector or translation identity.
     *
     * @param vector input (not null, unaffected)
     * @return true if the vector equals (0,0,0), false otherwise
     */
    public static boolean isZero(Vector3f vector) {
        if (vector.x == 0f && vector.y == 0f && vector.z == 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Determine the squared length of a vector. Unlike
     * {@link com.jme3.math.Vector3f#lengthSquared()}, this method returns a
     * double-precision value for precise comparison of lengths.
     *
     * @param vector input (not null, unaffected)
     * @return the squared length (&ge;0)
     */
    public static double lengthSquared(Vector3f vector) {
        double result = MyMath.sumOfSquares(vector.x, vector.y, vector.z);
        return result;
    }

    /**
     * Interpolate between (or extrapolate from) 2 vectors using linear (Lerp)
     * *polation. No rounding error is introduced when v1==v2.
     *
     * @param t descaled parameter value (0&rarr;v0, 1&rarr;v1)
     * @param v0 function value at t=0 (not null, unaffected unless it's also
     * storeResult)
     * @param v1 function value at t=1 (not null, unaffected unless it's also
     * storeResult)
     * @param storeResult storage for the result (modified if not null, may be
     * v0 or v1)
     * @return an interpolated vector (either storeResult or a new instance)
     */
    public static Vector3f lerp(float t, Vector3f v0, Vector3f v1,
            Vector3f storeResult) {
        assert Validate.nonNull(v0, "v0");
        assert Validate.nonNull(v1, "v1");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.x = MyMath.lerp(t, v0.x, v1.x);
        result.y = MyMath.lerp(t, v0.y, v1.y);
        result.z = MyMath.lerp(t, v0.z, v1.z);

        return result;
    }

    /**
     * Determine the midpoint between 2 locations.
     *
     * @param vector1 coordinates of the first location (not null, unaffected
     * unless it's storeResult)
     * @param vector2 coordinates of the 2nd location (not null, unaffected
     * unless it's storeResult)
     * @param storeResult storage for the result (modified if not null, may be
     * vector1 or vector2)
     * @return a coordinate vector (either storeResult or a new instance)
     */
    public static Vector3f midpoint(Vector3f vector1, Vector3f vector2,
            Vector3f storeResult) {
        assert Validate.finite(vector1, "first location");
        assert Validate.finite(vector2, "2nd location");

        float x = (vector1.x + vector2.x) / 2f;
        float y = (vector1.y + vector2.y) / 2f;
        float z = (vector1.z + vector2.z) / 2f;

        if (storeResult == null) {
            return new Vector3f(x, y, z);
        } else {
            return storeResult.set(x, y, z);
        }
    }

    /**
     * Test whether 2 vectors are distinct, without distinguishing 0 from -0.
     *
     * @param v1 the first input vector (not null, unaffected)
     * @param v2 the 2nd input vector (not null, unaffected)
     * @return true if distinct, otherwise false
     */
    public static boolean ne(Vector3f v1, Vector3f v2) {
        assert Validate.nonNull(v1, "first input vector");
        assert Validate.nonNull(v2, "2nd input vector");

        boolean result = v1.x != v2.x || v1.y != v2.y || v1.z != v2.z;
        return result;
    }

    /**
     * Normalize the specified vector in place.
     *
     * @param input (not null, modified)
     */
    public static void normalizeLocal(Vector3f input) {
        assert Validate.nonNull(input, "input");

        double lengthSquared = lengthSquared(input);
        double dScale = Math.sqrt(lengthSquared);
        float fScale = (float) dScale;
        if (fScale != 0f && fScale != 1f) {
            input.divideLocal(fScale);
        }
    }

    /**
     * Standardize a vector in preparation for hashing.
     *
     * @param input (not null, unaffected unless it's also storeResult)
     * @param storeResult storage for the result (modified if not null, may be
     * input)
     * @return an equivalent vector without any negative zero components (either
     * storeResult or a new instance)
     */
    public static Vector3f standardize(Vector3f input, Vector3f storeResult) {
        assert Validate.nonNull(input, "input vector");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.x = MyMath.standardize(input.x);
        result.y = MyMath.standardize(input.y);
        result.z = MyMath.standardize(input.z);

        return result;
    }
}
