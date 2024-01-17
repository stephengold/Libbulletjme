/*
 Copyright (c) 2013-2024, Stephen Gold
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
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.util.TempVars;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Mathematical utility methods.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MyMath {
    // *************************************************************************
    // constants and loggers

    /**
     * golden ratio = 1.618...
     */
    final public static float phi = (1f + FastMath.sqrt(5f)) / 2f;
    /**
     * square root of 2
     */
    final public static float root2 = FastMath.sqrt(2f);
    /**
     * square root of 1/2
     */
    final public static float rootHalf = FastMath.sqrt(0.5f);
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MyMath.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyMath() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Clamp a double-precision value between 2 limits.
     *
     * @param dValue input value to be clamped
     * @param min lower limit of the clamp
     * @param max upper limit of the clamp
     * @return the value between min and max inclusive that is closest to fValue
     */
    public static double clamp(double dValue, double min, double max) {
        double result;
        if (dValue < min) {
            result = min;
        } else if (dValue > max) {
            result = max;
        } else {
            result = dValue;
        }

        return result;
    }

    /**
     * Clamp an integer value between 2 limits.
     *
     * @param iValue input value to be clamped
     * @param min the lower limit
     * @param max the upper limit
     * @return the value between min and max inclusive that is closest to iValue
     */
    public static int clamp(int iValue, int min, int max) {
        int result;
        if (iValue < min) {
            result = min;
        } else if (iValue > max) {
            result = max;
        } else {
            result = iValue;
        }

        return result;
    }

    /**
     * Combine the specified transforms.
     * <p>
     * It is safe for any or all of {@code child}, {@code parent}, and
     * {@code storeResult} to be the same object.
     * <p>
     * This method works on transforms containing non-normalized quaternions.
     *
     * @param child the transform applied first (not null, unaffected unless
     * it's {@code storeResult})
     * @param parent the transform applied last (not null, unaffected unless
     * it's {@code storeResult})
     * @param storeResult (modified if not null)
     * @return a Transform equivalent to {@code child} followed by
     * {@code parent} (either {@code storeResult} or a new instance)
     */
    public static Transform combine(
            Transform child, Transform parent, Transform storeResult) {
        TempVars tempVars = TempVars.get();
        Vector3f combTranslation = tempVars.vect1; // alias
        Quaternion combRotation = tempVars.quat1; // alias
        Vector3f combScale = tempVars.vect2; // alias

        Vector3f parentTranslation = parent.getTranslation(); // alias
        Quaternion parentRotation = parent.getRotation(); // alias
        Vector3f parentScale = parent.getScale(); // alias

        // Combine the scales.
        child.getScale().mult(parentScale, combScale);

        // Combine the (intrinsic) rotations and re-normalize.
        Quaternion childRotation = child.getRotation(); // alias
        parentRotation.mult(childRotation, combRotation);
        MyQuaternion.normalizeLocal(combRotation);
        /*
         * The combined translation is the parent's scale, rotation,
         * and translation applied (in that order) to the child's translation.
         */
        child.getTranslation().mult(parentScale, combTranslation);
        MyQuaternion.rotate(parentRotation, combTranslation, combTranslation);
        combTranslation.addLocal(parentTranslation);

        Transform result
                = (storeResult == null) ? new Transform() : storeResult;
        result.setTranslation(combTranslation);
        result.setRotation(combRotation);
        result.setScale(combScale);
        tempVars.release();

        return result;
    }

    /**
     * Cube the specified single-precision value. Logs a warning in case of
     * overflow.
     *
     * @param fValue input value to be cubed
     * @return fValue raised to the third power
     */
    public static float cube(float fValue) {
        float result = fValue * fValue * fValue;

        if (Float.isInfinite(result)) {
            String message = String.format("Overflow from cubing %g.", fValue);
            logger.warning(message);
        }
        return result;
    }

    /**
     * Sets a rotation matrix from the specified Tait-Bryan angles, applying the
     * rotations in x-z-y extrinsic order or y-z'-x" intrinsic order.
     *
     * @param xAngle the X angle (in radians)
     * @param yAngle the Y angle (in radians)
     * @param zAngle the Z angle (in radians)
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (either {@code storeResult} or a new instance)
     */
    public static Matrix3f fromAngles(
            float xAngle, float yAngle, float zAngle, Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        float c1 = FastMath.cos(yAngle);
        float c2 = FastMath.cos(zAngle);
        float c3 = FastMath.cos(xAngle);
        float s1 = FastMath.sin(yAngle);
        float s2 = FastMath.sin(zAngle);
        float s3 = FastMath.sin(xAngle);

        result.set(0, 0, c1 * c2);
        result.set(0, 1, s1 * s3 - c1 * c3 * s2);
        result.set(0, 2, c3 * s1 + c1 * s2 * s3);

        result.set(1, 0, s2);
        result.set(1, 1, c2 * c3);
        result.set(1, 2, -c2 * s3);

        result.set(2, 0, -c2 * s1);
        result.set(2, 1, c1 * s3 + c3 * s1 * s2);
        result.set(2, 2, c1 * c3 - s1 * s2 * s3);

        return result;
    }

    /**
     * Return the root sum of squares of some single-precision values.
     * Double-precision arithmetic is used to reduce the risk of overflow.
     *
     * @param fValues the input values
     * @return the positive square root of the sum of squares (&ge;0)
     */
    public static float hypotenuse(float... fValues) {
        double sum = 0.0;
        for (float fValue : fValues) {
            double value = fValue;
            sum += value * value;
        }

        float result = (float) Math.sqrt(sum);
        assert result >= 0f : result;
        return result;
    }

    /**
     * Return the root sum of squares of some double-precision values.
     *
     * @param dValues the input values
     * @return the positive square root of the sum of squares (&ge;0)
     */
    public static double hypotenuseDouble(double... dValues) {
        double sum = 0.0;
        for (double value : dValues) {
            sum += value * value;
        }

        double result = Math.sqrt(sum);
        assert result >= 0.0 : result;
        return result;
    }

    /**
     * Test whether b is between a and c.
     *
     * @param a the first input value
     * @param b the 2nd input value
     * @param c the 3rd input value
     * @return true if b is between a and c (inclusive), otherwise false
     */
    public static boolean isBetween(float a, float b, float c) {
        if (a > c) {
            return a >= b && b >= c;
        } else if (a < c) {
            return a <= b && b <= c;
        } else if (a == c) {
            return a == b;
        } else {
            String message = "a = " + a + " c = " + c;
            throw new IllegalArgumentException(message);
        }
    }

    /**
     * Test whether the specified floating-point value is finite. Note that Java
     * 8 provides {@link java.lang.Float#isFinite(float)}.
     *
     * @param value the value to test
     * @return true if finite, false if NaN or infinity
     */
    public static boolean isFinite(float value) {
        if (Float.isInfinite(value)) {
            return false;
        } else if (Float.isNaN(value)) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Test whether the specified floating-point value is finite. Note that Java
     * 8 provides {@link java.lang.Double#isFinite(double)}.
     *
     * @param value the value to test
     * @return true if finite, false if NaN or infinity
     */
    public static boolean isFiniteDouble(double value) {
        if (Double.isInfinite(value)) {
            return false;
        } else if (Double.isNaN(value)) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Test the specified transform for exact identity.
     *
     * @param transform which transform to test (not null, unaffected)
     * @return true if exact identity, otherwise false
     */
    public static boolean isIdentity(Transform transform) {
        boolean result = false;
        Vector3f translation = transform.getTranslation(); // alias
        if (MyVector3f.isZero(translation)) {
            Quaternion rotation = transform.getRotation(); // alias
            if (MyQuaternion.isRotationIdentity(rotation)) {
                Vector3f scale = transform.getScale(); // alias
                result = MyVector3f.isScaleIdentity(scale);
            }
        }

        return result;
    }

    /**
     * Test whether an integer value is odd.
     *
     * @param iValue the value to be tested
     * @return true if x is odd, false if it's even
     */
    public static boolean isOdd(int iValue) {
        boolean result = (iValue % 2) != 0;
        return result;
    }

    /**
     * Interpolate linearly between (or extrapolate linearly from) 2
     * single-precision values.
     * <p>
     * No rounding error is introduced when y0==y1.
     *
     * @param t the weight given to {@code y1}
     * @param y0 the function value at t=0
     * @param y1 the function value at t=1
     * @return the interpolated function value
     */
    public static float lerp(float t, float y0, float y1) {
        float result;
        if (y0 == y1) {
            result = y0;
        } else {
            float u = 1f - t;
            result = u * y0 + t * y1;
        }

        return result;
    }

    /**
     * Calculate the floor of the base-2 logarithm of the input value.
     *
     * @param iValue the input value (&ge;1)
     * @return the largest integer N&le;30 for which {@code (1 << N) <= iValue}
     * (&ge;0, &le;30)
     */
    public static int log2(int iValue) {
        Validate.positive(iValue, "input value");
        int result = 31 - Integer.numberOfLeadingZeros(iValue);
        return result;
    }

    /**
     * Find the maximum of some single-precision values.
     *
     * @param fValues the input values
     * @return the most positive value
     * @see java.lang.Math#max(float, float)
     */
    public static float max(float... fValues) {
        float result = Float.NEGATIVE_INFINITY;
        for (float value : fValues) {
            if (value > result) {
                result = value;
            }
        }

        return result;
    }

    /**
     * Find the maximum of some int values.
     *
     * @param iValues the input values
     * @return the most positive value
     * @see java.util.Collections#max(java.util.Collection)
     * @see java.lang.Math#max(int, int)
     */
    public static int maxInt(int... iValues) {
        int result = Integer.MIN_VALUE;
        for (int value : iValues) {
            if (value > result) {
                result = value;
            }
        }

        return result;
    }

    /**
     * Find the median of 3 single-precision values.
     *
     * @param a the first input value
     * @param b the 2nd input value
     * @param c the 3rd input value
     * @return the median of the 3 values
     */
    public static float mid(float a, float b, float c) {
        if (a >= b) {
            if (b >= c) {
                return b; // a >= b >= c
            } else if (a >= c) {
                return c; // a >= c > b
            } else {
                return a; // c > a >= b
            }
        } else if (a >= c) {
            return a; // b > a >= c
        } else if (b >= c) {
            return c; // b >= c > a
        } else {
            return b; // c > b > a
        }
    }

    /**
     * Find the minimum of some single-precision values.
     *
     * @param fValues the input values
     * @return the most negative value
     * @see java.lang.Math#min(float, float)
     */
    public static float min(float... fValues) {
        float result = Float.POSITIVE_INFINITY;
        for (float value : fValues) {
            if (value < result) {
                result = value;
            }
        }

        return result;
    }

    /**
     * Return the least non-negative value congruent with the input value with
     * respect to the specified modulus.
     * <p>
     * This differs from remainder for negative input values. For instance,
     * modulo(-1, 4) == 3, while -1 % 4 == -1.
     *
     * @param iValue the input value
     * @param modulus (&gt;0)
     * @return iValue MOD modulus (&lt;modulus, &ge;0)
     */
    public static int modulo(int iValue, int modulus) {
        assert Validate.positive(modulus, "modulus");

        int remainder = iValue % modulus;
        int result;
        if (iValue >= 0) {
            result = remainder;
        } else {
            result = (remainder + modulus) % modulus;
        }

        assert result >= 0f : result;
        assert result < modulus : result;
        return result;
    }

    /**
     * Return the least non-negative value congruent with the input value with
     * respect to the specified modulus.
     * <p>
     * This differs from remainder for negative input values. For instance,
     * modulo(-1f, 4f) == 3f, while -1f % 4f == -1f.
     *
     * @param fValue the input value
     * @param modulus (&gt;0)
     * @return fValue MOD modulus (&lt;modulus, &ge;0)
     */
    public static float modulo(float fValue, float modulus) {
        assert Validate.positive(modulus, "modulus");

        float remainder = fValue % modulus;
        float result;
        if (fValue >= 0) {
            result = remainder;
        } else {
            result = (remainder + modulus) % modulus;
        }

        assert result >= 0f : result;
        assert result < modulus : result;
        return result;
    }

    /**
     * Standardize a single-precision value in preparation for hashing.
     *
     * @param fValue input value
     * @return an equivalent value that's not -0
     */
    public static float standardize(float fValue) {
        float result = fValue;
        if (Float.compare(fValue, -0f) == 0) {
            result = 0f;
        }

        return result;
    }

    /**
     * Standardize a rotation angle to the range [-Pi, Pi).
     *
     * @param angle the input angle (in radians)
     * @return the standardized angle (in radians, &lt;Pi, &ge;-Pi)
     */
    public static float standardizeAngle(float angle) {
        Validate.finite(angle, "angle");

        float result = modulo(angle, FastMath.TWO_PI);
        if (result >= FastMath.PI) {
            result -= FastMath.TWO_PI;
        }

        assert result >= -FastMath.PI : result;
        assert result < FastMath.PI : result;
        return result;
    }

    /**
     * Return the sum of squares of some single-precision values.
     * Double-precision arithmetic is used to reduce the risk of overflow.
     *
     * @param fValues the input values
     * @return the sum of squares (&ge;0)
     */
    public static double sumOfSquares(float... fValues) {
        double result = 0.0;
        for (float fValue : fValues) {
            double value = fValue;
            result += value * value;
        }

        assert result >= 0.0 : result;
        return result;
    }

    /**
     * Convert an angle from radians to degrees.
     *
     * @param radians input angle
     * @return equivalent in degrees
     * @see java.lang.Math#toDegrees(double)
     */
    public static float toDegrees(float radians) {
        float result = radians * FastMath.RAD_TO_DEG;
        return result;
    }

    /**
     * Convert an angle from degrees to radians.
     *
     * @param degrees input angle
     * @return equivalent in radians
     * @see java.lang.Math#toRadians(double)
     */
    public static float toRadians(float degrees) {
        float result = degrees * FastMath.DEG_TO_RAD;
        return result;
    }

    /**
     * Apply the specified transform to a Vector3f.
     * <p>
     * It is safe for {@code input} and {@code storeResult} to be the same
     * object.
     * <p>
     * This method works on transforms containing non-normalized quaternions.
     *
     * @param transform the transform to apply (not null, unaffected unless
     * {@code storeResult} is its translation or scaling component)
     * @param input the input vector (not null, unaffected unless it's
     * {@code storeResult})
     * @param storeResult storage for the result (modified if not null)
     * @return the transformed vector (either {@code storeResult} or a new
     * instance)
     */
    public static Vector3f transform(
            Transform transform, Vector3f input, Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        Vector3f translation = transform.getTranslation(); // alias
        if (translation == result) {
            translation = translation.clone();
        }

        // scale
        Vector3f scale = transform.getScale(); // alias
        input.mult(scale, result);

        // rotate
        Quaternion rotation = transform.getRotation(); // alias
        MyQuaternion.rotate(rotation, result, result);

        // translate
        result.addLocal(translation);

        return result;
    }

    /**
     * Apply the inverse of the specified transform to each vertex of a
     * Triangle.
     * <p>
     * It is safe for {@code input} and {@code storeResult} to be the same
     * object.
     *
     * @param transform the transform to apply (not null, unaffected)
     * @param input the input triangle (not null, unaffected unless it's
     * {@code storeResult})
     * @param storeResult storage for the result (modified if not null)
     * @return the transformed triangle (either {@code storeResult} or a new
     * instance)
     */
    public static Triangle transformInverse(
            Transform transform, Triangle input, Triangle storeResult) {
        Triangle result = (storeResult == null) ? new Triangle() : storeResult;

        Vector3f tmpVector = new Vector3f();
        for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            Vector3f inputVector = input.get(vertexIndex); // alias
            MyMath.transformInverse(transform, inputVector, tmpVector);
            result.set(vertexIndex, tmpVector);
        }

        return result;
    }

    /**
     * Apply the inverse of the specified transform to a Vector3f.
     * <p>
     * It is safe for {@code input} and {@code storeResult} to be the same
     * object.
     * <p>
     * This method works on transforms containing non-normalized quaternions.
     *
     * @param transform the transform to un-apply (not null, unaffected unless
     * {@code storeResult} is its translation or scaling component)
     * @param input the input vector (not null, unaffected unless it's
     * {@code storeResult})
     * @param storeResult storage for the result (modified if not null)
     * @return the transformed vector (either {@code storeResult} or a new
     * instance)
     */
    public static Vector3f transformInverse(
            Transform transform, Vector3f input, Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        Vector3f scale = transform.getScale(); // alias
        if (scale == result) {
            scale = scale.clone();
        }

        // un-translate
        Vector3f translation = transform.getTranslation(); // alias
        input.subtract(translation, result);

        // un-rotate
        Quaternion rotation = transform.getRotation(); // alias
        MyQuaternion.rotateInverse(rotation, result, result);

        // de-scale
        result.divideLocal(scale);

        return result;
    }
}
