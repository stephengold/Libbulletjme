/*
 Copyright (c) 2017-2024, Stephen Gold
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
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;

/**
 * Mathematical utility methods.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MyQuaternion {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(MyQuaternion.class.getName());
    /**
     * array of cardinal axes
     */
    final private static Vector3f[] cardinalAxes = {
        new Vector3f(1f, 0f, 0f),
        new Vector3f(0f, 1f, 0f),
        new Vector3f(0f, 0f, 1f),
        new Vector3f(-1f, 0f, 0f),
        new Vector3f(0f, -1f, 0f),
        new Vector3f(0f, 0f, -1f)
    };
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyQuaternion() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Accumulate a linear combination of quaternions.
     *
     * @param total sum of the scaled inputs so far (not null, modified, may be
     * {@code input})
     * @param input the Quaternion to scale and add (not null, unaffected unless
     * it's {@code total})
     * @param scale scale factor to apply to the input
     */
    public static void accumulateScaled(
            Quaternion total, Quaternion input, float scale) {
        float tx = total.getX();
        float ix = input.getX();
        float x = tx + ix * scale;
        float y = total.getY() + input.getY() * scale;
        float z = total.getZ() + input.getZ() * scale;
        float w = total.getW() + input.getW() * scale;
        total.set(x, y, z, w);
    }

    /**
     * Find the cardinal rotation most similar to the specified Quaternion. A
     * cardinal rotation is one for which the rotation angles on all 3 axes are
     * integer multiples of Pi/2 radians.
     *
     * @param input the input value (not null, modified)
     */
    public static void cardinalizeLocal(Quaternion input) {
        assert Validate.nonNull(input, "input");

        normalizeLocal(input);

        // Generate each of the 24 cardinal rotations.
        Quaternion cardinalRotation = new Quaternion();
        Quaternion bestCardinalRotation = new Quaternion();
        Vector3f z = new Vector3f();
        double bestAbsDot = -1.0;
        for (Vector3f x : cardinalAxes) {
            for (Vector3f y : cardinalAxes) {
                x.cross(y, z);
                if (z.isUnitVector()) {
                    cardinalRotation.fromAxes(x, y, z);
                    /*
                     * Measure the similarity of the 2 rotations
                     * using the absolute value of their dot product.
                     */
                    double dot = dot(cardinalRotation, input);
                    double absDot = Math.abs(dot);
                    if (absDot > bestAbsDot) {
                        bestAbsDot = absDot;
                        bestCardinalRotation.set(cardinalRotation);
                    }
                }
            }
        }

        input.set(bestCardinalRotation);
    }

    /**
     * Return the conjugate of a Quaternion.
     * <p>
     * For normalized quaternions, the conjugate is a fast way to calculate the
     * inverse.
     *
     * @param q the input value (not null, unaffected unless it's
     * {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code q})
     * @return a conjugate quaternion (either {@code storeResult} or a new
     * instance)
     */
    public static Quaternion conjugate(Quaternion q, Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        float qx = q.getX();
        float qy = q.getY();
        float qz = q.getZ();
        float qw = q.getW();
        result.set(-qx, -qy, -qz, qw);

        return result;
    }

    /**
     * Generate a textual description of a Quaternion value.
     *
     * @param q the value to describe (may be null, unaffected)
     * @return a description (not null, not empty)
     */
    public static String describe(Quaternion q) {
        String result;
        if (q == null) {
            result = "null";
        } else {
            StringBuilder builder = new StringBuilder(40);
            builder.append("x=");
            String x = MyString.describeFraction(q.getX());
            builder.append(x);

            builder.append(" y=");
            String y = MyString.describeFraction(q.getY());
            builder.append(y);

            builder.append(" z=");
            String z = MyString.describeFraction(q.getZ());
            builder.append(z);

            builder.append(" w=");
            String w = MyString.describeFraction(q.getW());
            builder.append(w);

            result = builder.toString();
        }

        assert result != null;
        assert !result.isEmpty();
        return result;
    }

    /**
     * Return the dot (scalar) product of 2 quaternions.
     * <p>
     * This method returns a double-precision value for precise comparison of
     * angles.
     *
     * @param q1 the first input value (not null, unaffected)
     * @param q2 the 2nd input value (not null, unaffected)
     * @return the dot product
     */
    public static double dot(Quaternion q1, Quaternion q2) {
        double w1 = q1.getW();
        double w2 = q2.getW();
        double x1 = q1.getX();
        double x2 = q2.getX();
        double y1 = q1.getY();
        double y2 = q2.getY();
        double z1 = q1.getZ();
        double z2 = q2.getZ();
        double result = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2;

        return result;
    }

    /**
     * Return the exponential of a pure Quaternion.
     *
     * @param q the input value (not null, w=0, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return the exponential value (either {@code storeResult} or a new
     * instance)
     */
    public static Quaternion exp(Quaternion q, Quaternion storeResult) {
        assert Validate.require(isPure(q), "a pure quaternion");
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        double qx = q.getX();
        double qy = q.getY();
        double qz = q.getZ();
        double theta = MyMath.hypotenuseDouble(qx, qy, qz);
        if (theta == 0.0) {
            result.loadIdentity();
        } else {
            float w = (float) Math.cos(theta);
            double scale = Math.sin(theta) / theta;
            float x = (float) (scale * qx);
            float y = (float) (scale * qy);
            float z = (float) (scale * qz);
            result.set(x, y, z, w);
        }

        return result;
    }

    /**
     * Test for a pure Quaternion.
     *
     * @param q the value to test (not null, unaffected)
     * @return true if w=0, false otherwise
     */
    public static boolean isPure(Quaternion q) {
        float qw = q.getW();

        if (qw == 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test whether the specified Quaternion represents an identity rotation.
     * <p>
     * Accepts any finite, non-zero value for w.
     *
     * @param q the value to test (not null, unaffected)
     * @return true for a rotation identity, otherwise false
     */
    public static boolean isRotationIdentity(Quaternion q) {
        float qw = q.getW();

        if (qw != 0f && !Float.isNaN(qw)
                && q.getX() == 0f && q.getY() == 0f && q.getZ() == 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test for a zero Quaternion.
     *
     * @param q the value to test (not null, unaffected)
     * @return true if the argument equals (0,0,0,0), false otherwise
     */
    public static boolean isZero(Quaternion q) {
        float qw = q.getW();

        if (qw == 0f && q.getX() == 0f && q.getY() == 0f && q.getZ() == 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Return the squared length of the argument.
     * <p>
     * Unlike {@link com.jme3.math.Quaternion#norm()}, this method returns a
     * double-precision value for precise comparison of lengths.
     *
     * @param q the input value (not null, unaffected)
     * @return the squared length (&ge;0)
     */
    public static double lengthSquared(Quaternion q) {
        double xx = q.getX();
        double yy = q.getY();
        double zz = q.getZ();
        double ww = q.getW();
        double result = xx * xx + yy * yy + zz * zz + ww * ww;

        return result;
    }

    /**
     * Return the natural logarithm of a normalized quaternion.
     * <p>
     * In general, the result isn't itself normalized.
     *
     * @param q the input value (not null, norm approximately equal to 1,
     * unaffected unless it's {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code q})
     * @return a pure quaternion (either {@code storeResult} or a new instance)
     */
    public static Quaternion log(Quaternion q, Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        float qw = q.getW();
        if (qw >= 1f || qw <= -1f) {
            result.set(0f, 0f, 0f, 0f);
        } else {
            double qx = q.getX();
            double qy = q.getY();
            double qz = q.getZ();
            double sineTheta = MyMath.hypotenuseDouble(qx, qy, qz);
            sineTheta = MyMath.clamp(sineTheta, 0.0, 1.0);
            if (sineTheta == 0.0) {
                result.set(0f, 0f, 0f, 0f);
            } else {
                double theta = Math.asin(sineTheta);
                double scale = theta / sineTheta;
                float x = (float) (scale * qx);
                float y = (float) (scale * qy);
                float z = (float) (scale * qz);
                result.set(x, y, z, 0f);
            }
        }

        return result;
    }

    /**
     * Test whether 2 quaternions are distinct, without distinguishing 0 from
     * -0.
     *
     * @param a the first input value (not null, unaffected)
     * @param b the 2nd input value (not null, unaffected)
     * @return true if distinct, otherwise false
     */
    public static boolean ne(Quaternion a, Quaternion b) {
        float aw = a.getW();
        float bw = b.getW();
        boolean result = aw != bw
                || a.getX() != b.getX()
                || a.getY() != b.getY()
                || a.getZ() != b.getZ();
        return result;
    }

    /**
     * Normalize the specified Quaternion in place.
     *
     * @param input the instance to normalize (not null, modified)
     */
    public static void normalizeLocal(Quaternion input) {
        assert Validate.nonNull(input, "input");

        double lengthSquared = lengthSquared(input);
        if (lengthSquared < 0.9999998 || lengthSquared > 1.0000002) {
            float fScale = (float) Math.sqrt(lengthSquared);
            if (fScale != 0f) {
                input.multLocal(1f / fScale);
            }
        }
    }

    /**
     * Raise a normalized quaternion to the specified real power.
     *
     * @param base the input value (not null, norm approximately equal to 1,
     * unaffected unless it's {@code storeResult})
     * @param exponent the exponent
     * @param storeResult storage for the result (modified if not null, may be
     * {@code base})
     * @return the power (either {@code storeResult} or a new instance)
     */
    public static Quaternion pow(
            Quaternion base, float exponent, Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        float baseW = base.getW();
        if (baseW >= 1f || baseW <= -1f || exponent == 0f) {
            result.loadIdentity();
        } else {
            double baseX = base.getX();
            double baseY = base.getY();
            double baseZ = base.getZ();
            double sineTheta = MyMath.hypotenuseDouble(baseX, baseY, baseZ);
            sineTheta = MyMath.clamp(sineTheta, 0.0, 1.0);
            if (sineTheta == 0.0) {
                result.loadIdentity();
            } else {
                double theta = Math.asin(sineTheta);
                float w = (float) Math.cos(exponent * theta);
                double scale = Math.sin(exponent * theta) / sineTheta;
                float x = (float) (scale * baseX);
                float y = (float) (scale * baseY);
                float z = (float) (scale * baseZ);
                result.set(x, y, z, w);
            }
        }

        return result;
    }

    /**
     * Rotate the input vector using the specified quaternion.
     * <p>
     * This method doesn't assume the quaternion is normalized. Instead,
     * rotation is performed using a normalized version of the quaternion.
     *
     * @param rotation the desired rotation (not null, not zero, unaffected)
     * @param input the vector to rotate (not null, finite, unaffected unless
     * it's {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code input})
     * @return the rotated vector (either {@code storeResult} or a new instance)
     */
    public static Vector3f rotate(
            Quaternion rotation, Vector3f input, Vector3f storeResult) {
        Validate.nonZero(rotation, "rotation");
        Validate.finite(input, "input vector");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        float x = rotation.getX();
        float y = rotation.getY();
        float z = rotation.getZ();
        float w = rotation.getW();
        double lengthSquared = lengthSquared(rotation);
        if (lengthSquared < 0.9999998 || lengthSquared > 1.0000002) {
            double dScale = Math.sqrt(lengthSquared);
            x /= dScale;
            y /= dScale;
            z /= dScale;
            w /= dScale;
        }

        float x2 = x * x;
        float y2 = y * y;
        float z2 = z * z;
        float w2 = w * w;

        float vx = input.x;
        float vy = input.y;
        float vz = input.z;

        result.x = vx * (x2 - y2 - z2 + w2)
                + 2f * y * (x * vy + w * vz) + 2f * z * (x * vz - w * vy);
        result.y = vy * (y2 - z2 - x2 + w2)
                + 2f * z * (y * vz + w * vx) + 2f * x * (y * vx - w * vz);
        result.z = vz * (z2 - x2 - y2 + w2)
                + 2f * x * (z * vx + w * vy) + 2f * y * (z * vy - w * vx);

        return result;
    }

    /**
     * Rotate the input vector using the inverse of the specified quaternion.
     *
     * @param rotation the rotation (not null, not zero, unaffected)
     * @param input the vector to rotate (not null, finite, unaffected unless
     * it's {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code input})
     * @return the rotated vector (either {@code storeResult} or a new instance)
     */
    public static Vector3f rotateInverse(
            Quaternion rotation, Vector3f input, Vector3f storeResult) {
        Validate.nonZero(rotation, "rotation");
        Validate.finite(input, "input vector");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        float x = rotation.getX();
        float y = rotation.getY();
        float z = rotation.getZ();
        float w = rotation.getW();
        double lengthSquared = lengthSquared(rotation);
        if (lengthSquared < 0.9999998 || lengthSquared > 1.0000002) {
            double dScale = Math.sqrt(lengthSquared);
            x /= dScale;
            y /= dScale;
            z /= dScale;
            w /= dScale;
        }

        float x2 = x * x;
        float y2 = y * y;
        float z2 = z * z;
        float w2 = w * w;

        float vx = input.x;
        float vy = input.y;
        float vz = input.z;

        result.x = vx * (x2 - y2 - z2 + w2)
                + 2f * y * (x * vy - w * vz) + 2f * z * (x * vz + w * vy);
        result.y = vy * (y2 - z2 - x2 + w2)
                + 2f * z * (y * vz - w * vx) + 2f * x * (y * vx + w * vz);
        result.z = vz * (z2 - x2 - y2 + w2)
                + 2f * x * (z * vx - w * vy) + 2f * y * (z * vy + w * vx);

        return result;
    }

    /**
     * Interpolate between 2 normalized quaternions using spherical linear
     * (Slerp) interpolation.
     * <p>
     * The caller is responsible for flipping the sign of {@code q0} or
     * {@code q1} when it's appropriate to do so.
     *
     * @param t the weight given to {@code q1} (&ge;0, &le;1)
     * @param q0 the function value at t=0 (not null, norm approximately equal
     * to 1, unaffected unless it's {@code storeResult})
     * @param q1 the function value at t=1 (not null, norm approximately equal
     * to 1, unaffected unless it's {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code q0} or {@code q1})
     * @return an interpolated value (either {@code storeResult} or a new
     * instance)
     */
    public static Quaternion slerp(
            float t, Quaternion q0, Quaternion q1, Quaternion storeResult) {
        assert Validate.fraction(t, "weight");
        assert validateUnit(q0, "q0", 0.0001f);
        assert validateUnit(q1, "q1", 0.0001f);
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        Quaternion q0inverse = conjugate(q0, null); // TODO garbage
        Quaternion ratio = q0inverse.multLocal(q1);
        Quaternion power = pow(ratio, t, ratio);
        result.set(q0);
        result.multLocal(power);

        return result;
    }

    /**
     * Interpolate between 4 normalized quaternions using the Squad function.
     * The caller is responsible for flipping signs when it's appropriate to do
     * so.
     *
     * @param t the weight given to {@code q} (&ge;0, &le;1)
     * @param p function value at t=0 (not null, norm approximately equal to 1,
     * unaffected unless it's {@code storeResult})
     * @param a the first control point (not null, norm approximately equal to
     * 1, unaffected unless it's {@code storeResult})
     * @param b the 2nd control point (not null, norm approximately equal to 1,
     * unaffected unless it's {@code storeResult})
     * @param q function value at t=1 (not null, norm approximately equal to 1,
     * unaffected unless it's {@code storeResult})
     * @param storeResult storage for the result (modified if not null)
     * @return the interpolated value (either {@code storeResult} or a new
     * instance)
     */
    public static Quaternion squad(float t, Quaternion p, Quaternion a,
            Quaternion b, Quaternion q, Quaternion storeResult) {
        assert Validate.fraction(t, "weight");
        assert validateUnit(p, "p", 0.0001f);
        assert validateUnit(a, "a", 0.0001f);
        assert validateUnit(b, "b", 0.0001f);
        assert validateUnit(q, "q", 0.0001f);
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        Quaternion qSlerp = slerp(t, p, q, null);
        Quaternion aSlerp = slerp(t, a, b, null);
        slerp(2f * t * (1f - t), qSlerp, aSlerp, result);

        return result;
    }

    /**
     * Return Squad parameter "a" for a continuous first derivative at the
     * middle point of 3 specified control points.
     *
     * @param q0 the previous control point (not null, norm approximately equal
     * to 1, unaffected unless it's {@code storeResult})
     * @param q1 the current control point (not null, norm approximately equal
     * to 1, unaffected unless it's {@code storeResult})
     * @param q2 the following control point (not null, norm approximately equal
     * to 1, unaffected unless it's {@code storeResult})
     * @param storeResult storage for the result (modified if not null)
     * @return the Squad parameter (either {@code storeResult} or a new
     * instance)
     */
    public static Quaternion squadA(Quaternion q0, Quaternion q1,
            Quaternion q2, Quaternion storeResult) {
        assert validateUnit(q0, "q0", 0.0001f);
        assert validateUnit(q1, "q1", 0.0001f);
        assert validateUnit(q2, "q2", 0.0001f);
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        Quaternion q1c = conjugate(q1, null);
        Quaternion turn0 = q1c.mult(q0);
        Quaternion logTurn0 = log(turn0, turn0);
        Quaternion turn2 = q1c.mult(q2);
        Quaternion logTurn2 = log(turn2, turn2);
        Quaternion sum = logTurn2.addLocal(logTurn0);
        sum.multLocal(-0.25f);
        Quaternion exp = exp(sum, sum);
        result.set(q1);
        result.multLocal(exp);

        return result;
    }

    /**
     * Standardize a Quaternion in preparation for hashing.
     *
     * @param input the input value (not null, unaffected unless it's
     * {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code input})
     * @return an equivalent Quaternion without negative zeros (either
     * storeResult or a new instance)
     */
    public static Quaternion standardize(
            Quaternion input, Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        float w = input.getW();
        float x = input.getX();
        float y = input.getY();
        float z = input.getZ();
        w = MyMath.standardize(w);
        x = MyMath.standardize(x);
        y = MyMath.standardize(y);
        z = MyMath.standardize(z);
        result.set(x, y, z, w);

        return result;
    }

    /**
     * Validate a normalized quaternion as a method argument.
     *
     * @param q the Quaternion to validate (not null, unaffected)
     * @param description description of the Quaternion
     * @param tolerance for the norm (&ge;0)
     * @return true
     * @throws IllegalArgumentException if the norm is out of tolerance
     * @throws NullPointerException if the Quaternion is null
     */
    public static boolean validateUnit(
            Quaternion q, String description, float tolerance) {
        assert Validate.nonNull(q, description);

        double norm = lengthSquared(q);
        double delta = Math.abs(1.0 - norm);
        if (!(delta <= tolerance)) {
            String what;
            if (description == null) {
                what = "quaternion argument";
            } else {
                what = description;
            }
            logger.log(Level.SEVERE, "norm({0})={1}", new Object[]{what, norm});
            String message = String.format(
                    "norm(%s) must be within %f of 1.", what, tolerance);
            throw new IllegalArgumentException(message);
        }

        return true;
    }
}
