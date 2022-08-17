/*
 Copyright (c) 2017-2022, Stephen Gold
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
import jme3utilities.Validate;

/**
 * Mathematical utility methods.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MyQuaternion { // TODO finalize the class
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
    public static void accumulateScaled(Quaternion total, Quaternion input,
            float scale) {
        assert Validate.nonNull(total, "total");
        assert Validate.nonNull(input, "input");

        float x = total.getX() + input.getX() * scale;
        float y = total.getY() + input.getY() * scale;
        float z = total.getZ() + input.getZ() * scale;
        float w = total.getW() + input.getW() * scale;
        total.set(x, y, z, w);
    }

    /**
     * Find the cardinal rotation most similar to the specified input. A
     * cardinal rotation is one for which the rotation angles on all 3 axes are
     * integer multiples of Pi/2 radians.
     *
     * @param input (not null, modified)
     */
    public static void cardinalizeLocal(Quaternion input) {
        assert Validate.nonNull(input, "input");

        normalizeLocal(input);
        /*
         * Generate each of the 24 cardinal rotations.
         */
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
     * Determine the conjugate of a Quaternion. For unit quaternions, the
     * conjugate is a faster way to calculate the inverse.
     *
     * @param q input value (not null, unaffected unless it's
     * {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code q})
     * @return a conjugate quaternion (either storeResult or a new instance)
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
     * Determine the dot (scalar) product of 2 quaternions. This method returns
     * a double-precision value for precise calculation of angles.
     *
     * @param q1 the first Quaternion (not null, unaffected)
     * @param q2 the 2nd Quaternion (not null, unaffected)
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
     * Determine the exponential of a pure Quaternion.
     *
     * @param q input value (not null, unaffected, w=0)
     * @param storeResult storage for the result (modified if not null)
     * @return a unit quaternion (either storeResult or a new instance)
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
     * @param q the input (not null, unaffected)
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
     * Accepts any non-zero value for w.
     *
     * @param q input value (not null, unaffected)
     * @return true for a rotation identity, otherwise false
     */
    public static boolean isRotationIdentity(Quaternion q) {
        float qx = q.getX();
        float qy = q.getY();
        float qz = q.getZ();
        float qw = q.getW();

        if (qx == 0f && qy == 0f && qz == 0f && qw != 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Test for a zero Quaternion.
     *
     * @param q the input (not null, unaffected)
     * @return true if the Quaternion equals (0,0,0,0), false otherwise
     */
    public static boolean isZero(Quaternion q) {
        float qx = q.getX();
        float qy = q.getY();
        float qz = q.getZ();
        float qw = q.getW();

        if (qx == 0f && qy == 0f && qz == 0f && qw == 0f) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Determine the squared length of a Quaternion. Unlike
     * {@link com.jme3.math.Quaternion#norm()}, this method returns a
     * double-precision value for precise comparison of lengths.
     *
     * @param q input (not null, unaffected)
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
     * Determine the natural logarithm of a unit quaternion. Generally the
     * logarithm isn't itself a unit.
     *
     * @param q input value (not null, unaffected unless it's
     * {@code storeResult}, norm=1)
     * @param storeResult storage for the result (modified if not null, may be
     * {@code q})
     * @return a pure Quaternion (either storeResult or a new instance)
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
     * @param a the first input quaternion (not null, unaffected)
     * @param b the 2nd input quaternion (not null, unaffected)
     * @return true if distinct, otherwise false
     */
    public static boolean ne(Quaternion a, Quaternion b) {
        assert Validate.nonNull(a, "first input quaternion");
        assert Validate.nonNull(b, "2nd input quaternion");

        boolean result = a.getW() != b.getW()
                || a.getX() != b.getX()
                || a.getY() != b.getY()
                || a.getZ() != b.getZ();
        return result;
    }

    /**
     * Normalize the specified Quaternion in place.
     *
     * @param input (not null, modified)
     */
    public static void normalizeLocal(Quaternion input) {
        assert Validate.nonNull(input, "input");

        double lengthSquared = lengthSquared(input);
        double dScale = Math.sqrt(lengthSquared);
        float fScale = (float) dScale;
        if (fScale != 0f && fScale != 1f) {
            input.multLocal(1f / fScale);
        }
    }

    /**
     * Raise a unit quaternion to the specified real power.
     *
     * @param base input value (not null, unaffected unless it's
     * {@code storeResult}, norm=1)
     * @param exponent the exponent
     * @param storeResult storage for the result (modified if not null, may be
     * {@code base})
     * @return a unit quaternion (either storeResult or a new instance)
     */
    public static Quaternion pow(Quaternion base, float exponent,
            Quaternion storeResult) {
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
     * Interpolate between 2 unit quaternions using spherical linear (Slerp)
     * interpolation. This method always produces a unit, and doesn't trash q1.
     * The caller is responsible for flipping the sign of q0 or q1 when it's
     * appropriate to do so.
     *
     * @param t descaled parameter value (&ge;0, &le;1)
     * @param q0 function value at t=0 (not null, unaffected unless it's {@code
     * storeResult}, norm=1)
     * @param q1 function value at t=1 (not null, unaffected unless it's {@code
     * storeResult}, norm=1)
     * @param storeResult storage for the result (modified if not null, may be
     * {@code q0} or {@code q1})
     * @return an interpolated unit quaternion (either storeResult or a new
     * instance)
     */
    public static Quaternion slerp(float t, Quaternion q0, Quaternion q1,
            Quaternion storeResult) {
        assert Validate.inRange(t, "t", 0f, 1f);
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
     * Interpolate between 4 unit quaternions using the Squad function. The
     * caller is responsible for flipping signs when it's appropriate to do so.
     *
     * @param t descaled parameter value (&ge;0, &le;1)
     * @param p function value at t=0 (not null, unaffected, norm=1)
     * @param a the first control point (not null, unaffected, norm=1)
     * @param b the 2nd control point (not null, unaffected, norm=1)
     * @param q function value at t=1 (not null, unaffected, norm=1)
     * @param storeResult storage for the result (modified if not null)
     * @return interpolated unit quaternion (either storeResult or a new
     * instance)
     */
    public static Quaternion squad(float t, Quaternion p, Quaternion a,
            Quaternion b, Quaternion q, Quaternion storeResult) {
        assert Validate.inRange(t, "t", 0f, 1f);
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
     * Determine Squad parameter "a" for a continuous first derivative at the
     * middle point of 3 specified control points.
     *
     * @param q0 previous control point (not null, unaffected, norm=1)
     * @param q1 current control point (not null, unaffected, norm=1)
     * @param q2 following control point (not null, unaffected, norm=1)
     * @param storeResult storage for the result (modified if not null)
     * @return a unit quaternion for use as a Squad parameter (either
     * storeResult or a new instance)
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
     * @param input (not null, unaffected unless it's {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code input})
     * @return an equivalent Quaternion without negative zeros (either
     * storeResult or a new instance)
     */
    public static Quaternion standardize(Quaternion input,
            Quaternion storeResult) {
        assert Validate.nonNull(input, "input quaternion");
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
     * Validate a unit quaternion as a method argument.
     *
     * @param q Quaternion to validate (not null, unaffected)
     * @param description description of the Quaternion
     * @param tolerance for the norm (&ge;0)
     * @return true
     * @throws IllegalArgumentException if the norm is out of tolerance
     * @throws NullPointerException if the Quaternion is null
     */
    public static boolean validateUnit(Quaternion q, String description,
            float tolerance) {
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
