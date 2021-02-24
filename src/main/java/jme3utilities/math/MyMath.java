/*
 Copyright (c) 2013-2021, Stephen Gold
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

import java.util.logging.Logger;

/**
 * Mathematical utility methods. TODO method to combine 2 transforms
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MyMath {
    // *************************************************************************
    // constants and loggers

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
     * Determine the root sum of squares of some single-precision values.
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
     * Determine the root sum of squares of some double-precision values.
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
     * Interpolate between (or extrapolate from) 2 single-precision values using
     * linear (Lerp) *polation. No rounding error is introduced when y0==y1.
     *
     * @param t descaled parameter value (0&rarr;y0, 1&rarr;y1)
     * @param y0 function value at t=0
     * @param y1 function value at t=1
     * @return an interpolated function value
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
     * Compute the sum of squares of some single-precision values.
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
}
