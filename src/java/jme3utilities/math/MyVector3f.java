/*
 Copyright (c) 2013-2019, Stephen Gold
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
import com.jme3.math.Line;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import jme3utilities.MySpatial;
import jme3utilities.MyString;
import jme3utilities.Validate;

/**
 * Utility methods for 3-D vectors.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MyVector3f {
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
    final private static Logger logger
            = Logger.getLogger(MyVector3f.class.getName());
    /**
     * pattern for matching a Vector3f
     */
    final private static Pattern pattern = Pattern.compile(
            "\\(\\s*([^,]+),\\s+([^,]+),\\s+(\\S+)\\s*\\)");
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_X}
     */
    final private static Vector3f unitX = new Vector3f(1f, 0f, 0f);
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_Z}
     */
    final private static Vector3f unitZ = new Vector3f(0f, 0f, 1f);
    /**
     * array of cardinal axes
     */
    final private static Vector3f cardinalAxes[] = {
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
        Validate.nonNull(total, "total");
        Validate.nonNull(input, "input");

        total.x += input.x * scale;
        total.y += input.y * scale;
        total.z += input.z * scale;
    }

    /**
     * Calculate the altitude angle of a non-zero offset.
     *
     * @param offset difference of world coordinates (not null, not zero,
     * unaffected)
     * @return angle above the X-Z plane (in radians, &le;Pi/2, &ge;-Pi/2)
     */
    public static float altitude(Vector3f offset) {
        Validate.nonZero(offset, "offset");

        float xzRange = MyMath.hypotenuse(offset.x, offset.z);
        float result = (float) Math.atan2(offset.y, xzRange);

        assert result <= FastMath.HALF_PI : result;
        assert result >= -FastMath.HALF_PI : result;
        return result;
    }

    /**
     * Test whether 3 locations are collinear.
     *
     * @param point1 coordinates of the first location (not null, unaffected)
     * @param point2 coordinates of the 2nd location (not null, unaffected)
     * @param point3 coordinates of the 3rd location (not null, unaffected)
     * @param tolerance2 for coincidence (in squared units, &ge;0)
     * @return true if collinear, otherwise false
     */
    public static boolean areCollinear(Vector3f point1, Vector3f point2,
            Vector3f point3, float tolerance2) {
        Validate.nonNull(point1, "first location");
        Validate.nonNull(point2, "2nd location");
        Validate.nonNegative(tolerance2, "tolerance");
        /*
         * Shortcut:
         * If point1 and point3 coincide, then the 3 points are collinear.
         */
        Vector3f offset3 = point3.subtract(point1);
        double normSquared3 = lengthSquared(offset3);
        if (normSquared3 <= tolerance2) {
            return true;
        }
        /*
         * The long way:
         * Calculate the projection of offset2 onto offset3.
         *
         * Don't use Vector3f.project() because (as of jME 3.0.10) it contained
         * a logic bug.
         */
        Vector3f offset2 = point2.subtract(point1);
        double scaleFactor = dot(offset2, offset3) / normSquared3;
        Vector3f projection = offset3.mult((float) scaleFactor);
        /*
         * If the projection coincides with offset2,
         * then the 3 points are collinear.
         */
        boolean result = doCoincide(projection, offset2, tolerance2);

        return result;
    }

    /**
     * Generate an axis-aligned vector with the specified length.
     *
     * @param axisIndex 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @param length how long (ge;0)
     * @param storeResult storage for the result (modified if not null)
     * @return an axis-aligned vector (either storeResult or a new instance)
     */
    public static Vector3f axisVector(int axisIndex, float length,
            Vector3f storeResult) {
        Validate.inRange(axisIndex, "axis index", firstAxis, lastAxis);
        Validate.nonNegative(length, "length");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.zero();
        result.set(axisIndex, length);

        return result;
    }

    /**
     * Calculate the azimuth angle of an offset.
     *
     * @param offset difference of world coordinates (not null, unaffected)
     * @return horizontal angle in radians (measured CW from the X axis) or 0 if
     * the vector is zero or parallel to the Y axis.
     */
    public static float azimuth(Vector3f offset) {
        float result = (float) Math.atan2(offset.z, offset.x);
        return result;
    }

    /**
     * Find the cardinal direction most similar to the specified input.
     *
     * @param input (not null, modified)
     */
    public static void cardinalizeLocal(Vector3f input) {
        normalizeLocal(input);
        /*
         * Generate each of the 6 cardinal directions.
         */
        Vector3f bestCardinalDirection = new Vector3f();
        float bestDot = -2f;
        for (Vector3f x : cardinalAxes) {
            /*
             * Measure the similarity of the 2 directions
             * using their dot product.
             */
            float dot = x.dot(input);
            if (dot > bestDot) {
                bestDot = dot;
                bestCardinalDirection.set(x);
            }
        }

        input.set(bestCardinalDirection);
    }

    /**
     * Compare 2 vectors lexicographically, with the X-component having
     * priority, then the Y-component. For this purpose, 0 and -0 are distinct.
     *
     * @param v1 the first input vector (not null, unaffected)
     * @param v2 the 2nd input vector (not null, unaffected)
     * @return 0 if v1 is equal to v2; negative if v1 comes before v2; positive
     * if v1 comes after v2
     */
    public static int compare(Vector3f v1, Vector3f v2) {
        Validate.nonNull(v1, "v1");
        Validate.nonNull(v2, "v2");

        int result = Float.compare(v1.x, v2.x);
        if (result == 0) {
            result = Float.compare(v1.y, v2.y);
            if (result == 0) {
                result = Float.compare(v1.z, v2.z);
            }
        }

        return result;
    }

    /**
     * Calculate the sample covariance of a collection of vectors.
     *
     * @param collection the sample vectors (not null, at least 2, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return the unbiased sample covariance (either storeResult or a new
     * matrix, not null)
     */
    public static Matrix3f covariance(Collection<Vector3f> collection,
            Matrix3f storeResult) {
        Validate.nonEmpty(collection, "collection");
        int numSamples = collection.size();
        assert numSamples > 1 : numSamples;
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        Vector3f sampleMean = mean(collection, null);
        /*
         * Accumulate sums in the upper triangle of the matrix.
         */
        result.zero();
        float[] aboveMean = new float[3];
        for (Vector3f location : collection) {
            aboveMean[0] = location.x - sampleMean.x;
            aboveMean[1] = location.y - sampleMean.y;
            aboveMean[2] = location.z - sampleMean.z;
            for (int rowIndex = 0; rowIndex < 3; ++rowIndex) {
                for (int columnIndex = rowIndex; columnIndex < 3; ++columnIndex) {
                    float sum = result.get(rowIndex, columnIndex);
                    sum += aboveMean[rowIndex] * aboveMean[columnIndex];
                    result.set(rowIndex, columnIndex, sum);
                }
            }
        }
        /*
         * Multiply sums by 1/(N-1) and fill in the lower triangle.
         */
        float nMinus1 = numSamples - 1;
        for (int rowIndex = 0; rowIndex < 3; ++rowIndex) {
            for (int columnIndex = rowIndex; columnIndex < 3; ++columnIndex) {
                float sum = result.get(rowIndex, columnIndex);
                float element = sum / nMinus1;
                result.set(rowIndex, columnIndex, element);
                result.set(columnIndex, rowIndex, element);
            }
        }

        return result;
    }

    /**
     * Generate a textual description of a Vector3f value.
     *
     * @param vector the value to describe (may be null, unaffected)
     * @return a description (not null, not empty)
     */
    public static String describe(Vector3f vector) {
        String result;
        if (vector == null) {
            result = "null";

        } else if (isScaleUniform(vector)) {
            result = "xyz=" + MyString.describe(vector.x);

        } else {
            StringBuilder builder = new StringBuilder(40);
            if (vector.x != 0f) {
                builder.append("x=");
                String x = MyString.describe(vector.x);
                builder.append(x);
            }
            if (vector.y != 0f) {
                if (builder.length() > 0) {
                    builder.append(' ');
                }
                builder.append("y=");
                String y = MyString.describe(vector.y);
                builder.append(y);
            }
            if (vector.z != 0f) {
                if (builder.length() > 0) {
                    builder.append(' ');
                }
                builder.append("z=");
                String z = MyString.describe(vector.z);
                builder.append(z);
            }

            result = builder.toString();
        }

        assert result != null;
        assert !result.isEmpty();
        return result;
    }

    /**
     * Generate a textual description of a direction vector.
     *
     * @param v the value to describe (may be null, unaffected)
     * @return a description (not null, not empty)
     */
    public static String describeDirection(Vector3f v) {
        String result;
        if (v == null) {
            result = "null";

        } else {
            StringBuilder builder = new StringBuilder(40);

            builder.append("dx=");
            String x = MyString.describeFraction(v.x);
            builder.append(x);

            builder.append(" dy=");
            String y = MyString.describeFraction(v.y);
            builder.append(y);

            builder.append(" dz=");
            String z = MyString.describeFraction(v.z);
            builder.append(z);

            result = builder.toString();
        }

        assert result != null;
        assert !result.isEmpty();
        return result;
    }

    /**
     * Calculate the squared distance between 2 vectors. Unlike
     * {@link com.jme3.math.Vector3f#distanceSquared(Vector3f)}, this method
     * returns a double-precision value for precise comparison of distances.
     *
     * @param vector1 the first input vector (not null, unaffected)
     * @param vector2 the 2nd input vector (not null, unaffected)
     * @return the squared distance (&ge;0)
     */
    public static double distanceSquared(Vector3f vector1, Vector3f vector2) {
        double dx = vector1.x - vector2.x;
        double dy = vector1.y - vector2.y;
        double dz = vector1.z - vector2.z;
        double result = dx * dx + dy * dy + dz * dz;

        return result;
    }

    /**
     * Calculate square of the distance between a location and a line segment.
     *
     * @param location coordinates of input location (not null, unaffected)
     * @param segStart coordinates of start of segment (not null, unaffected)
     * @param segEnd coordinates of end of segment (not null, unaffected)
     * @param storeClosest storage for the coordinates of the closest location
     * on the segment (modified if not null)
     * @return the squared distance (&ge;0)
     */
    public static double distanceSquaredToSegment(Vector3f location,
            Vector3f segStart, Vector3f segEnd, Vector3f storeClosest) {
        /*
         * Calculate direction of the segment.
         */
        Vector3f segOffset = segEnd.subtract(segStart);
        /*
         * If the segment has zero length, return the squared distance
         * to segStart.
         */
        double segLengthSquared = lengthSquared(segOffset);
        if (segLengthSquared == 0.0) {
            if (storeClosest != null) {
                storeClosest.set(segStart);
            }
            double result = distanceSquared(segStart, location);
            return result;
        }
        /*
         * Extend the segment into a straight line and find the
         * parametric value t at closest approach.
         */
        Vector3f locOffset = location.subtract(segStart);
        double dot = dot(locOffset, segOffset);
        double t = dot / segLengthSquared;
        /*
         * Calculate offset of the closest point on the segment.
         */
        float tPrime = FastMath.saturate((float) t);
        Vector3f closestOffset = segOffset.mult(tPrime);
        if (storeClosest != null) {
            storeClosest.set(segStart);
            storeClosest.addLocal(closestOffset);
        }
        double result = distanceSquared(closestOffset, locOffset);

        assert result >= 0.0 : result;
        return result;
    }

    /**
     * Test whether 2 locations coincide.
     *
     * @param point1 coordinates of the first location (not null, unaffected)
     * @param point2 coordinates of the 2nd location (not null, unaffected)
     * @param tolerance2 for coincidence (in squared units, &ge;0)
     * @return true if they coincide, otherwise false
     */
    public static boolean doCoincide(Vector3f point1, Vector3f point2,
            float tolerance2) {
        Validate.nonNull(point1, "first point");
        Validate.nonNull(point2, "second point");
        Validate.nonNegative(tolerance2, "tolerance");

        double d2 = distanceSquared(point1, point2);
        if (d2 > tolerance2) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Calculate the dot (scalar) product of 2 vectors. Unlike
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
     * Test whether 2 vectors are distinct, without distinguishing 0 from -0.
     *
     * @param v1 the first input vector (not null, unaffected)
     * @param v2 the 2nd input vector (not null, unaffected)
     * @return true if equal, otherwise false
     */
    public static boolean eq(Vector3f v1, Vector3f v2) {
        Validate.nonNull(v1, "first input vector");
        Validate.nonNull(v2, "2nd input vector");

        boolean result = v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
        return result;
    }

    /**
     * Generate a direction from altitude and azimuth angles.
     *
     * @param altitude angle above the X-Z plane (radians toward +Y)
     * @param azimuth angle in the X-Z plane (radians CCW from +X)
     * @return a new unit vector
     */
    public static Vector3f fromAltAz(float altitude, float azimuth) {
        Quaternion elevate = new Quaternion();
        elevate.fromAngleNormalAxis(altitude, unitZ);
        Vector3f elevation = elevate.mult(unitX);
        Vector3f direction = yRotate(elevation, azimuth);

        assert direction.isUnitVector() : direction;
        return direction;
    }

    /**
     * Generate an orthonormal basis that includes the specified vector.
     *
     * @param in1 input direction for the first basis vector (not null, not
     * zero, modified)
     * @param store2 storage for the 2nd basis vector (not null, modified)
     * @param store3 storage for the 3nd basis vector (not null, modified)
     */
    public static void generateBasis(Vector3f in1, Vector3f store2,
            Vector3f store3) {
        Validate.nonZero(in1, "starting direction");
        Validate.nonNull(store2, "2nd basis vector");
        Validate.nonNull(store3, "3nd basis vector");

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
     * Calculate the horizontal direction of an offset in world space.
     *
     * @param offset difference of world coordinates (not null, unaffected)
     * @return a unit vector or a zero vector
     */
    public static ReadXZ horizontalDirection(Vector3f offset) {
        Validate.nonNull(offset, "offset");

        VectorXZ horizontalOffset = new VectorXZ(offset);
        ReadXZ result = horizontalOffset.normalize();

        return result;
    }

    /**
     * Calculate where (if at all) 2 line segments intersect.
     *
     * @param start1 coordinates of start of the first segment (not null)
     * @param end1 coordinates of end of the first segment (not null)
     * @param start2 coordinates of start of the 2nd segment (not null)
     * @param end2 coordinates of end of the 2nd segment (not null)
     * @param tolerance2 for coincidence (in squared units, &ge;0)
     * @return a new coordinate vector, or null if no intersection found
     */
    public static Vector3f intersectSegments(Vector3f start1, Vector3f end1,
            Vector3f start2, Vector3f end2, float tolerance2) {
        Validate.nonNull(start1, "start of first segment");
        Validate.nonNull(start2, "start of 2nd segment");
        Validate.nonNull(end2, "end of 2nd segment");
        Validate.nonNegative(tolerance2, "tolerance2");
        /*
         * Calculate direction of the first segment.
         */
        Vector3f offset1 = end1.subtract(start1);
        /*
         * If the first segment has zero length, test its start for coincidence
         * with the 2nd segment.
         */
        double ls1 = lengthSquared(offset1);
        if (ls1 == 0.0) {
            Vector3f closest = new Vector3f();
            double ds = distanceSquaredToSegment(start1, start2, end2, closest);
            if (ds > tolerance2) {
                return null;
            } else {
                return closest;
            }
        }
        /*
         * Calculate direction of the 2nd segment.
         */
        Vector3f offset2 = end2.subtract(start2);
        /*
         * If the 2nd segment has zero length, test its start for coincidence
         * with the first segment.
         */
        double ls2 = lengthSquared(offset1);
        if (ls2 == 0.0) {
            Vector3f closest = new Vector3f();
            double ds = distanceSquaredToSegment(start2, start1, end1, closest);
            if (ds > tolerance2) {
                return null;
            } else {
                return closest;
            }
        }
        /*
         * Both segments have positive length.
         * Find a vector N perpendicular to both directions.
         */
        Vector3f n = offset2.cross(offset1);
        if (lengthSquared(n) <= tolerance2) {
            /*
             * The segments are parallel.
             */
            Vector3f result = intersectParallelSegments(
                    start1, end1, start2, end2, tolerance2);
            return result;
        }
        /*
         * The segments are not parallel.
         * Extend them into straight lines and find the
         * locations of closest approach (C1 and C2) on each line.
         * The segment and side intersect if and only if C1 and C2 not only
         * coincide but also lie on their respective segment/side.
         */
        Vector3f n1 = offset2.cross(n);
        Vector3f n2 = offset1.cross(n);
        float t1 = start1.subtract(start2).dot(n2) / offset2.dot(n2);
        float t2 = start2.subtract(start1).dot(n1) / offset1.dot(n1);
        Vector3f c1 = offset2.mult(t1).add(start2);
        Vector3f c2 = offset1.mult(t2).add(start1);
        if (!doCoincide(c1, c2, tolerance2)) {
            return null;
        }
        double fuzz1 = tolerance2 / ls2;
        if (t1 < 0f && t1 * t1 > fuzz1) {
            return null;
        }
        float ct1 = 1f - t1;
        if (ct1 < 0f && ct1 * ct1 > fuzz1) {
            return null;
        }
        double fuzz2 = tolerance2 / ls1;
        if (t2 < 0 && t2 * t2 > fuzz2) {
            return null;
        }
        float ct2 = 1f - t2;
        if (ct2 < 0f && ct2 * ct2 > fuzz2) {
            return null;
        }

        return c1;
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
     * Test whether b is inside the axis-aligned bounding box formed by a and c.
     *
     * @param a the location of one corner of the box (not null, unaffected)
     * @param b the location of being tested (not null, unaffected)
     * @param c the location of the corner opposite a (not null, unaffected)
     * @return true if b is between a and c (inclusive), otherwise false
     */
    public static boolean isBetween(Vector3f a, Vector3f b, Vector3f c) {
        boolean result = MyMath.isBetween(a.x, b.x, c.x)
                && MyMath.isBetween(a.y, b.y, c.y)
                && MyMath.isBetween(a.z, b.z, c.z);
        return result;
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
     * Calculate the squared length of a vector. Unlike
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
     * *polation. Unlike
     * {@link com.jme3.math.FastMath#interpolateLinear(float, com.jme3.math.Vector3f, com.jme3.math.Vector3f, com.jme3.math.Vector3f)},
     * no rounding error is introduced when v1==v2.
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
        Validate.nonNull(v0, "v0");
        Validate.nonNull(v1, "v1");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.x = MyMath.lerp(t, v0.x, v1.x);
        result.y = MyMath.lerp(t, v0.y, v1.y);
        result.z = MyMath.lerp(t, v0.z, v1.z);

        return result;
    }

    /**
     * Find the point on line1 closest to line2.
     *
     * @param line1 line1 (not null, unaffected)
     * @param line2 line2 (not null, unaffected)
     * @return a new coordinate vector, or null if the lines are parallel
     */
    public static Vector3f lineMeetsLine(Line line1, Line line2) {
        Vector3f d1 = line1.getDirection();
        Vector3f d2 = line2.getDirection();
        Vector3f p1 = line1.getOrigin();
        Vector3f p2 = line2.getOrigin();
        Vector3f n = d1.cross(d2);
        Vector3f n2 = d2.cross(n);

        Vector3f result = null;
        float denom = d1.dot(n2);
        if (denom != 0f) {
            float numer = p2.dot(n2) - p1.dot(n2);
            result = d1.mult(numer / denom);
            result.addLocal(p1);
        }

        return result;
    }

    /**
     * Find the specified intersection between a specified line and a specified
     * sphere. If there's no intersection, find the point on the sphere closest
     * to the line.
     *
     * @param line the line (not null, unaffected)
     * @param center center of the sphere (not null, unaffected)
     * @param radius size of the sphere (&ge;0)
     * @param farSide which intersection to use: true&rarr;far side of sphere,
     * false&rarr;near side
     * @return a new coordinate vector
     */
    public static Vector3f lineMeetsSphere(Line line, Vector3f center,
            float radius, boolean farSide) {
        Validate.nonNull(center, "center");
        Validate.nonNegative(radius, "radius");

        Vector3f direction = line.getDirection().normalize();
        Vector3f lineOrigin = line.getOrigin();
        Vector3f offset = lineOrigin.subtract(center);
        float dDotV = direction.dot(offset);
        float normV2 = offset.lengthSquared();
        float discriminant = dDotV * dDotV - normV2 + radius * radius;

        Vector3f result;
        if (discriminant >= 0f) {
            /*
             * Calculate the pseudodistance from the line's origin
             * to the intersection.
             */
            float t = -dDotV;
            if (farSide) {
                t += FastMath.sqrt(discriminant);
            } else {
                t -= FastMath.sqrt(discriminant);
            }
            /*
             * Calculate the position of that point on the line.
             */
            result = direction.mult(t);
            result.addLocal(lineOrigin);

        } else {
            /*
             * Calculate the line's closest approach to the center.
             */
            Vector3f projection = direction.mult(dDotV);
            result = offset.subtract(projection);
            /*
             * Scale and translate to the surface of the sphere.
             */
            float factor = radius / result.length();
            assert factor <= 1f : factor;
            result.multLocal(factor);
            result.addLocal(center);
        }

        return result;
    }

    /**
     * Transform a direction vector from world coordinates to the local
     * coordinates of the specified spatial.
     *
     * @param worldDirection input direction to transform (not null, unaffected,
     * length&gt;0)
     * @param spatial which spatial (not null)
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector (either storeResult or a new instance)
     */
    public static Vector3f localizeDirection(Vector3f worldDirection,
            Spatial spatial, Vector3f storeResult) {
        Validate.nonZero(worldDirection, "direction");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        if (MySpatial.isIgnoringTransforms(spatial)) {
            result.set(worldDirection);
        } else {
            spatial.worldToLocal(worldDirection, result);
        }

        double lengthSquared = lengthSquared(result);
        double scaleFactor = 1.0 / Math.sqrt(lengthSquared);
        result.multLocal((float) scaleFactor);

        assert result.isUnitVector();
        return result;
    }

    /**
     * Calculate the component-wise maximum of absolute values of 2 vectors.
     *
     * @param vector1 the first input vector (not null, unaffected unless it's
     * also storeResult)
     * @param vector2 the 2nd input vector (not null, unaffected unless it's
     * also storeResult)
     * @param storeResult storage for the result (modified if not null, may be
     * vector1 or vector2)
     * @return the max magnitude for each axis (either storeResult or a new
     * vector)
     */
    public static Vector3f maxAbs(Vector3f vector1, Vector3f vector2,
            Vector3f storeResult) {
        Validate.nonNull(vector1, "vector1");
        Validate.nonNull(vector2, "vector2");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.x = Math.max(Math.abs(vector1.x), Math.abs(vector2.x));
        result.y = Math.max(Math.abs(vector1.y), Math.abs(vector2.y));
        result.x = Math.max(Math.abs(vector1.z), Math.abs(vector2.z));

        return result;
    }

    /**
     * Calculate the arithmetic mean of a non-empty collection of vectors.
     *
     * @param collection the vectors to average (not null, not empty,
     * unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return the mean (either storeResult or a new instance, not null)
     */
    public static Vector3f mean(Collection<Vector3f> collection,
            Vector3f storeResult) {
        Validate.nonEmpty(collection, "collection");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.zero();
        for (Vector3f location : collection) {
            result.addLocal(location);
        }
        int count = collection.size();
        result.divideLocal(count);

        return result;
    }

    /**
     * Calculate the midpoint between 2 locations.
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
     * Find the most distant pair of locations in a specified list. TODO
     * array/buffer versions
     *
     * @param locations list of input coordinates (not null, all elements
     * non-null, unaffected)
     * @return a new array of 2 locations, or null if no pairs found
     */
    public static Vector3f[] mostRemote(List<Vector3f> locations) {
        Validate.nonNull(locations, "locations");

        double largestSD = -1.0;
        Vector3f[] result = null;

        for (int i = 0; i < locations.size(); ++i) {
            Vector3f iVector = locations.get(i);
            for (int j = i + 1; j < locations.size(); ++j) {
                Vector3f jVector = locations.get(j);
                double squaredDistance = distanceSquared(iVector, jVector);
                if (squaredDistance > largestSD) {
                    largestSD = squaredDistance;
                    if (result == null) {
                        result = new Vector3f[2];
                    }
                    result[0] = iVector;
                    result[1] = jVector;
                }
            }
        }

        return result;
    }

    /**
     * Test whether 2 vectors are distinct, without distinguishing 0 from -0.
     *
     * @param v1 the first input vector (not null, unaffected)
     * @param v2 the 2nd input vector (not null, unaffected)
     * @return true if distinct, otherwise false
     */
    public static boolean ne(Vector3f v1, Vector3f v2) {
        Validate.nonNull(v1, "first input vector");
        Validate.nonNull(v2, "2nd input vector");

        boolean result = v1.x != v2.x || v1.y != v2.y || v1.z != v2.z;
        return result;
    }

    /**
     * Normalize the specified vector in place. This method is less vulnerable
     * to overflow than {@link com.jme3.math.Vector3f#normalizeLocal()}.
     *
     * @param input (not null, modified)
     */
    public static void normalizeLocal(Vector3f input) {
        Validate.nonNull(input, "input");

        double lengthSquared = lengthSquared(input);
        double dScale = Math.sqrt(lengthSquared);
        float fScale = (float) dScale;
        if (fScale != 0f && fScale != 1f) {
            input.divideLocal(fScale);
        }
    }

    /**
     * Parse a Vector3f from the specified text string.
     *
     * @param textString input text (not null, not empty)
     * @return a new vector, or null if the text is invalid
     */
    public static Vector3f parse(String textString) {
        Validate.nonEmpty(textString, "text string");

        Vector3f result = null;
        Matcher matcher = pattern.matcher(textString);
        boolean valid = matcher.matches();
        if (valid) {
            String xText = matcher.group(1);
            float x = Float.parseFloat(xText);
            String yText = matcher.group(2);
            float y = Float.parseFloat(yText);
            String zText = matcher.group(3);
            float z = Float.parseFloat(zText);
            result = new Vector3f(x, y, z);
        }

        return result;
    }

    /**
     * Project vector1 onto vector2. Don't use
     * {@link com.jme3.math.Vector3f#project(Vector3f)} for this because (as of
     * jME 3.0.10) it contained a logic bug which gave the wrong magnitude when
     * vector2 had length != 1.
     *
     * @param vector1 the first input vector (not null, unaffected unless it's
     * storeResult)
     * @param vector2 the 2nd input vector (length&gt;0, unaffected unless it's
     * storeResult)
     * @param storeResult storage for the result (modified if not null, may be
     * vector1 or vector2)
     * @return a vector with the same direction as vector2 (either storeResult
     * or a new instance)
     */
    public static Vector3f projection(Vector3f vector1, Vector3f vector2,
            Vector3f storeResult) {
        Validate.nonNull(vector1, "vector1");
        Validate.nonZero(vector2, "vector2");

        double lengthSquared = lengthSquared(vector2);
        double dot = dot(vector1, vector2);
        double scaleFactor = dot / lengthSquared;
        Vector3f projection = vector2.mult((float) scaleFactor, storeResult);

        return projection;
    }

    /**
     * Project vector1 onto vector2.
     *
     * @param vector1 the first input vector (not null, unaffected)
     * @param vector2 the 2nd input vector (length&gt;0, unaffected)
     * @return the scalar projection of vector1 onto vector2
     */
    public static float scalarProjection(Vector3f vector1, Vector3f vector2) {
        Validate.nonZero(vector2, "vector2");

        double dot = dot(vector1, vector2);
        double lengthSquared = lengthSquared(vector2);
        double length = Math.sqrt(lengthSquared);
        double projection = dot / length;

        return (float) projection;
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
        Validate.nonNull(input, "input vector");
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.x = MyMath.standardize(input.x);
        result.y = MyMath.standardize(input.y);
        result.z = MyMath.standardize(input.z);

        return result;
    }

    /**
     * Rotate a vector CLOCKWISE about the +Y axis. Note: Used for applying
     * azimuths, which is why its rotation angle convention is non-standard.
     *
     * @param input (not null, unaffected)
     * @param angle the clockwise (LH) angle of rotation in radians
     * @return a new vector
     */
    public static Vector3f yRotate(Vector3f input, float angle) {
        float cosine = FastMath.cos(angle);
        float sine = FastMath.sin(angle);
        float x = cosine * input.x - sine * input.z;
        float y = input.y;
        float z = cosine * input.z + sine * input.x;
        Vector3f result = new Vector3f(x, y, z);

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Test whether all locations in the specified list lie on the line
     * connecting 2 specified locations.
     *
     * @param first coordinates of first location on line (not null, unaffected)
     * @param last coordinates of last location on line (not null, unaffected)
     * @param list coordinates of test locations (not null, all elements
     * non-null, unaffected)
     * @param tolerance2 for coincidence (in squared units, &ge;0)
     * @return true if all middle locations lie on the line, false otherwise
     */
    private static boolean allCollinear(Vector3f first, Vector3f last,
            List<Vector3f> list, float tolerance2) {
        Validate.nonNull(first, "first location");
        Validate.nonNull(list, "list");
        Validate.nonNegative(tolerance2, "tolerance2");

        Vector3f fl = last.subtract(first);
        double normSquaredFL = lengthSquared(fl);
        if (normSquaredFL <= tolerance2) {
            /*
             * The line is ill-defined.
             */
            return true;
        }
        /*
         * Calculate the offset of the last location from the first.
         */
        for (Vector3f middle : list) {
            assert middle != null;
            /*
             * Calculate the offset of the middle location from the first.
             */
            Vector3f fm = middle.subtract(first);
            /*
             * Project FM onto FL.
             *
             * Don't use Vector3f.project() because (as of jME 3.0.10)
             * it contained a logic bug.
             */
            double fm_dot_fl = dot(fm, fl);
            double fraction = fm_dot_fl / normSquaredFL;
            Vector3f projection = fl.mult((float) fraction);
            /*
             * If the projection coincides with FM,
             * then consider the 3 corners to be collinear.
             */
            boolean collin = doCoincide(projection, fm, tolerance2);
            if (!collin) {
                return false;
            }
        }
        return true;
    }

    /**
     * Helper method to determine where (if at all) 2 collinear segments
     * intersect.
     *
     * @param ext coordinates of an outer endpoint (not null, unaffected)
     * @param start1 coordinates of start of the first segment (not null,
     * unaffected)
     * @param end1 coordinates of end of the first segment (not null,
     * unaffected)
     * @param start2 coordinates of start of the 2nd segment (not null,
     * unaffected)
     * @param end2 coordinates of end of the 2nd segment (not null, unaffected)
     * @return a new coordinate vector, or null if no overlap
     */
    private static Vector3f intersectCollinearSegments(Vector3f ext,
            Vector3f start1, Vector3f end1, Vector3f start2, Vector3f end2) {
        assert ext != null;
        assert ext == start1 || ext == end1 || ext == start2 || ext == end2;
        assert start1 != null;
        assert end1 != null;
        assert start2 != null;
        assert end2 != null;
        /*
         * Test the partner of ext against both ends of the other segment.
         */
        Vector3f otherStart, otherEnd;
        if (ext == start1 || ext == end1) {
            otherStart = start2;
            otherEnd = end2;
        } else {
            assert ext == start2 || ext == end2;
            otherStart = start1;
            otherEnd = end1;
        }
        Vector3f partner;
        if (ext == start1) {
            partner = end1;
        } else if (ext == end1) {
            partner = start1;
        } else if (ext == start2) {
            partner = end2;
        } else {
            assert ext == end2;
            partner = start2;
        }

        double sdPartner = distanceSquared(ext, partner);
        double sdOtherStart = distanceSquared(ext, otherStart);
        if (sdPartner >= sdOtherStart) {
            return otherStart.clone();
        }
        double sdOtherEnd = distanceSquared(ext, otherEnd);
        if (sdPartner >= sdOtherEnd) {
            return otherEnd.clone();
        }
        /*
         * Segments are disjoint.
         */
        return null;
    }

    /**
     * Helper method to determine where (if at all) 2 parallel segments
     * intersect.
     *
     * @param start1 coordinates of start of the first segment (not null)
     * @param end1 coordinates of end of the first segment (not null)
     * @param start2 coordinates of start of the 2nd segment (not null)
     * @param end2 coordinates of end of the 2nd segment (not null)
     * @param tolerance2 for coincidence (in squared units, &ge;0)
     * @return a new coordinate vector, or null if no overlap found
     */
    private static Vector3f intersectParallelSegments(Vector3f start1,
            Vector3f end1, Vector3f start2, Vector3f end2, float tolerance2) {
        assert start1 != null;
        assert end1 != null;
        assert start2 != null;
        assert end2 != null;
        /*
         * Find the 2 most remote locations.
         */
        List<Vector3f> locations = new ArrayList<>(4);
        locations.add(start1);
        locations.add(end1);
        locations.add(start2);
        locations.add(end2);
        Vector3f[] md = mostRemote(locations);
        Vector3f first = md[0];
        Vector3f last = md[1];
        if (doCoincide(first, last, tolerance2)) {
            return start2.clone();
        }
        /*
         * Test the middle 2 locations.
         */
        boolean success = locations.remove(first);
        assert success;
        success = locations.remove(last);
        assert success;
        boolean collin = allCollinear(first, last, locations, tolerance2);
        if (!collin) {
            /*
             * The 2 segments are parallel but not collinear, so they
             * do not intersect.
             */
            return null;
        }
        /*
         * All 4 locations are collinear.
         */
        Vector3f result
                = intersectCollinearSegments(first, start2, end2, start1, end1);

        return result;
    }
}
