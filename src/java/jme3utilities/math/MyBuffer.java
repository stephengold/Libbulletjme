/*
 Copyright (c) 2019-2020, Stephen Gold
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

import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Utility methods that operate on buffers, especially float buffers containing
 * 3-D vectors. Unlike the com.jme3.util.BufferUtils methods, these methods
 * ignore buffer limits.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MyBuffer {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(MyBuffer.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyBuffer() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Enumerate all distinct 3-D vectors in the specified FloatBuffer range,
     * distinguishing 0 from -0.
     *
     * @param buffer the buffer that contains the vectors (not null, unaffected)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @return a new VectorSet (not null)
     */
    public static VectorSet distinct(FloatBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        VectorSet result;
        int numVectors = numFloats / numAxes;
        if (numVectors > 20) {
            result = new VectorSetUsingCollection(numVectors);
        } else {
            boolean direct = false;
            result = new VectorSetUsingBuffer(numVectors, direct);
        }

        Vector3f tmpVector = new Vector3f();
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;
            get(buffer, position, tmpVector);
            result.add(tmpVector);
        }

        return result;
    }

    /**
     * Calculate the sample covariance of 3-D vectors in the specified
     * FloatBuffer range.
     *
     * @param buffer the buffer that contains the vectors (not null, unaffected)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition-6)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition+6, &le;capacity)
     * @param storeResult storage for the result (modified if not null)
     * @return the unbiased sample covariance (either storeResult or a new
     * matrix, not null)
     */
    public static Matrix3f covariance(FloatBuffer buffer, int startPosition,
            int endPosition, Matrix3f storeResult) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0,
                endPosition - 2 * numAxes);
        Validate.inRange(endPosition, "end position",
                startPosition + 2 * numAxes, buffer.capacity());
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        int numVectors = numFloats / numAxes;
        Vector3f sampleMean = mean(buffer, startPosition, endPosition, null);
        /*
         * Accumulate sums in the upper triangle of the matrix.
         */
        result.zero();
        float[] aboveMean = new float[numAxes];
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;

            float x = buffer.get(position + MyVector3f.xAxis);
            float y = buffer.get(position + MyVector3f.yAxis);
            float z = buffer.get(position + MyVector3f.zAxis);
            aboveMean[0] = x - sampleMean.x;
            aboveMean[1] = y - sampleMean.y;
            aboveMean[2] = z - sampleMean.z;
            for (int rowIndex = 0; rowIndex < numAxes; ++rowIndex) {
                for (int colIndex = rowIndex; colIndex < numAxes; ++colIndex) {
                    float sum = result.get(rowIndex, colIndex);
                    sum += aboveMean[rowIndex] * aboveMean[colIndex];
                    result.set(rowIndex, colIndex, sum);
                }
            }
        }
        /*
         * Multiply sums by 1/(N-1) and fill in the lower triangle.
         */
        float nMinus1 = numVectors - 1;
        for (int rowIndex = 0; rowIndex < numAxes; ++rowIndex) {
            for (int colIndex = rowIndex; colIndex < numAxes; ++colIndex) {
                float sum = result.get(rowIndex, colIndex);
                float element = sum / nMinus1;
                result.set(rowIndex, colIndex, element);
                result.set(colIndex, rowIndex, element);
            }
        }

        return result;
    }

    /**
     * Find the the radius of a bounding cylinder for the specified FloatBuffer
     * range.
     *
     * @param buffer the buffer that contains the vectors (not null, unaffected)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @param axisIndex the cylinder's height axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return the radius of the minimum bounding cylinder centered at the
     * origin (&ge;0)
     */
    public static float cylinderRadius(FloatBuffer buffer, int startPosition,
            int endPosition, int axisIndex) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        Validate.inRange(axisIndex, "axis index", MyVector3f.xAxis,
                MyVector3f.zAxis);
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        double maxRadiusSquared = 0.0;
        int numVectors = numFloats / numAxes;

        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;

            float x = buffer.get(position + MyVector3f.xAxis);
            float y = buffer.get(position + MyVector3f.yAxis);
            float z = buffer.get(position + MyVector3f.zAxis);
            float radiusSquared;
            switch (axisIndex) {
                case MyVector3f.xAxis:
                    radiusSquared = y * y + z * z;
                    break;
                case MyVector3f.yAxis:
                    radiusSquared = x * x + z * z;
                    break;
                case MyVector3f.zAxis:
                    radiusSquared = x * x + y * y;
                    break;
                default:
                    String message = Integer.toString(axisIndex);
                    throw new RuntimeException(message);
            }

            if (radiusSquared > maxRadiusSquared) {
                maxRadiusSquared = radiusSquared;
            }
        }

        float result = (float) Math.sqrt(maxRadiusSquared);
        assert result >= 0f : result;
        return result;
    }

    /**
     * Reuse the specified FloatBuffer, if it has the required capacity. If no
     * buffer is specified, allocate a new one.
     *
     * @param minFloats the required capacity (in floats, &ge;0)
     * @param bufferToReuse the buffer to reuse, or null for none
     * @return a buffer with at least the required capacity (either storeResult
     * or a new direct buffer)
     */
    public static FloatBuffer ensureCapacity(int minFloats,
            FloatBuffer bufferToReuse) {
        Validate.nonNegative(minFloats, "minimum number of elements");

        FloatBuffer result;
        if (bufferToReuse == null) {
            result = BufferUtils.createFloatBuffer(minFloats);

        } else {
            int capacityFloats = bufferToReuse.capacity();
            if (capacityFloats < minFloats) {
                logger.log(Level.SEVERE, "capacity={0}", capacityFloats);
                String message = String.format(
                        "Buffer capacity must be greater than or equal to %d.",
                        minFloats);
                throw new IllegalArgumentException(message);
            }
            result = bufferToReuse;
        }

        return result;
    }

    /**
     * Count the number of times the specified value occurs in the specified
     * IntBuffer range.
     *
     * @param buffer the buffer that contains the data (not null, unaffected)
     * @param startPosition the position at which the data start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the data end (&ge;startPosition,
     * &le;capacity)
     * @param intValue the value to search for
     * @return the number of occurrences found (&ge;0)
     */
    public static int frequency(IntBuffer buffer, int startPosition,
            int endPosition, int intValue) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());

        int result = 0;
        for (int position = startPosition; position < endPosition; ++position) {
            int bufferValue = buffer.get(position);
            if (bufferValue == intValue) {
                ++result;
            }
        }

        return result;
    }

    /**
     * Read a Vector3f starting from the specified position. Does not alter the
     * buffer's position.
     *
     * @param buffer the buffer to read from (not null, unaffected)
     * @param startPosition the position at which to start reading (&ge;0)
     * @param storeVector storage for the vector (not null, modified)
     *
     * @see com.jme3.util.BufferUtils#populateFromBuffer(com.jme3.math.Vector3f,
     * java.nio.FloatBuffer, int)
     */
    public static void get(FloatBuffer buffer, int startPosition,
            Vector3f storeVector) {
        Validate.nonNull(buffer, "buffer");
        Validate.nonNegative(startPosition, "start position");
        Validate.nonNull(storeVector, "store vector");

        storeVector.x = buffer.get(startPosition + MyVector3f.xAxis);
        storeVector.y = buffer.get(startPosition + MyVector3f.yAxis);
        storeVector.z = buffer.get(startPosition + MyVector3f.zAxis);
    }

    /**
     * Find the maximum absolute coordinate for each axis in the specified
     * FloatBuffer range.
     *
     * @param buffer the buffer that contains the vectors (not null, unaffected)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @param storeResult storage for the result (modified if not null)
     * @return the half extent for each axis (either storeResult or a new
     * instance)
     */
    public static Vector3f maxAbs(FloatBuffer buffer, int startPosition,
            int endPosition, Vector3f storeResult) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        int numVectors = numFloats / numAxes;
        result.zero();
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;
            float x = buffer.get(position + MyVector3f.xAxis);
            float y = buffer.get(position + MyVector3f.yAxis);
            float z = buffer.get(position + MyVector3f.zAxis);
            result.x = Math.max(result.x, Math.abs(x));
            result.y = Math.max(result.y, Math.abs(y));
            result.z = Math.max(result.z, Math.abs(z));
        }

        return result;
    }

    /**
     * Find the magnitude of the longest 3-D vector in the specified FloatBuffer
     * range.
     *
     * @param buffer the buffer that contains the vectors (not null, unaffected)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @return the radius of the minimum bounding sphere centered at the origin
     * (&ge;0)
     */
    public static float maxLength(FloatBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        double maxLengthSquared = 0.0;
        int numVectors = numFloats / numAxes;
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;

            float x = buffer.get(position + MyVector3f.xAxis);
            float y = buffer.get(position + MyVector3f.yAxis);
            float z = buffer.get(position + MyVector3f.zAxis);

            double lengthSquared = MyMath.sumOfSquares(x, y, z);
            if (lengthSquared > maxLengthSquared) {
                maxLengthSquared = lengthSquared;
            }
        }

        float result = (float) Math.sqrt(maxLengthSquared);
        assert result >= 0f : result;
        return result;
    }

    /**
     * Find the maximum and minimum coordinates of 3-D vectors in the specified
     * FloatBuffer range. An empty range stores infinities.
     *
     * @see com.jme3.bounding.BoundingBox#containAABB(java.nio.FloatBuffer)
     *
     * @param buffer the buffer that contains the vectors (not null, unaffected)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @param storeMaxima storage for maxima (not null, modified)
     * @param storeMinima storage for minima (not null, modified)
     */
    public static void maxMin(FloatBuffer buffer, int startPosition,
            int endPosition, Vector3f storeMaxima, Vector3f storeMinima) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        storeMaxima.set(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY,
                Float.NEGATIVE_INFINITY);
        storeMinima.set(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY,
                Float.POSITIVE_INFINITY);
        Vector3f tmpVector = new Vector3f();
        int numVectors = numFloats / numAxes;
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;
            get(buffer, position, tmpVector);
            MyVector3f.accumulateMinima(storeMinima, tmpVector);
            MyVector3f.accumulateMaxima(storeMaxima, tmpVector);
        }
    }

    /**
     * Calculate the arithmetic mean of 3-D vectors in the specified FloatBuffer
     * range.
     *
     * @param buffer the buffer that contains the vectors (not null, unaffected)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition-3)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition+3, &le;capacity)
     * @param storeResult storage for the result (modified if not null)
     * @return the mean (either storeResult or a new vector, not null)
     */
    public static Vector3f mean(FloatBuffer buffer, int startPosition,
            int endPosition, Vector3f storeResult) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0,
                endPosition - numAxes);
        Validate.inRange(endPosition, "end position", startPosition + numAxes,
                buffer.capacity());
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        int numVectors = numFloats / numAxes;
        result.zero();
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;

            float x = buffer.get(position + MyVector3f.xAxis);
            float y = buffer.get(position + MyVector3f.yAxis);
            float z = buffer.get(position + MyVector3f.zAxis);

            result.addLocal(x, y, z);
        }
        result.divideLocal(numVectors);

        return result;
    }

    /**
     * Normalize 3-D vectors in the specified FloatBuffer range.
     *
     * @param buffer the buffer that contains the vectors (not null, modified)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition-3)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition+3, &le;capacity)
     */
    public static void normalize(FloatBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0,
                endPosition - numAxes);
        Validate.inRange(endPosition, "end position", startPosition + numAxes,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        int numVectors = numFloats / numAxes;
        Vector3f tmpVector = new Vector3f();
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;
            get(buffer, position, tmpVector);
            MyVector3f.normalizeLocal(tmpVector);
            put(buffer, position, tmpVector);
        }
    }

    /**
     * Write a Vector3f starting at the specified position. Does not alter the
     * buffer's position.
     *
     * @param buffer the buffer to write to (not null, modified)
     * @param startPosition the position at which to start writing (&ge;0)
     * @param vector the input vector (not null, unaffected)
     *
     * @see com.jme3.util.BufferUtils#setInBuffer(com.jme3.math.Vector3f,
     * java.nio.FloatBuffer, int)
     */
    public static void put(FloatBuffer buffer, int startPosition,
            Vector3f vector) {
        Validate.nonNull(buffer, "buffer");
        Validate.nonNegative(startPosition, "start position");
        Validate.nonNull(vector, "vector");

        buffer.put(startPosition + MyVector3f.xAxis, vector.x);
        buffer.put(startPosition + MyVector3f.yAxis, vector.y);
        buffer.put(startPosition + MyVector3f.zAxis, vector.z);
    }

    /**
     * Apply the specified rotation to 3-D vectors in the specified FloatBuffer
     * range.
     *
     * @param buffer the buffer that contains the vectors (not null, MODIFIED)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @param rotation the rotation to apply (not null, unaffected)
     */
    public static void rotate(FloatBuffer buffer, int startPosition,
            int endPosition, Quaternion rotation) {
        Validate.nonNull(buffer, "buffer");
        Validate.nonNull(rotation, "rotation");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        int numVectors = numFloats / numAxes;
        Vector3f tmpVector = new Vector3f();
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;
            get(buffer, position, tmpVector);
            rotation.mult(tmpVector, tmpVector);
            put(buffer, position, tmpVector);
        }
    }

    /**
     * Apply the specified scale factors to 3-D vectors in the specified
     * FloatBuffer range.
     *
     * @param buffer the buffer that contains the vectors (not null, MODIFIED)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @param scale the scale factor to apply to each axis (not null,
     * unaffected)
     */
    public static void scale(FloatBuffer buffer, int startPosition,
            int endPosition, Vector3f scale) {
        Validate.nonNull(buffer, "buffer");
        Validate.finite(scale, "scale factors");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        Vector3f tmpVector = new Vector3f();
        int numVectors = numFloats / numAxes;
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;
            get(buffer, position, tmpVector);
            scale.mult(tmpVector, tmpVector);
            put(buffer, position, tmpVector);
        }
    }

    /**
     * Copy data in the specified FloatBuffer range to a new array.
     *
     * @param buffer the buffer that contains the data (not null, unaffected)
     * @param startPosition the position at which the data start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the data end (&ge;startPosition,
     * &le;capacity)
     * @return a new array (not null)
     */
    public static float[] toFloatArray(FloatBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());

        int numFloats = endPosition - startPosition;
        float[] result = new float[numFloats];

        for (int floatIndex = 0; floatIndex < numFloats; ++floatIndex) {
            int position = startPosition + floatIndex;
            result[floatIndex] = buffer.get(position);
        }

        return result;
    }

    /**
     * Copy data in the specified IntBuffer range to a new array.
     *
     * @param buffer the buffer that contains the data (not null, unaffected)
     * @param startPosition the position at which the data start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the data end (&ge;startPosition,
     * &le;capacity)
     * @return a new array (not null)
     */
    public static int[] toIntArray(IntBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());

        int numInts = endPosition - startPosition;
        int[] result = new int[numInts];

        for (int intIndex = 0; intIndex < numInts; ++intIndex) {
            int position = startPosition + intIndex;
            result[intIndex] = buffer.get(position);
        }

        return result;
    }

    /**
     * Apply the specified coordinate transform to 3-D vectors in the specified
     * FloatBuffer range.
     *
     * @param buffer the buffer that contains the vectors (not null, MODIFIED)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @param transform the transform to apply (not null, unaffected)
     */
    public static void transform(FloatBuffer buffer, int startPosition,
            int endPosition, Transform transform) {
        Validate.nonNull(buffer, "buffer");
        Validate.nonNull(transform, "transform");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        Vector3f tmpVector = new Vector3f();
        int numVectors = numFloats / numAxes;
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;
            get(buffer, position, tmpVector);
            transform.transformVector(tmpVector, tmpVector);
            put(buffer, position, tmpVector);
        }
    }

    /**
     * Add the specified offset to 3-D vectors in the specified FloatBuffer
     * range.
     *
     * @param buffer the buffer that contains the vectors (not null, MODIFIED)
     * @param startPosition the position at which the vectors start (&ge;0,
     * &le;endPosition)
     * @param endPosition the position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     * @param offsetVector the vector to add (not null, unaffected)
     */
    public static void translate(FloatBuffer buffer, int startPosition,
            int endPosition, Vector3f offsetVector) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(endPosition, "end position", startPosition,
                buffer.capacity());
        Validate.finite(offsetVector, "offset vector");
        int numFloats = endPosition - startPosition;
        assert (numFloats % numAxes == 0) : numFloats;

        int numVectors = numFloats / numAxes;
        Vector3f tmpVector = new Vector3f();
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + vectorIndex * numAxes;
            get(buffer, position, tmpVector);
            tmpVector.addLocal(offsetVector);
            put(buffer, position, tmpVector);
        }
    }
}
