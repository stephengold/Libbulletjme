/*
 Copyright (c) 2019-2022 Stephen Gold
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
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A VectorSet implemented using FloatBuffer.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class VectorSetUsingBuffer implements VectorSet {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a Vector3f
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VectorSetUsingBuffer.class.getName());
    // *************************************************************************
    // fields

    /**
     * true&rarr;allocate direct buffers, false&rarr;allocate indirect buffers
     */
    final private boolean useDirectBuffers;
    /**
     * buffer to hold the Vector3f values
     */
    private FloatBuffer buffer;
    /**
     * look up last buffer position for hash index
     */
    private int[] endPosition;
    /**
     * number of enlargements since last clearStats()
     */
    private static int numEnlargements = 0;
    /**
     * number of reads since last clearStats()
     */
    private static int numReads = 0;
    /**
     * number of searches since last clearStats()
     */
    private static int numSearches = 0;
    /**
     * look up first buffer position plus 1 for hash index
     */
    private int[] startPositionPlus1;
    /**
     * system milliseconds as of last clearStats()
     */
    private static long resetMillis = 0L;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an empty set with the specified initial capacity.
     *
     * @param numVectors the number of vectors the set must hold without needing
     * enlargement (&gt;0)
     * @param direct true&rarr;allocate direct buffers, false&rarr;allocate
     * indirect buffers
     */
    public VectorSetUsingBuffer(int numVectors, boolean direct) {
        Validate.positive(numVectors, "number of vectors");

        useDirectBuffers = direct;
        allocate(numVectors);
        flip();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Reset the hashing statistics.
     */
    public static void clearStats() {
        numEnlargements = 0;
        numReads = 0;
        numSearches = 0;
        resetMillis = System.currentTimeMillis();
    }

    /**
     * Print the hashing statistics.
     *
     * @param tag (not null)
     */
    public static void dumpStats(String tag) {
        long msec = System.currentTimeMillis() - resetMillis;
        String msg = String.format(
                "%s %d enlargement%s, %d search%s, and %d read%s in %d msec",
                tag, numEnlargements, (numEnlargements == 1) ? "" : "s",
                numSearches, (numSearches == 1) ? "" : "es",
                numReads, (numReads == 1) ? "" : "s", msec);
        System.out.println(msg);
    }
    // *************************************************************************
    // VectorSet methods

    /**
     * Add the specified value to this set, if it's not already present.
     *
     * @param x the X component of the new value
     * @param y the Y component of the new value
     * @param z the Z component of the new value
     */
    @Override
    public void add(float x, float y, float z) {
        if (startPositionPlus1 == null) {
            throw new IllegalStateException("toBuffer() has been invoked.");
        }

        int hashCode = hash(x, y, z);
        if (!contains(x, y, z, hashCode)) {
            unflip();
            if (buffer.remaining() < numAxes) {
                enlarge();
                assert buffer.remaining() >= numAxes;
            }
            add(x, y, z, hashCode);
            flip();
        }
    }

    /**
     * Add the value of the specified Vector3f to this set, if it's not already
     * present.
     *
     * @param vector the value to add (not null, unaffected)
     */
    @Override
    public void add(Vector3f vector) {
        if (startPositionPlus1 == null) {
            throw new IllegalStateException("toBuffer() has been invoked.");
        }

        add(vector.x, vector.y, vector.z);
    }

    /**
     * Add the specified values to this set, to the extent that they're not
     * already present.
     *
     * @param vectors the values to add (not null, unaffected)
     */
    @Override
    public void addAll(Iterable<? extends Vector3f> vectors) {
        for (Vector3f vector : vectors) {
            add(vector.x, vector.y, vector.z);
        }
    }

    /**
     * Reset this set to its initial (empty, flipped, rewound) state without
     * altering its capacity. The hashing statistics are unaffected.
     */
    @Override
    public void clear() {
        int numFloats = buffer.capacity();
        if (startPositionPlus1 == null) { // user has invoked toBuffer()
            int numVectors = (numFloats - 1) / numAxes;
            allocate(numVectors);
            assert endPosition.length == numFloats;

        } else {
            // reuse the existing buffer
            for (int floatIndex = 0; floatIndex < numFloats; ++floatIndex) {
                startPositionPlus1[floatIndex] = 0;
            }
        }

        buffer.rewind();
        buffer.limit(0);
        assert isFlipped();
    }

    /**
     * Test whether this set contains the specified value.
     *
     * @param x the X component of the value to find
     * @param y the Y component of the value to find
     * @param z the Z component of the value to find
     * @return true if found, otherwise false
     */
    @Override
    public boolean contains(float x, float y, float z) {
        int hashCode = hash(x, y, z);
        boolean result = contains(x, y, z, hashCode);

        return result;
    }

    /**
     * Test whether this set contains the value of the specified Vector3f.
     *
     * @param vector the value to find (not null, unaffected)
     * @return true if found, otherwise false
     */
    @Override
    public boolean contains(Vector3f vector) {
        boolean result = contains(vector.x, vector.y, vector.z);
        return result;
    }

    /**
     * Determine the sample covariance of the values in this set.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unbiased sample covariance (either storeResult or a new
     * matrix, not null)
     */
    @Override
    public Matrix3f covariance(Matrix3f storeResult) {
        Matrix3f result
                = MyBuffer.covariance(buffer, 0, buffer.limit(), storeResult);
        return result;
    }

    /**
     * Find the maximum absolute coordinate for each axis among the Vector3f
     * values in this set.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the half extent for each axis (either storeResult or a new
     * instance)
     */
    @Override
    public Vector3f maxAbs(Vector3f storeResult) {
        Vector3f result
                = MyBuffer.maxAbs(buffer, 0, buffer.limit(), storeResult);
        return result;
    }

    /**
     * Find the magnitude of the longest vector in this set.
     *
     * @return the magnitude (&ge;0)
     */
    @Override
    public float maxLength() {
        float length = MyBuffer.maxLength(buffer, 0, buffer.limit());
        return length;
    }

    /**
     * Find the maximum and minimum coordinates for each axis among the values
     * in this set.
     *
     * @param storeMaxima storage for the maxima (not null, modified)
     * @param storeMinima storage for the minima (not null, modified)
     */
    @Override
    public void maxMin(Vector3f storeMaxima, Vector3f storeMinima) {
        Validate.nonNull(storeMaxima, "store maxima");
        Validate.nonNull(storeMinima, "store minima");

        MyBuffer.maxMin(buffer, 0, buffer.limit(), storeMaxima, storeMinima);
    }

    /**
     * Determine the sample mean for each axis among the values in this set.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the sample mean for each axis (either storeResult or a new
     * Vector3f)
     */
    @Override
    public Vector3f mean(Vector3f storeResult) {
        Vector3f result = MyBuffer.mean(buffer, 0, buffer.limit(), storeResult);
        return result;
    }

    /**
     * Determine the number of values in this set.
     *
     * @return the count (&ge;0)
     */
    @Override
    public int numVectors() {
        int limit = buffer.limit();
        int numFloats = (limit < buffer.capacity()) ? limit : buffer.position();
        assert numFloats % numAxes == 0 : numFloats;
        int numVectors = numFloats / numAxes;

        assert numVectors >= 0 : numVectors;
        return numVectors;
    }

    /**
     * Access a Buffer containing all values in this set. No further add() is
     * allowed.
     *
     * @return the pre-existing Buffer, flipped but possibly not rewound
     */
    @Override
    public FloatBuffer toBuffer() {
        startPositionPlus1 = null;
        endPosition = null;

        return buffer;
    }

    /**
     * Create an array of floats containing all values in this set.
     *
     * @return a new array
     */
    @Override
    public float[] toFloatArray() {
        int numFloats = numAxes * numVectors();
        float[] result = new float[numFloats];

        for (int floatIndex = 0; floatIndex < numFloats; ++floatIndex) {
            result[floatIndex] = buffer.get(floatIndex);
        }

        return result;
    }

    /**
     * Create an array of vectors containing all values in this set.
     *
     * @return a new array of new vectors
     */
    @Override
    public Vector3f[] toVectorArray() {
        int numVectors = numVectors();
        Vector3f[] result = new Vector3f[numVectors];

        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int startPosition = numAxes * vectorIndex;
            result[vectorIndex] = new Vector3f();
            MyBuffer.get(buffer, startPosition, result[vectorIndex]);
        }

        return result;
    }
    // *************************************************************************
    // Object methods

    /**
     * Represent this set as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        int numVectors = numVectors();
        StringBuilder builder = new StringBuilder(numVectors * 32);

        builder.append("VectorSet[ n=");
        builder.append(numVectors);
        String separator = System.lineSeparator();
        builder.append(separator);

        Vector3f tmpVector = new Vector3f();
        for (int iVector = 0; iVector < numVectors; ++iVector) {
            MyBuffer.get(buffer, numAxes * iVector, tmpVector);
            builder.append("  ");
            builder.append(tmpVector);
            builder.append(separator);
        }
        builder.append("]");
        builder.append(separator);

        return builder.toString();
    }
    // *************************************************************************
    // private methods

    /**
     * Add the specified value to this set without checking for capacity or
     * duplication. The buffer must not be in a flipped state.
     *
     * @param x the X component of the value
     * @param y the Y component of the value
     * @param z the Z component of the value
     * @param hashCode the hash code of vector
     */
    private void add(float x, float y, float z, int hashCode) {
        assert buffer.limit() == buffer.capacity();

        int position = buffer.position();
        int hashIndex = MyMath.modulo(hashCode, startPositionPlus1.length);
        int spp1 = startPositionPlus1[hashIndex];
        if (spp1 == 0) {
            startPositionPlus1[hashIndex] = position + 1;
        }
        endPosition[hashIndex] = position;

        buffer.put(x);
        buffer.put(y);
        buffer.put(z);
    }

    /**
     * Initialize an empty, unflipped set with the specified initial capacity.
     *
     * @param numVectors the initial capacity (&gt;0)
     */
    private void allocate(int numVectors) {
        assert numVectors > 0 : numVectors;

        int numFloats = numAxes * numVectors + 1;
        if (useDirectBuffers) {
            buffer = BufferUtils.createFloatBuffer(numFloats);
        } else {
            buffer = FloatBuffer.wrap(new float[numFloats]);
        }
        startPositionPlus1 = new int[numFloats]; // initialized to all 0s
        endPosition = new int[numFloats];
    }

    /**
     * Test whether the specified value is in the set.
     *
     * @param x the X component of the value to find
     * @param y the Y component of the value to find
     * @param z the Z component of the value to find
     * @param hashCode the hash code of value
     * @return true if found, otherwise false
     */
    private boolean contains(float x, float y, float z, int hashCode) {
        boolean result = false;
        int hashIndex = MyMath.modulo(hashCode, startPositionPlus1.length);
        int spp1 = startPositionPlus1[hashIndex];
        if (spp1 != 0) {
            int end = endPosition[hashIndex];
            buffer.position(spp1 - 1);
            while (buffer.position() <= end) {
                float bx = buffer.get();
                float by = buffer.get();
                float bz = buffer.get();
                if (Float.compare(bx, x) == 0
                        && Float.compare(by, y) == 0
                        && Float.compare(bz, z) == 0) {
                    result = true;
                    break;
                }
            }
            ++numSearches;
            numReads += (end - spp1 + 1) / numAxes;
        }

        return result;
    }

    /**
     * Quadruple the capacity of the buffer, which must be unflipped.
     */
    private void enlarge() {
        int numVectors = numVectors();

        FloatBuffer oldBuffer = toBuffer();
        allocate(4 * numVectors);
        oldBuffer.flip();
        while (oldBuffer.hasRemaining()) {
            float x = oldBuffer.get();
            float y = oldBuffer.get();
            float z = oldBuffer.get();
            int hashCode = hash(x, y, z);
            add(x, y, z, hashCode);
        }
        assert numVectors() == numVectors;
        ++numEnlargements;
    }

    /**
     * Switch from writing to reading.
     */
    private void flip() {
        assert !isFlipped();
        buffer.limit(buffer.position());
        assert isFlipped();
    }

    /**
     * Hash function for vectors, used to index into endPosition[] and
     * startPositionPlus1[]. Because floatToIntBit()s is used, -0 is
     * distinguished from 0.
     *
     * @param x the X component of the value
     * @param y the Y component of the value
     * @param z the Z component of the value
     * @return the hash code of the value
     */
    private static int hash(float x, float y, float z) {
        int hashCode = 37;
        hashCode += 37 * hashCode + Float.floatToIntBits(x);
        hashCode += 37 * hashCode + Float.floatToIntBits(y);
        hashCode += 37 * hashCode + Float.floatToIntBits(z);

        return hashCode;
    }

    /**
     * Test whether the buffer is flipped.
     *
     * @return true if flipped for reading (its usual state), false if unflipped
     * for writing (temporary state while adding a vector)
     */
    private boolean isFlipped() {
        boolean result = (buffer.limit() != buffer.capacity());
        return result;
    }

    /**
     * Switch from reading to writing.
     */
    private void unflip() {
        assert isFlipped();

        buffer.position(buffer.limit());
        buffer.limit(buffer.capacity());

        assert !isFlipped();
    }
}
