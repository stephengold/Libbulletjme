/*
 Copyright (c) 2019-2023 Stephen Gold
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
import java.nio.FloatBuffer;

/**
 * A collection of Vector3f values without duplicates.
 * <p>
 * When determining duplicates, -0 is distinguished from 0.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public interface VectorSet {
    /**
     * Add the specified value to this set, if it's not already present.
     *
     * @param x the X component of the new value
     * @param y the Y component of the new value
     * @param z the Z component of the new value
     */
    void add(float x, float y, float z);

    /**
     * Add the value of the specified Vector3f to this set, if it's not already
     * present.
     *
     * @param vector the value to add (not null, unaffected)
     */
    void add(Vector3f vector);

    /**
     * Add the specified values to this set, to the extent that they're not
     * already present.
     *
     * @param vectors the values to add (not null, unaffected)
     */
    void addAll(Iterable<? extends Vector3f> vectors); // TODO Vector3f is final

    /**
     * Reset this set to its initial (empty) state without altering its
     * capacity.
     */
    void clear();

    /**
     * Test whether this set contains the specified value.
     *
     * @param x the X component of the value to find
     * @param y the Y component of the value to find
     * @param z the Z component of the value to find
     * @return true if found, otherwise false
     */
    boolean contains(float x, float y, float z);

    /**
     * Test whether this set contains the value of the specified Vector3f.
     *
     * @param vector the value to find (not null, unaffected)
     * @return true if found, otherwise false
     */
    boolean contains(Vector3f vector);

    /**
     * Determine the sample covariance of the values in this set.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unbiased sample covariance (either storeResult or a new
     * matrix, not null)
     */
    Matrix3f covariance(Matrix3f storeResult);

    /**
     * Find the maximum absolute coordinate for each axis among the Vector3f
     * values in this set.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the half extent for each axis (either storeResult or a new
     * instance)
     */
    Vector3f maxAbs(Vector3f storeResult);

    /**
     * Find the magnitude of the longest vector in this set.
     *
     * @return the magnitude (&ge;0)
     */
    float maxLength();

    /**
     * Find the maximum and minimum coordinates for each axis among the values
     * in this set.
     *
     * @param storeMaxima storage for the maxima (not null, modified)
     * @param storeMinima storage for the minima (not null, modified)
     */
    void maxMin(Vector3f storeMaxima, Vector3f storeMinima);

    /**
     * Determine the sample mean for each axis among the values in this set.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the sample mean for each axis (either storeResult or a new
     * Vector3f)
     */
    Vector3f mean(Vector3f storeResult);

    /**
     * Determine the number of values in this set.
     *
     * @return the count (&ge;0)
     */
    int numVectors();

    /**
     * Access a Buffer containing all values in this set. No further add() is
     * allowed.
     *
     * @return a Buffer, flipped but possibly not rewound
     */
    FloatBuffer toBuffer();

    /**
     * Create an array of floats containing all values in this set.
     *
     * @return a new array
     */
    float[] toFloatArray();

    /**
     * Create an array of vectors containing all values in this set.
     *
     * @return a new array of new vectors
     */
    Vector3f[] toVectorArray();
}
