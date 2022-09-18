/*
 Copyright (c) 2021-2022, Stephen Gold
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
import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * Analyze a FloatBuffer to identify all of its distinct Vector3f values.
 * Immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class DistinctVectorValues {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(DistinctVectorValues.class.getName());
    // *************************************************************************
    // fields

    /**
     * the number of distinct vector values in the buffer slice
     */
    final private int numDistinctValues;
    /**
     * number of vectors in the buffer slice
     */
    final private int numVectors;
    /**
     * map vector indices (which are typically vertex indices) to vector value
     * IDs
     */
    final private int[] vectorIndex2Vvid;
    /**
     * map Vector3f values to IDs
     */
    final private Map<Vector3f, Integer> value2Vvid;
    /**
     * temporary storage for a vector
     */
    final private Vector3f tmpVector = new Vector3f();
    // *************************************************************************
    // constructors

    /**
     * Analyze the specified FloatBuffer range using exact vector comparisons
     * (without distinguishing 0 from -0) and instantiate the results.
     *
     * @param buffer the buffer to analyze (not null, unaffected)
     * @param startPosition the buffer position at which the vectors start
     * (&ge;0, &le;endPosition, typically 0)
     * @param endPosition the buffer position at which the vectors end
     * (&ge;startPosition, &le;capacity)
     */
    public DistinctVectorValues(
            FloatBuffer buffer, int startPosition, int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0, endPosition);
        Validate.inRange(
                endPosition, "end position", startPosition, buffer.capacity());
        int numFloats = endPosition - startPosition;
        Validate.require(
                numFloats % MyVector3f.numAxes == 0, "whole number of vectors");

        this.numVectors = numFloats / MyVector3f.numAxes;
        this.value2Vvid = new HashMap<>(numVectors);

        this.vectorIndex2Vvid = new int[numVectors];
        Arrays.fill(vectorIndex2Vvid, -1);

        // Assign an ID to each distinct vector value in the buffer range.
        int nextVvid = 0;
        for (int vectorIndex = 0; vectorIndex < numVectors; ++vectorIndex) {
            int position = startPosition + MyVector3f.numAxes * vectorIndex;
            Vector3f vector = new Vector3f();
            MyBuffer.get(buffer, position, vector);

            MyVector3f.standardize(vector, tmpVector);
            Integer vvid = value2Vvid.get(tmpVector);
            if (vvid == null) {
                vvid = nextVvid;
                ++nextVvid;
                value2Vvid.put(vector, vvid);
            }
            vectorIndex2Vvid[vectorIndex] = vvid;
        }
        this.numDistinctValues = nextVvid;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count the number of distinct vectors in the original buffer range.
     *
     * @return the count (&ge;0)
     */
    public int countDistinct() {
        assert numDistinctValues >= 0 : numDistinctValues;
        return numDistinctValues;
    }

    /**
     * Find the ID of the indexed vector in the original buffer range.
     *
     * @param vectorIndex the index of the vector in the original buffer (&ge;0)
     * which is typically a vertex ID
     * @return an ID (&ge;0) or -1 if not found
     */
    public int findVvid(int vectorIndex) {
        Validate.nonNegative(vectorIndex, "vector index");
        int result = vectorIndex2Vvid[vectorIndex];

        assert result >= -1 : result;
        assert result < numDistinctValues;
        return result;
    }
}
