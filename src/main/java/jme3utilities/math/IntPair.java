/*
 * Copyright (c) 2020, Stephen Gold
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.math;

import java.util.logging.Logger;

/**
 * Represent a pair of distinct integers. Immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class IntPair {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(IntPair.class.getName());
    // *************************************************************************
    // fields

    /**
     * larger element of the pair
     */
    final private int larger;
    /**
     * smaller element of the pair
     */
    final private int smaller;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a pair.
     *
     * @param a the first element (&ne; b)
     * @param b the 2nd element (&ne; a)
     */
    public IntPair(int a, int b) {
        assert a != b;

        if (a < b) {
            smaller = a;
            larger = b;
        } else {
            larger = a;
            smaller = b;
        }
        assert smaller < larger;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read the larger element of the pair.
     *
     * @return the element value
     */
    public int larger() {
        return larger;
    }

    /**
     * Read the smaller element of the pair.
     *
     * @return the element value
     */
    public int smaller() {
        return smaller;
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for exact equivalence with another Object.
     *
     * @param otherObject the object to compare to (may be null, unaffected)
     * @return true if the objects are equivalent, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;
        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            IntPair otherEdge = (IntPair) otherObject;
            result = (otherEdge.larger == larger)
                    && (otherEdge.smaller == smaller);
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this IntPair.
     *
     * @return the value to use for hashing
     */
    @Override
    public int hashCode() {
        int result = 707;
        result = 29 * result + smaller;
        result = 29 * result + larger;

        return result;
    }
}
