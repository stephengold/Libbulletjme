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
package jme3utilities.math.noise;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.util.Random;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyMath;
import jme3utilities.math.MyQuaternion;
import jme3utilities.math.MyVector3f;

/**
 * Generate pseudo-random numbers, vectors, and selections.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Generator extends Random {
    // *************************************************************************
    // fields

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(Generator.class.getName());
    /**
     * version number for serialization
     */
    final private static long serialVersionUID = 37_705_297_950_129_619L;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a pseudo-random generator with a seed likely to be unique.
     */
    public Generator() {
    }

    /**
     * Instantiate a pseudo-random generator with the specified seed.
     *
     * @param seed initial value for the seed
     */
    public Generator(long seed) {
        super(seed);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a single-precision value, uniformly distributed between 2
     * extremes.
     *
     * @param e1 the first extreme
     * @param e2 the 2nd extreme
     * @return a pseudo-random value (&ge;min(e1,e2), &le;max(e1,e2))
     */
    public float nextFloat(float e1, float e2) {
        float result = e1 + nextFloat() * (e2 - e1);

        assert MyMath.isBetween(e1, result, e2);
        return result;
    }

    /**
     * Generate an integer value, uniformly distributed between 2 extremes.
     *
     * @param e1 the first extreme
     * @param e2 the 2nd extreme
     * @return a pseudo-random value (&ge;min(e1,e2), &le;max(e1,e2))
     */
    public int nextInt(int e1, int e2) {
        int max = Math.max(e1, e2);
        int min = Math.min(e1, e2);
        int result = min + nextInt(max - min + 1);

        assert MyMath.isBetween(e1, result, e2);
        return result;
    }

    /**
     * Generate a uniformly distributed, pseudo-random unit quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit quaternion (either storeResult or a new instance)
     */
    public Quaternion nextQuaternion(Quaternion storeResult) {
        Quaternion result = (storeResult == null) ? new Quaternion()
                : storeResult;

        double lengthSquared = 0.0;
        while (lengthSquared < 0.1 || lengthSquared > 1.0) {
            float x = nextFloat(-1f, 1f);
            float y = nextFloat(-1f, 1f);
            float z = nextFloat(-1f, 1f);
            float w = nextFloat(-1f, 1f);
            result.set(x, y, z, w);
            lengthSquared = MyQuaternion.lengthSquared(result);
        }
        double scaleFactor = 1.0 / Math.sqrt(lengthSquared);
        result.multLocal((float) scaleFactor);

        return result;
    }

    /**
     * Generate a uniformly distributed, pseudo-random unit vector.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit vector (either storeResult or a new instance)
     */
    public Vector3f nextUnitVector3f(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        double lengthSquared = 0.0;
        while (lengthSquared < 0.1 || lengthSquared > 1.0) {
            float x = nextFloat(-1f, 1f);
            float y = nextFloat(-1f, 1f);
            float z = nextFloat(-1f, 1f);
            result.set(x, y, z);
            lengthSquared = MyVector3f.lengthSquared(result);
        }
        double scaleFactor = 1.0 / Math.sqrt(lengthSquared);
        result.multLocal((float) scaleFactor);

        assert result.isUnitVector();
        return result;
    }

    /**
     * Generate a pseudo-random vector that is uniformly distributed throughout
     * the unit sphere centered on the origin.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a vector with length&le;1 (either storeResult or a new instance)
     */
    public Vector3f nextVector3f(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        double lengthSquared = 2.0;
        while (lengthSquared > 1.0) {
            float x = nextFloat(-1f, 1f);
            float y = nextFloat(-1f, 1f);
            float z = nextFloat(-1f, 1f);
            result.set(x, y, z);
            lengthSquared = MyVector3f.lengthSquared(result);
        }

        return result;
    }

    /**
     * Pick a pseudo-random element from the specified array.
     *
     * @param <E> the type of list elements
     * @param array the array to select from (not null, may be empty)
     * @return a pre-existing element of array, or null if it's empty
     */
    public <E> E pick(E[] array) {
        Validate.nonNull(array, "array");

        int count = array.length;
        if (count == 0) {
            return null;
        }
        assert count > 0 : count;
        int index = nextInt(count);
        E result = array[index];

        return result;
    }
}
