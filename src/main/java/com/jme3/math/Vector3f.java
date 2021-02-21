/*
 * Copyright (c) 2009-2021 jMonkeyEngine
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
package com.jme3.math;

import java.util.logging.Logger;

/*
 * -- Added *Local methods to cut down on object creation - JS
 */

/**
 * <code>Vector3f</code> defines a Vector for a three float value tuple.
 * <code>Vector3f</code> can represent any three dimensional value, such as a
 * vertex, a normal, etc. Utility methods are also included to aid in
 * mathematical calculations.
 *
 * @author Mark Powell
 * @author Joshua Slack
 */
public final class Vector3f implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;
    private static final Logger logger = Logger.getLogger(Vector3f.class.getName());
    /**
     * shared instance of the all-zero vector (0,0,0) - Do not modify!
     */
    public final static Vector3f ZERO = new Vector3f(0, 0, 0);
    /**
     * shared instance of the +X direction (1,0,0) - Do not modify!
     */
    public final static Vector3f UNIT_X = new Vector3f(1, 0, 0);
    /**
     * shared instance of the +Y direction (0,1,0) - Do not modify!
     */
    public final static Vector3f UNIT_Y = new Vector3f(0, 1, 0);
    /**
     * shared instance of the +Z direction (0,0,1) - Do not modify!
     */
    public final static Vector3f UNIT_Z = new Vector3f(0, 0, 1);
    /**
     * shared instance of the all-ones vector (1,1,1) - Do not modify!
     */
    public final static Vector3f UNIT_XYZ = new Vector3f(1, 1, 1);
    /**
     * the x value of the vector.
     */
    public float x;
    /**
     * the y value of the vector.
     */
    public float y;
    /**
     * the z value of the vector.
     */
    public float z;

    /**
     * Constructor instantiates a new <code>Vector3f</code> with default
     * values of (0,0,0).
     *
     */
    public Vector3f() {
        x = y = z = 0;
    }

    /**
     * Constructor instantiates a new <code>Vector3f</code> with provides
     * values.
     *
     * @param x   the x value of the vector.
     * @param y   the y value of the vector.
     * @param z   the z value of the vector.
     */
    public Vector3f(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * <code>set</code> sets the x,y,z values of the vector based on passed
     * parameters.
     *
     * @param x   the x value of the vector.
     * @param y   the y value of the vector.
     * @param z   the z value of the vector.
     * @return this vector
     */
    public Vector3f set(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    /**
     * <code>set</code> sets the x,y,z values of the vector by copying the
     * supplied vector.
     *
     * @param vect
     *            the vector to copy.
     * @return this vector
     */
    public Vector3f set(Vector3f vect) {
        this.x = vect.x;
        this.y = vect.y;
        this.z = vect.z;
        return this;
    }

    /**
     * <code>add</code> adds a provided vector to this vector creating a
     * resultant vector which is returned. If the provided vector is null, null
     * is returned.
     *
     * @param vec
     *            the vector to add to this.
     * @return the resultant vector.
     */
    public Vector3f add(Vector3f vec) {
        if (null == vec) {
            logger.warning("Provided vector is null, null returned.");
            return null;
        }
        return new Vector3f(x + vec.x, y + vec.y, z + vec.z);
    }

    /**
     * <code>addLocal</code> adds a provided vector to this vector internally,
     * and returns a handle to this vector for easy chaining of calls. If the
     * provided vector is null, null is returned.
     *
     * @param vec
     *            the vector to add to this vector.
     * @return this
     */
    public Vector3f addLocal(Vector3f vec) {
        if (null == vec) {
            logger.warning("Provided vector is null, null returned.");
            return null;
        }
        x += vec.x;
        y += vec.y;
        z += vec.z;
        return this;
    }

    /**
     * <code>addLocal</code> adds the provided values to this vector
     * internally, and returns a handle to this vector for easy chaining of
     * calls.
     *
     * @param addX
     *            value to add to x
     * @param addY
     *            value to add to y
     * @param addZ
     *            value to add to z
     * @return this
     */
    public Vector3f addLocal(float addX, float addY, float addZ) {
        x += addX;
        y += addY;
        z += addZ;
        return this;
    }

    /**
     * <code>dot</code> calculates the dot product of this vector with a
     * provided vector. If the provided vector is null, 0 is returned.
     *
     * @param vec
     *            the vector to dot with this vector.
     * @return the resultant dot product of this vector and a given vector.
     */
    public float dot(Vector3f vec) {
        if (null == vec) {
            logger.warning("Provided vector is null, 0 returned.");
            return 0;
        }
        return x * vec.x + y * vec.y + z * vec.z;
    }

    /**
     * <code>cross</code> calculates the cross product of this vector with a
     * parameter vector v.
     *
     * @param v   the vector to take the cross product of with this.
     * @return the cross product vector.
     */
    public Vector3f cross(Vector3f v) {
        return cross(v, null);
    }

    /**
     * <code>cross</code> calculates the cross product of this vector with a
     * parameter vector v.  The result is stored in <code>result</code>
     *
     * @param v   the vector to take the cross product of with this.
     * @param result
     *            the vector to store the cross product result.
     * @return result, after receiving the cross product vector.
     */
    public Vector3f cross(Vector3f v, Vector3f result) {
        return cross(v.x, v.y, v.z, result);
    }

    /**
     * <code>cross</code> calculates the cross product of this vector with a
     * parameter vector v.  The result is stored in <code>result</code>
     *
     * @param otherX
     *            x component of the vector to take the cross product of with this.
     * @param otherY
     *            y component of the vector to take the cross product of with this.
     * @param otherZ
     *            z component of the vector to take the cross product of with this.
     * @param result
     *            the vector to store the cross product result.
     * @return result, after receiving the cross product vector.
     */
    public Vector3f cross(float otherX, float otherY, float otherZ, Vector3f result) {
        if (result == null) {
            result = new Vector3f();
        }
        float resX = ((y * otherZ) - (z * otherY));
        float resY = ((z * otherX) - (x * otherZ));
        float resZ = ((x * otherY) - (y * otherX));
        result.set(resX, resY, resZ);
        return result;
    }

    /**
     * Returns true if this vector is a unit vector (length() ~= 1),
     * returns false otherwise.
     *
     * @return true if this vector is a unit vector (length() ~= 1),
     * or false otherwise.
     */
    public boolean isUnitVector() {
        float len = length();
        return 0.99f < len && len < 1.01f;
    }

    /**
     * <code>length</code> calculates the magnitude of this vector.
     *
     * @return the length or magnitude of the vector.
     */
    public float length() {
        /*
         * Use double-precision arithmetic to reduce the chance of overflow
         * (when lengthSquared > Float.MAX_VALUE) or underflow (when
         * lengthSquared is < Float.MIN_VALUE).
         */
        double xx = x;
        double yy = y;
        double zz = z;
        double lengthSquared = xx * xx + yy * yy + zz * zz;
        float result = (float) Math.sqrt(lengthSquared);

        return result;
    }

    /**
     * <code>lengthSquared</code> calculates the squared value of the
     * magnitude of the vector.
     *
     * @return the magnitude squared of the vector.
     */
    public float lengthSquared() {
        return x * x + y * y + z * z;
    }

    /**
     * <code>multLocal</code> multiplies this vector by a scalar internally,
     * and returns a handle to this vector for easy chaining of calls.
     *
     * @param scalar
     *            the value to multiply this vector by.
     * @return this
     */
    public Vector3f multLocal(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return this;
    }

    /**
     * <code>multLocal</code> multiplies a provided vector to this vector
     * internally, and returns a handle to this vector for easy chaining of
     * calls. If the provided vector is null, null is returned.
     *
     * @param vec
     *            the vector to mult to this vector.
     * @return this
     */
    public Vector3f multLocal(Vector3f vec) {
        if (null == vec) {
            logger.warning("Provided vector is null, null returned.");
            return null;
        }
        x *= vec.x;
        y *= vec.y;
        z *= vec.z;
        return this;
    }

    /**
     * <code>multLocal</code> multiplies a provided vector to this vector
     * internally, and returns a handle to this vector for easy chaining of
     * calls. If the provided vector is null, null is returned.
     *
     * @param vec
     *            the vector to mult to this vector.
     * @param store result vector (null to create a new vector)
     * @return this
     */
    public Vector3f mult(Vector3f vec, Vector3f store) {
        if (null == vec) {
            logger.warning("Provided vector is null, null returned.");
            return null;
        }
        if (store == null) {
            store = new Vector3f();
        }
        return store.set(x * vec.x, y * vec.y, z * vec.z);
    }

    /**
     * <code>divideLocal</code> divides this vector by a scalar internally,
     * and returns a handle to this vector for easy chaining of calls. Dividing
     * by zero will result in an exception.
     *
     * @param scalar
     *            the value to divides this vector by.
     * @return this
     */
    public Vector3f divideLocal(float scalar) {
        scalar = 1f / scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return this;
    }


    /**
     * <code>divideLocal</code> divides this vector by a scalar internally,
     * and returns a handle to this vector for easy chaining of calls. Dividing
     * by zero will result in an exception.
     *
     * @param scalar
     *            the value to divides this vector by.
     * @return this
     */
    public Vector3f divideLocal(Vector3f scalar) {
        x /= scalar.x;
        y /= scalar.y;
        z /= scalar.z;
        return this;
    }

    /**
     * <code>subtract</code> subtracts the values of a given vector from those
     * of this vector creating a new vector object. If the provided vector is
     * null, null is returned.
     *
     * @param vec
     *            the vector to subtract from this vector.
     * @return the result vector.
     */
    public Vector3f subtract(Vector3f vec) {
        return new Vector3f(x - vec.x, y - vec.y, z - vec.z);
    }

    /**
     * <code>subtract</code>
     *
     * @param vec
     *            the vector to subtract from this
     * @param result
     *            the vector to store the result in
     * @return result
     */
    public Vector3f subtract(Vector3f vec, Vector3f result) {
        if (result == null) {
            result = new Vector3f();
        }
        result.x = x - vec.x;
        result.y = y - vec.y;
        result.z = z - vec.z;
        return result;
    }

    /**
     * <code>subtractLocal</code> subtracts the provided values from this vector
     * internally, and returns a handle to this vector for easy chaining of
     * calls.
     *
     * @param subtractX
     *            the x value to subtract.
     * @param subtractY
     *            the y value to subtract.
     * @param subtractZ
     *            the z value to subtract.
     * @return this
     */
    public Vector3f subtractLocal(float subtractX, float subtractY, float subtractZ) {
        x -= subtractX;
        y -= subtractY;
        z -= subtractZ;
        return this;
    }

    /**
     * <code>normalize</code> returns the unit vector of this vector.
     *
     * @return unit vector of this vector.
     */
    public Vector3f normalize() {
//        float length = length();
//        if (length != 0) {
//            return divide(length);
//        }
//
//        return divide(1);
        float length = x * x + y * y + z * z;
        if (length != 1f && length != 0f) {
            length = 1.0f / FastMath.sqrt(length);
            return new Vector3f(x * length, y * length, z * length);
        }
        return clone();
    }

    /**
     * <code>zero</code> resets this vector's data to zero internally.
     *
     * @return this
     */
    public Vector3f zero() {
        x = y = z = 0;
        return this;
    }

    /**
     * Check a vector... if it is null or its floats are NaN or infinite,
     * return false.  Else return true.
     *
     * @param vector the vector to check
     * @return true or false as stated above.
     */
    public static boolean isValidVector(Vector3f vector) {
        if (vector == null) {
            return false;
        }
        if (Float.isNaN(vector.x)
                || Float.isNaN(vector.y)
                || Float.isNaN(vector.z)) {
            return false;
        }
        if (Float.isInfinite(vector.x)
                || Float.isInfinite(vector.y)
                || Float.isInfinite(vector.z)) {
            return false;
        }
        return true;
    }

    /**
     * Create a copy of this vector.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public Vector3f clone() {
        try {
            return (Vector3f) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new AssertionError(); // can not happen
        }
    }

    /**
     * are these two vectors the same? they are is they both have the same x,y,
     * and z values.
     *
     * @param o   the object to compare for equality
     * @return true if they are equal
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Vector3f)) {
            return false;
        }

        if (this == o) {
            return true;
        }

        Vector3f comp = (Vector3f) o;
        if (Float.compare(x, comp.x) != 0) {
            return false;
        }
        if (Float.compare(y, comp.y) != 0) {
            return false;
        }
        if (Float.compare(z, comp.z) != 0) {
            return false;
        }
        return true;
    }

    /**
     * <code>hashCode</code> returns a unique code for this vector object based
     * on its values. If two vectors are logically equivalent, they will return
     * the same hash code value.
     *
     * @return the hash code value of this vector.
     */
    @Override
    public int hashCode() {
        int hash = 37;
        hash += 37 * hash + Float.floatToIntBits(x);
        hash += 37 * hash + Float.floatToIntBits(y);
        hash += 37 * hash + Float.floatToIntBits(z);
        return hash;
    }

    /**
     * <code>toString</code> returns a string representation of this vector.
     * The format is:
     *
     * (XX.XXXX, YY.YYYY, ZZ.ZZZZ)
     *
     * @return the string representation of this vector.
     */
    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }

    /**
     * Determine the X component of this vector.
     *
     * @return x
     */
    public float getX() {
        return x;
    }

    /**
     * @param index 0, 1, or 2
     * @return x value if index == 0, y value if index == 1 or z value if index == 2
     * @throws IllegalArgumentException
     *             if index is not one of 0, 1, 2.
     */
    public float get(int index) {
        switch (index) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
        }
        throw new IllegalArgumentException("index must be either 0, 1 or 2");
    }

    /**
     * @param index
     *            which field index in this vector to set.
     * @param value
     *            to set to one of x, y or z.
     * @throws IllegalArgumentException
     *             if index is not one of 0, 1, 2.
     */
    public void set(int index, float value) {
        switch (index) {
            case 0:
                x = value;
                return;
            case 1:
                y = value;
                return;
            case 2:
                z = value;
                return;
        }
        throw new IllegalArgumentException("index must be either 0, 1 or 2");
    }
}
