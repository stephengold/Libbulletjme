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

/**
 * A vector composed of 3 single-precision components, used to represent
 * locations, offsets, and directions in 3-dimensional space.
 *
 * <p>Methods with names ending in "Local" modify the current instance. They are
 * used to cut down on the creation of new instances.
 *
 * @author Mark Powell
 * @author Joshua Slack
 */
public final class Vector3f implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;
    private static final Logger logger = Logger.getLogger(Vector3f.class.getName());
    /**
     * Shared instance of the all-zero vector (0,0,0). Do not modify!
     */
    public final static Vector3f ZERO = new Vector3f(0, 0, 0);
    /**
     * Shared instance of the +X direction (1,0,0). Do not modify!
     */
    public final static Vector3f UNIT_X = new Vector3f(1, 0, 0);
    /**
     * Shared instance of the +Y direction (0,1,0). Do not modify!
     */
    public final static Vector3f UNIT_Y = new Vector3f(0, 1, 0);
    /**
     * Shared instance of the +Z direction (0,0,1). Do not modify!
     */
    public final static Vector3f UNIT_Z = new Vector3f(0, 0, 1);
    /**
     * Shared instance of the all-ones vector (1,1,1). Do not modify!
     */
    public final static Vector3f UNIT_XYZ = new Vector3f(1, 1, 1);
    /**
     * The first (X) component.
     */
    public float x;
    /**
     * The 2nd (Y) component.
     */
    public float y;
    /**
     * The 3rd (Z) component.
     */
    public float z;

    /**
     * Instantiates an all-zero vector (0,0,0).
     */
    public Vector3f() {
        x = y = z = 0;
    }

    /**
     * Instantiates a vector with specified components.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     */
    public Vector3f(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Instantiates a copy of the argument.
     *
     * @param copy the vector to copy (not null, unaffected)
     */
    public Vector3f(Vector3f copy) {
        this.set(copy);
    }

    /**
     * Sets all 3 components to specified values.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f set(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    /**
     * Copies all 3 components from the argument.
     *
     * @param vect the Vector3f to copy (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f set(Vector3f vect) {
        this.x = vect.x;
        this.y = vect.y;
        this.z = vect.z;
        return this;
    }

    /**
     * Adds the argument and returns the sum as a new instance. If the argument
     * is null, null is returned. Either way, the current instance is
     * unaffected.
     *
     * @param vec the vector to add (unaffected) or null for none
     * @return a new Vector3f or null
     */
    public Vector3f add(Vector3f vec) {
        if (null == vec) {
            logger.warning("Provided vector is null, null returned.");
            return null;
        }
        return new Vector3f(x + vec.x, y + vec.y, z + vec.z);
    }

    /**
     * Adds a specified vector and returns the sum in a 3rd vector. The current
     * instance is unaffected unless it's <code>result</code>.
     *
     * @param vec the vector to add (not null, unaffected unless it's
     *     <code>result</code>)
     * @param result storage for the sum (not null)
     * @return <code>result</code> (for chaining)
     */
    public Vector3f add(Vector3f vec, Vector3f result) {
        result.x = x + vec.x;
        result.y = y + vec.y;
        result.z = z + vec.z;
        return result;
    }

    /**
     * Adds the argument and returns the (modified) current instance. If the
     * argument is null, null is returned.
     *
     * @param vec the vector to add (unaffected unless it's <code>this</code>)
     *     or null for none
     * @return the (modified) current instance or null
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
     * Adds specified amounts to the vector's components and returns the sum as
     * a new instance. The current instance is unaffected.
     *
     * @param addX the amount to add to the X component
     * @param addY the amount to add to the Y component
     * @param addZ the amount to add to the Z component
     * @return a new Vector3f
     */
    public Vector3f add(float addX, float addY, float addZ) {
        return new Vector3f(x + addX, y + addY, z + addZ);
    }

    /**
     * Adds specified amounts to the vector's components and returns the
     * (modified) current instance.
     *
     * @param addX the amount to add to the X component
     * @param addY the amount to add to the Y component
     * @param addZ the amount to add to the Z component
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f addLocal(float addX, float addY, float addZ) {
        x += addX;
        y += addY;
        z += addZ;
        return this;
    }

    /**
     * Returns the dot (or inner) product with the argument. If the argument is
     * null, 0 is returned. Either way, the current instance is unaffected.
     *
     * @param vec the vector to multiply (unaffected) or null for none
     * @return the product or 0
     */
    public float dot(Vector3f vec) {
        if (null == vec) {
            logger.warning("Provided vector is null, 0 returned.");
            return 0;
        }
        return x * vec.x + y * vec.y + z * vec.z;
    }

    /**
     * Calculates a cross product with the argument and returns the product as a
     * new instance. The current instance is unaffected.
     *
     * @param v the right factor (not null, unaffected)
     * @return <code>this</code> cross <code>v</code> (a new Vector3f)
     */
    public Vector3f cross(Vector3f v) {
        return cross(v, null);
    }

    /**
     * Calculates a cross product with a specified vector and returns the
     * product in a 3rd vector. The current instance is unaffected unless it's
     * <code>result</code>.
     *
     * @param v the right factor (not null, unaffected unless it's
     *     <code>result</code>)
     * @param result storage for the product, or null for a new Vector3f
     * @return <code>this</code> cross <code>v</code> (either
     *     <code>result</code> or a new Vector3f)
     */
    public Vector3f cross(Vector3f v, Vector3f result) {
        return cross(v.x, v.y, v.z, result);
    }

    /**
     * Calculates a cross product with specified components and returns the
     * product in the specified vector. The current instance is unaffected
     * unless it's <code>result</code>.
     *
     * @param otherX the X component of the right factor
     * @param otherY the Y component of the right factor
     * @param otherZ the Z component of the right factor
     * @param result storage for the product, or null for a new Vector3f
     * @return <code>this</code> cross <code>v</code> (either
     *     <code>result</code> or a new Vector3f)
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
     * Tests for a unit vector, with 1% tolerance. The current instance is
     * unaffected.
     *
     * @return true if the current vector's length is between 0.99 and 1.01
     *     inclusive, otherwise false
     */
    public boolean isUnitVector() {
        float len = length();
        return 0.99f < len && len < 1.01f;
    }

    /**
     * Returns the length (or magnitude). The current instance is unaffected.
     *
     * @return the root-sum of the squared components (not negative)
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
     * Returns the square of the length. The current instance is unaffected.
     *
     * @return the sum of the squared components (not negative)
     */
    public float lengthSquared() {
        return x * x + y * y + z * z;
    }

    /**
     * Multiplies with the argument and returns the product as a new instance.
     * The current instance is unaffected.
     *
     * @param scalar the scaling factor
     * @return a new Vector3f
     */
    public Vector3f mult(float scalar) {
        return new Vector3f(x * scalar, y * scalar, z * scalar);
    }

    /**
     * Multiplies by the argument and returns the (modified) current instance.
     *
     * @param scalar the scaling factor
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f multLocal(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return this;
    }

    /**
     * Multiplies component-wise by the argument and returns the (modified)
     * current instance. If the argument is null, null is returned.
     *
     * @param vec the scale vector (unaffected unless it's <code>this</code>) or
     *     null for none
     * @return the (modified) current instance (for chaining) or null
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
     * Multiplies component-wise by the specified components and returns the
     * (modified) current instance.
     *
     * @param x the scale factor for the X component
     * @param y the scale factor for the Y component
     * @param z the scale factor for the Z component
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f multLocal(float x, float y, float z) {
        this.x *= x;
        this.y *= y;
        this.z *= z;
        return this;
    }

    /**
     * Multiplies component-wise by the specified components and returns the
     * product as a new instance. The current instance is unaffected.
     *
     * @param x the scale factor for the X component
     * @param y the scale factor for the Y component
     * @param z the scale factor for the Z component
     * @return a new Vector3f
     */
    public Vector3f mult(float x, float y, float z) {
        return new Vector3f(this.x * x, this.y * y, this.z * z);
    }

    /**
     * Multiplies component-wise with the specified vector and returns the
     * product in a 3rd vector. If the argument is null, null is returned.
     * Either way, the current instance is unaffected, unless it's
     * <code>store</code>.
     *
     * @param vec the scale vector (unaffected unless it's <code>store</code>)
     *     or null for none
     * @param store storage for the product, or null for a new Vector3f
     * @return either <code>store</code> or a new Vector3f or null
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
     * Divides by the argument and returns the (modified) current instance.
     *
     * @param scalar the divisor
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f divideLocal(float scalar) {
        scalar = 1f / scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return this;
    }

    /**
     * Divides component-wise by the specified components returns the (modified)
     * current instance.
     *
     * @param x the divisor for the X component
     * @param y the divisor for the Y component
     * @param z the divisor for the Z component
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f divideLocal(float x, float y, float z) {
        this.x /= x;
        this.y /= y;
        this.z /= z;
        return this;
    }

    /**
     * Divides component-wise by the argument and returns the quotient as a new
     * instance. The current instance is unaffected.
     *
     * @param divisor the divisor (not null, unaffected)
     * @return a new Vector3f
     */
    public Vector3f divide(Vector3f divisor) {
        return new Vector3f(x / divisor.x, y / divisor.y, z / divisor.z);
    }

    /**
     * Divides component-wise by the specified components and returns the quotient
     * as a new instance. The current instance is unaffected.
     *
     * @param x the divisor for the X component
     * @param y the divisor for the Y component
     * @param z the divisor for the Z component
     * @return a new Vector3f
     */
    public Vector3f divide(float x, float y, float z) {
        return new Vector3f(this.x / x, this.y / y, this.z / z);
    }

    /**
     * Divides component-wise by the argument and returns the (modified) current
     * instance.
     *
     * @param divisor the divisor (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f divideLocal(Vector3f divisor) {
        x /= divisor.x;
        y /= divisor.y;
        z /= divisor.z;
        return this;
    }

    /**
     * Returns the negative. The current instance is unaffected.
     *
     * @return a new Vector3f
     */
    public Vector3f negate() {
        return new Vector3f(-x, -y, -z);
    }

    /**
     * Negates all 3 components and returns the (modified) current instance.
     *
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f negateLocal() {
        x = -x;
        y = -y;
        z = -z;
        return this;
    }

    /**
     * Subtracts the argument and returns the difference as a new instance. The
     * current instance is unaffected.
     *
     * @param vec the vector to subtract (not null, unaffected)
     * @return a new Vector3f
     */
    public Vector3f subtract(Vector3f vec) {
        return new Vector3f(x - vec.x, y - vec.y, z - vec.z);
    }

    /**
     * Subtracts the argument and returns the (modified) current instance. If
     * the argument is null, null is returned.
     *
     * @param vec the vector to subtract (unaffected unless it's
     *     <code>this</code>) or null for none
     * @return the (modified) current instance or null
     */
    public Vector3f subtractLocal(Vector3f vec) {
        if (null == vec) {
            logger.warning("Provided vector is null, null returned.");
            return null;
        }
        x -= vec.x;
        y -= vec.y;
        z -= vec.z;
        return this;
    }

    /**
     * Subtracts the specified vector and returns the difference in a 3rd
     * vector. The current instance is unaffected unless it's
     * <code>result</code>.
     *
     * @param vec the vector to subtract (not null, unaffected unless it's
     *     <code>result</code>)
     * @param result storage for the difference, or null for a new Vector3f
     * @return either <code>result</code> or a new Vector3f
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
     * Subtracts the specified amounts from the vector's components and returns
     * the difference as a new instance. The current instance is unaffected.
     *
     * @param subtractX the amount to subtract from the X component
     * @param subtractY the amount to subtract from the Y component
     * @param subtractZ the amount to subtract from the Z component
     * @return a new Vector3f
     */
    public Vector3f subtract(float subtractX, float subtractY, float subtractZ) {
        return new Vector3f(x - subtractX, y - subtractY, z - subtractZ);
    }

    /**
     * Subtracts the specified amounts from the vector's components and returns
     * the (modified) current instance.
     *
     * @param subtractX the amount to subtract from the X component
     * @param subtractY the amount to subtract from the Y component
     * @param subtractZ the amount to subtract from the Z component
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f subtractLocal(float subtractX, float subtractY, float subtractZ) {
        x -= subtractX;
        y -= subtractY;
        z -= subtractZ;
        return this;
    }

    /**
     * Normalizes the vector to length=1 and returns the result as a new
     * instance. If the vector has length=0, a clone is returned. Either way,
     * the current instance is unaffected.
     *
     * @return a new Vector3f
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
     * Sets all 3 components to zero.
     *
     * @return the (modified) current instance (for chaining)
     */
    public Vector3f zero() {
        x = y = z = 0;
        return this;
    }

    /**
     * Tests whether the argument is a valid vector, returning false if it's
     * null or if any component is NaN or infinite.
     *
     * @param vector the vector to test (unaffected)
     * @return true if non-null and finite, otherwise false
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
     * Creates a copy. The current instance is unaffected.
     *
     * @return a new instance, equivalent to the current one
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
     * Tests for exact equality with the argument, distinguishing -0 from 0. If
     * {@code o} is null, false is returned. Either way, the current instance is
     * unaffected.
     *
     * @param o the object to compare (may be null, unaffected)
     * @return true if equal, otherwise false
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
     * Returns a hash code. If two vectors are logically equivalent, they will
     * return the same hash code. The current instance is unaffected.
     *
     * @return the hash code value
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
     * Returns a string representation. The current instance is unaffected. The
     * format is:
     *
     * <p>(XX.XXXX, YY.YYYY, ZZ.ZZZZ)
     *
     * @return the string representation
     */
    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }

    /**
     * Returns the indexed component. The vector is unaffected.
     *
     * @param index 0, 1, or 2
     * @return the X component if index=0, the Y component if index=1, or the Z
     *     component if index=2
     * @throws IllegalArgumentException if index is not 0, 1, or 2
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
     * Sets the indexed component.
     *
     * @param index which component to set: 0 &rarr; the X component, 1 &rarr;
     *     the Y component, 2 &rarr; the Z component
     * @param value the desired component value
     * @throws IllegalArgumentException if index is not 0, 1, or 2
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
