/*
 * Copyright (c) 2009-2022 jMonkeyEngine
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
 * A 3x3 matrix composed of 9 single-precision elements, used to represent
 * linear transformations of 3-D coordinates, such as rotations, reflections,
 * and scaling.
 *
 * <p>Element numbering is (row, column), so m01 is the element in row 0,
 * column 1.
 *
 * <p>For pure rotations, the {@link com.jme3.math.Quaternion} class provides a
 * more efficient representation.
 *
 * <p>With one exception, the methods with names ending in "Local" modify the
 * current instance. They are used to avoid creating garbage.
 *
 * @author Mark Powell
 * @author Joshua Slack
 */
public final class Matrix3f implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;

    private static final Logger logger = Logger.getLogger(Matrix3f.class.getName());
    /**
     * The element in row 0, column 0.
     */
    protected float m00;
    /**
     * The element in row 0, column 1.
     */
    protected float m01;
    /**
     * The element in row 0, column 2.
     */
    protected float m02;
    /**
     * The element in row 1, column 0.
     */
    protected float m10;
    /**
     * The element in row 1, column 1.
     */
    protected float m11;
    /**
     * The element in row 1, column 2.
     */
    protected float m12;
    /**
     * The element in row 2, column 0.
     */
    protected float m20;
    /**
     * The element in row 2, column 1.
     */
    protected float m21;
    /**
     * The element in row 2, column 2.
     */
    protected float m22;
    /**
     * Shared instance of the all-zero matrix. Do not modify!
     */
    public static final Matrix3f ZERO = new Matrix3f(0, 0, 0, 0, 0, 0, 0, 0, 0);
    /**
     * Shared instance of the identity matrix (diagonals = 1, other elements =
     * 0). Do not modify!
     */
    public static final Matrix3f IDENTITY = new Matrix3f();

    /**
     * Instantiates an identity matrix (diagonals = 1, other elements = 0).
     */
    public Matrix3f() {
        loadIdentity();
    }

    /**
     * Instantiates a matrix with specified elements.
     *
     * @param m00 the desired value for row 0, column 0
     * @param m01 the desired value for row 0, column 1
     * @param m02 the desired value for row 0, column 2
     * @param m10 the desired value for row 1, column 0
     * @param m11 the desired value for row 1, column 1
     * @param m12 the desired value for row 1, column 2
     * @param m20 the desired value for row 2, column 0
     * @param m21 the desired value for row 2, column 1
     * @param m22 the desired value for row 2, column 2
     */
    public Matrix3f(float m00, float m01, float m02, float m10, float m11,
            float m12, float m20, float m21, float m22) {
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
    }

    /**
     * Copies the matrix argument. If the argument is null, the current instance
     * is set to identity (diagonals = 1, other elements = 0).
     *
     * @param matrix the matrix to copy (unaffected) or null for identity
     * @return the (modified) current instance (for chaining)
     */
    public Matrix3f set(Matrix3f matrix) {
        if (null == matrix) {
            loadIdentity();
        } else {
            m00 = matrix.m00;
            m01 = matrix.m01;
            m02 = matrix.m02;
            m10 = matrix.m10;
            m11 = matrix.m11;
            m12 = matrix.m12;
            m20 = matrix.m20;
            m21 = matrix.m21;
            m22 = matrix.m22;
        }
        return this;
    }

    /**
     * Returns the element at the specified position. The matrix is unaffected.
     *
     * @param i the row index (0, 1, or 2)
     * @param j the column index (0, 1, or 2)
     * @return the value of the element at (i, j)
     * @throws IllegalArgumentException if either index isn't 0, 1, or 2
     */
    @SuppressWarnings("fallthrough")
    public float get(int i, int j) {
        switch (i) {
            case 0:
                switch (j) {
                    case 0:
                        return m00;
                    case 1:
                        return m01;
                    case 2:
                        return m02;
                }
            case 1:
                switch (j) {
                    case 0:
                        return m10;
                    case 1:
                        return m11;
                    case 2:
                        return m12;
                }
            case 2:
                switch (j) {
                    case 0:
                        return m20;
                    case 1:
                        return m21;
                    case 2:
                        return m22;
                }
        }

        logger.warning("Invalid matrix index.");
        throw new IllegalArgumentException("Invalid indices into matrix.");
    }

    /**
     * Sets the specified element.
     *
     * @param i the row index (0, 1, or 2)
     * @param j the column index (0, 1, or 2)
     * @param value desired value for the element at (i, j)
     * @return the (modified) current instance (for chaining)
     * @throws IllegalArgumentException if either index isn't 0, 1, or 2
     */
    @SuppressWarnings("fallthrough")
    public Matrix3f set(int i, int j, float value) {
        switch (i) {
            case 0:
                switch (j) {
                    case 0:
                        m00 = value;
                        return this;
                    case 1:
                        m01 = value;
                        return this;
                    case 2:
                        m02 = value;
                        return this;
                }
            case 1:
                switch (j) {
                    case 0:
                        m10 = value;
                        return this;
                    case 1:
                        m11 = value;
                        return this;
                    case 2:
                        m12 = value;
                        return this;
                }
            case 2:
                switch (j) {
                    case 0:
                        m20 = value;
                        return this;
                    case 1:
                        m21 = value;
                        return this;
                    case 2:
                        m22 = value;
                        return this;
                }
        }

        logger.warning("Invalid matrix index.");
        throw new IllegalArgumentException("Invalid indices into matrix.");
    }

    /**
     * Configures from the specified column vectors. If the vectors form an
     * orthonormal basis, the result will be a pure rotation matrix.
     *
     * @param uAxis the desired value for column 0 (not null, unaffected)
     * @param vAxis the desired value for column 1 (not null, unaffected)
     * @param wAxis the desired value for column 2 (not null, unaffected)
     */
    public void fromAxes(Vector3f uAxis, Vector3f vAxis, Vector3f wAxis) {
        m00 = uAxis.x;
        m10 = uAxis.y;
        m20 = uAxis.z;

        m01 = vAxis.x;
        m11 = vAxis.y;
        m21 = vAxis.z;

        m02 = wAxis.x;
        m12 = wAxis.y;
        m22 = wAxis.z;
    }

    /**
     * Configures as a rotation matrix equivalent to the argument.
     *
     * @param quaternion the input quaternion (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Matrix3f set(Quaternion quaternion) {
        return quaternion.toRotationMatrix(this);
    }

    /**
     * Configures as an identity matrix (diagonals = 1, other elements = 0).
     */
    public void loadIdentity() {
        m01 = m02 = m10 = m12 = m20 = m21 = 0;
        m00 = m11 = m22 = 1;
    }

    /**
     * Tests for exact identity. The matrix is unaffected.
     *
     * @return true if all diagonals = 1 and all other elements = 0 or -0,
     * otherwise false
     */
    public boolean isIdentity() {
        return (m00 == 1 && m01 == 0 && m02 == 0)
                && (m10 == 0 && m11 == 1 && m12 == 0)
                && (m20 == 0 && m21 == 0 && m22 == 1);
    }

    /**
     * Multiplies with the specified matrix and returns the product in a 3rd
     * matrix. The current instance is unaffected unless it's {@code product}.
     *
     * <p>Note that matrix multiplication is noncommutative, so generally
     * q * p != p * q.
     *
     * <p>It is safe for {@code mat} and {@code product} to be the same object.
     *
     * @param mat the right factor (not null, unaffected unless it's {@code
     *     product})
     * @param product storage for the product, or null for a new Matrix3f
     * @return {@code this} times {@code mat} (either {@code product} or a new
     *     Matrix3f)
     */
    public Matrix3f mult(Matrix3f mat, Matrix3f product) {
        float temp00, temp01, temp02;
        float temp10, temp11, temp12;
        float temp20, temp21, temp22;

        if (product == null) {
            product = new Matrix3f();
        }
        temp00 = m00 * mat.m00 + m01 * mat.m10 + m02 * mat.m20;
        temp01 = m00 * mat.m01 + m01 * mat.m11 + m02 * mat.m21;
        temp02 = m00 * mat.m02 + m01 * mat.m12 + m02 * mat.m22;
        temp10 = m10 * mat.m00 + m11 * mat.m10 + m12 * mat.m20;
        temp11 = m10 * mat.m01 + m11 * mat.m11 + m12 * mat.m21;
        temp12 = m10 * mat.m02 + m11 * mat.m12 + m12 * mat.m22;
        temp20 = m20 * mat.m00 + m21 * mat.m10 + m22 * mat.m20;
        temp21 = m20 * mat.m01 + m21 * mat.m11 + m22 * mat.m21;
        temp22 = m20 * mat.m02 + m21 * mat.m12 + m22 * mat.m22;

        product.m00 = temp00;
        product.m01 = temp01;
        product.m02 = temp02;
        product.m10 = temp10;
        product.m11 = temp11;
        product.m12 = temp12;
        product.m20 = temp20;
        product.m21 = temp21;
        product.m22 = temp22;

        return product;
    }

    /**
     * Applies the linear transformation to specified vector and stores the
     * result in another vector. The matrix is unaffected.
     *
     * <p>This can also be described as multiplying the matrix by a column
     * vector.
     *
     * <p>It is safe for {@code vec} and {@code product} to be the same object.
     *
     * @param vec the coordinates to transform (not null, unaffected unless it's
     *     {@code product})
     * @param product storage for the result, or null for a new Vector3f
     * @return either {@code product} or a new Vector3f
     */
    public Vector3f mult(Vector3f vec, Vector3f product) {
        if (null == product) {
            product = new Vector3f();
        }

        float x = vec.x;
        float y = vec.y;
        float z = vec.z;

        product.x = m00 * x + m01 * y + m02 * z;
        product.y = m10 * x + m11 * y + m12 * z;
        product.z = m20 * x + m21 * y + m22 * z;
        return product;
    }

    /**
     * Multiplies by the scalar argument and returns the (modified) current
     * instance.
     *
     * @param scale the scaling factor
     * @return the (modified) current instance (for chaining)
     */
    public Matrix3f multLocal(float scale) {
        m00 *= scale;
        m01 *= scale;
        m02 *= scale;
        m10 *= scale;
        m11 *= scale;
        m12 *= scale;
        m20 *= scale;
        m21 *= scale;
        m22 *= scale;
        return this;
    }

    /**
     * Returns the multiplicative inverse in the specified storage. If the
     * current instance is singular, an all-zero matrix is returned. In either
     * case, the current instance is unaffected.
     *
     * <p>If {@code this} and {@code store} are the same object, the result is
     * undefined.
     *
     * @param store storage for the result, or null for a new Matrix3f
     * @return either {@code store} or a new Matrix3f
     */
    public Matrix3f invert(Matrix3f store) {
        if (store == null) {
            store = new Matrix3f();
        }

        float det = determinant();
        if (FastMath.abs(det) <= FastMath.FLT_EPSILON) {
            return store.zero();
        }

        store.m00 = m11 * m22 - m12 * m21;
        store.m01 = m02 * m21 - m01 * m22;
        store.m02 = m01 * m12 - m02 * m11;
        store.m10 = m12 * m20 - m10 * m22;
        store.m11 = m00 * m22 - m02 * m20;
        store.m12 = m02 * m10 - m00 * m12;
        store.m20 = m10 * m21 - m11 * m20;
        store.m21 = m01 * m20 - m00 * m21;
        store.m22 = m00 * m11 - m01 * m10;

        store.multLocal(1f / det);
        return store;
    }

    /**
     * Returns the determinant. The matrix is unaffected.
     *
     * @return the determinant
     */
    public float determinant() {
        float fCo00 = m11 * m22 - m12 * m21;
        float fCo10 = m12 * m20 - m10 * m22;
        float fCo20 = m10 * m21 - m11 * m20;
        float fDet = m00 * fCo00 + m01 * fCo10 + m02 * fCo20;
        return fDet;
    }

    /**
     * Sets all elements to zero.
     *
     * @return the (modified) current instance (for chaining)
     */
    public Matrix3f zero() {
        m00 = m01 = m02 = m10 = m11 = m12 = m20 = m21 = m22 = 0.0f;
        return this;
    }

    /**
     * Returns a string representation of the matrix, which is unaffected. For
     * example, the identity matrix is represented by:
     * <pre>
     * Matrix3f
     * [
     *  1.0  0.0  0.0
     *  0.0  1.0  0.0
     *  0.0  0.0  1.0
     * ]
     * </pre>
     *
     * @return the string representation (not null, not empty)
     */
    @Override
    public String toString() {
        StringBuilder result = new StringBuilder("Matrix3f\n[\n");
        result.append(" ");
        result.append(m00);
        result.append("  ");
        result.append(m01);
        result.append("  ");
        result.append(m02);
        result.append(" \n");
        result.append(" ");
        result.append(m10);
        result.append("  ");
        result.append(m11);
        result.append("  ");
        result.append(m12);
        result.append(" \n");
        result.append(" ");
        result.append(m20);
        result.append("  ");
        result.append(m21);
        result.append("  ");
        result.append(m22);
        result.append(" \n]");
        return result.toString();
    }

    /**
     * Returns a hash code. If two matrices have identical values, they will
     * have the same hash code. The matrix is unaffected.
     *
     * @return a 32-bit value for use in hashing
     * @see java.lang.Object#hashCode()
     */
    @Override
    public int hashCode() {
        int hash = 37;
        hash = 37 * hash + Float.floatToIntBits(m00);
        hash = 37 * hash + Float.floatToIntBits(m01);
        hash = 37 * hash + Float.floatToIntBits(m02);

        hash = 37 * hash + Float.floatToIntBits(m10);
        hash = 37 * hash + Float.floatToIntBits(m11);
        hash = 37 * hash + Float.floatToIntBits(m12);

        hash = 37 * hash + Float.floatToIntBits(m20);
        hash = 37 * hash + Float.floatToIntBits(m21);
        hash = 37 * hash + Float.floatToIntBits(m22);

        return hash;
    }

    /**
     * Tests for exact equality with the argument, distinguishing -0 from 0. If
     * {@code o} is null, false is returned. Either way, the current instance is
     * unaffected.
     *
     * @param o the object to compare (may be null, unaffected)
     * @return true if {@code this} and {@code o} have identical values,
     *     otherwise false
     */
    @Override
    public boolean equals(Object o) {
        if (o == null || o.getClass() != getClass()) {
            return false;
        }

        if (this == o) {
            return true;
        }

        Matrix3f comp = (Matrix3f) o;
        if (Float.compare(m00, comp.m00) != 0) {
            return false;
        }
        if (Float.compare(m01, comp.m01) != 0) {
            return false;
        }
        if (Float.compare(m02, comp.m02) != 0) {
            return false;
        }

        if (Float.compare(m10, comp.m10) != 0) {
            return false;
        }
        if (Float.compare(m11, comp.m11) != 0) {
            return false;
        }
        if (Float.compare(m12, comp.m12) != 0) {
            return false;
        }

        if (Float.compare(m20, comp.m20) != 0) {
            return false;
        }
        if (Float.compare(m21, comp.m21) != 0) {
            return false;
        }
        if (Float.compare(m22, comp.m22) != 0) {
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
    public Matrix3f clone() {
        try {
            return (Matrix3f) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new AssertionError(); // can not happen
        }
    }
}
