/*
 * Copyright (c) 2009-2020 jMonkeyEngine
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
 * <code>Matrix3f</code> defines a 3x3 matrix. Matrix data is maintained
 * internally and is accessible via the get and set methods. Convenience methods
 * are used for matrix operations as well as generating a matrix from a given
 * set of values.
 * 
 * @author Mark Powell
 * @author Joshua Slack
 */
public final class Matrix3f implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;

    private static final Logger logger = Logger.getLogger(Matrix3f.class.getName());
    protected float m00, m01, m02;
    protected float m10, m11, m12;
    protected float m20, m21, m22;
    public static final Matrix3f ZERO = new Matrix3f(0, 0, 0, 0, 0, 0, 0, 0, 0);
    public static final Matrix3f IDENTITY = new Matrix3f();

    /**
     * Constructor instantiates a new <code>Matrix3f</code> object. The
     * initial values for the matrix is that of the identity matrix.
     *  
     */
    public Matrix3f() {
        loadIdentity();
    }

    /**
     * constructs a matrix with the given values.
     * 
     * @param m00
     *            0x0 in the matrix.
     * @param m01
     *            0x1 in the matrix.
     * @param m02
     *            0x2 in the matrix.
     * @param m10
     *            1x0 in the matrix.
     * @param m11
     *            1x1 in the matrix.
     * @param m12
     *            1x2 in the matrix.
     * @param m20
     *            2x0 in the matrix.
     * @param m21
     *            2x1 in the matrix.
     * @param m22
     *            2x2 in the matrix.
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
     * <code>copy</code> transfers the contents of a given matrix to this
     * matrix. If a null matrix is supplied, this matrix is set to the identity
     * matrix.
     * 
     * @param matrix
     *            the matrix to copy.
     * @return this
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
     * 
     * <code>set</code> defines the values of the matrix based on a supplied
     * <code>Quaternion</code>. It should be noted that all previous values
     * will be overridden.
     * 
     * @param quaternion
     *            the quaternion to create a rotational matrix from.
     * @return this
     */
    public Matrix3f set(Quaternion quaternion) {
        return quaternion.toRotationMatrix(this);
    }

    /**
     * <code>loadIdentity</code> sets this matrix to the identity matrix.
     * Where all values are zero except those along the diagonal which are one.
     *  
     */
    public void loadIdentity() {
        m01 = m02 = m10 = m12 = m20 = m21 = 0;
        m00 = m11 = m22 = 1;
    }

    /**
     * @return true if this matrix is identity
     */
    public boolean isIdentity() {
        return (m00 == 1 && m01 == 0 && m02 == 0)
                && (m10 == 0 && m11 == 1 && m12 == 0)
                && (m20 == 0 && m21 == 0 && m22 == 1);
    }

    /**
     * <code>toString</code> returns the string representation of this object.
     * It is in a format of a 3x3 matrix. For example, an identity matrix would
     * be represented by the following string. com.jme.math.Matrix3f <br>[<br>
     * 1.0  0.0  0.0 <br>
     * 0.0  1.0  0.0 <br>
     * 0.0  0.0  1.0 <br>]<br>
     * 
     * @return the string representation of this object.
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
     * 
     * <code>hashCode</code> returns the hash code value as an integer and is
     * supported for the benefit of hashing based collection classes such as
     * Hashtable, HashMap, HashSet etc.
     * 
     * @return the hashcode for this instance of Matrix4f.
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
     * are these two matrices the same? they are is they both have the same mXX values.
     *
     * @param o
     *            the object to compare for equality
     * @return true if they are equal
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Matrix3f) || o == null) {
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

    @Override
    public Matrix3f clone() {
        try {
            return (Matrix3f) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new AssertionError(); // can not happen
        }
    }
}
