/*
 * $Id$
 * 
 * Copyright (c) 2015, Simsilica, LLC
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in 
 *    the documentation and/or other materials provided with the 
 *    distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.simsilica.mathd;


/**
 * A square matrix composed of 9 double-precision elements, used to represent
 * linear transformations of 3-dimensional vectors.
 * <p>
 * Methods with names ending in "Local" modify the current instance. They are
 * used to avoid creating temporary objects.
 * <p>
 * By convention, indices start from zero. The conventional order of indices is
 * (row, column).
 *
 *  @author    Paul Speed
 */
public class Matrix3d implements Cloneable {

    /**
     * The element in row 0, column 0.
     */
    protected double m00;
    /**
     * The element in row 0, column 1.
     */
    protected double m01;
    /**
     * The element in row 0, column 2.
     */
    protected double m02;
    /**
     * The element in row 1, column 0.
     */
    protected double m10;
    /**
     * The element in row 1, column 1.
     */
    protected double m11;
    /**
     * The element in row 1, column 2.
     */
    protected double m12;
    /**
     * The element in row 2, column 0.
     */
    protected double m20;
    /**
     * The element in row 2, column 1.
     */
    protected double m21;
    /**
     * The element in row 2, column 2.
     */
    protected double m22;
    
    /**
     * Instantiates an identity matrix (diagonals = 1, other elements = 0).
     */
    public Matrix3d() {
        makeIdentity();
    }
 
    /**
     * Instantiates a matrix with the specified elements.
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
    public Matrix3d( double m00, double m01, double m02,
                     double m10, double m11, double m12,
                     double m20, double m21, double m22 ) {
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
     * Creates a copy of the current instance.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public Matrix3d clone() {
        return new Matrix3d(m00, m01, m02,  
                            m10, m11, m12,
                            m20, m21, m22);
    }
 
    /**
     * Copies all elements of the argument to the current instance.
     *
     * @param mat the desired value (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Matrix3d set( Matrix3d mat ) {
        this.m00 = mat.m00;
        this.m01 = mat.m01;
        this.m02 = mat.m02;
        this.m10 = mat.m10;
        this.m11 = mat.m11;
        this.m12 = mat.m12;
        this.m20 = mat.m20;
        this.m21 = mat.m21;
        this.m22 = mat.m22;
        return this;
    }
 
    /**
     * Configures the matrix as an identity matrix (diagonals = 1, other
     * elements = 0).
     * 
     * @return the (modified) current instance (for chaining)
     */
    public Matrix3d makeIdentity() {
        m01 = m02 = m10 = m12 = m20 = m21 = 0;
        m00 = m11 = m22 = 1;
        return this;
    }

    /**
     * Tests for identity. The matrix is unaffected.
     *
     * @return true if all diagonal elements are 1 and all other elements are
     * 0 or -0, otherwise false
     */
    public boolean isIdentity() {
        if( m00 == 1. && m01 == 0. && m02 == 0. && m10 == 0. && m11 == 1.
                && m12 == 0. && m20 == 0. && m21 == 0. && m22 == 1. )
            return true;
        else
            return false;
    }

    /**
     * Multiplies by the scalar argument and returns the (modified) current
     * instance.
     *
     * @param scale the scaling factor
     * @return the (modified) current instance (for chaining)
     */
    public Matrix3d multLocal( double scale ) {
        this.m00 *= scale;
        this.m01 *= scale;
        this.m02 *= scale;
        this.m10 *= scale;
        this.m11 *= scale;
        this.m12 *= scale;
        this.m20 *= scale;
        this.m21 *= scale;
        this.m22 *= scale;
        
        return this;
    }
    
    /**
     * Returns the determinant. The matrix is unaffected.
     *
     * @return the determinant
     */
    public double determinant() {
        double co00 = (m11 * m22) - (m12 * m21);
        double co10 = (m12 * m20) - (m10 * m22);
        double co20 = (m10 * m21) - (m11 * m20);
        return m00 * co00 + m01 * co10 + m02 * co20;
    }
    
    /**
     * Returns the multiplicative inverse. If the current instance is singular,
     * an identity matrix is returned. The current instance is unaffected.
     *
     * @return a new Matrix3d
     */
    public Matrix3d invert() {
        double d = determinant();
        if( d == 0 )
            return new Matrix3d(); // questionable
        
        double rm00 = m11 * m22 - m12 * m21;
        double rm01 = m02 * m21 - m01 * m22;
        double rm02 = m01 * m12 - m02 * m11;
        double rm10 = m12 * m20 - m10 * m22;
        double rm11 = m00 * m22 - m02 * m20;
        double rm12 = m02 * m10 - m00 * m12;
        double rm20 = m10 * m21 - m11 * m20;
        double rm21 = m01 * m20 - m00 * m21;
        double rm22 = m00 * m11 - m01 * m10;
 
        double s = 1.0 / d;
 
        return new Matrix3d( rm00 * s, rm01 * s, rm02 * s,
                             rm10 * s, rm11 * s, rm12 * s,              
                             rm20 * s, rm21 * s, rm22 * s );
    }
    
    /**
     * Returns the transpose. The current instance is unaffected.
     *
     * @return a new Matrix3d
     */
    public Matrix3d transpose() {
        return new Matrix3d( m00, m10, m20, m01, m11, m21, m02, m12, m22 );
    }

    /**
     * Tests for exact equality with the argument, distinguishing -0 from 0. If
     * {@code o} is null, false is returned. Either way, the current instance is
     * unaffected.
     *
     * @param o the object to compare (may be null, unaffected)
     * @return true if {@code this} and {@code o} have identical values,
     * otherwise false
     */
    @Override
    public boolean equals( Object o ) {
        if( o == this )
            return true;
        if( o == null || o.getClass() != getClass() )
            return false;

        Matrix3d other = (Matrix3d)o;
        if( Double.compare(m00, other.m00) != 0 )
            return false;
        if( Double.compare(m01, other.m01) != 0 )
            return false;
        if( Double.compare(m02, other.m02) != 0 )
            return false;
        if( Double.compare(m10, other.m10) != 0 )
            return false;
        if( Double.compare(m11, other.m11) != 0 )
            return false;
        if( Double.compare(m12, other.m12) != 0 )
            return false;
        if( Double.compare(m20, other.m20) != 0 )
            return false;
        if( Double.compare(m21, other.m21) != 0 )
            return false;
        if( Double.compare(m22, other.m22) != 0 )
            return false;

        return true;
    }

    /**
     * Returns a hash code. If two matrices have identical values, they
     * will have the same hash code. The matrix is unaffected.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        long bits = Double.doubleToLongBits(m00);
        bits ^= Double.doubleToLongBits(m01) * 2L;
        bits ^= Double.doubleToLongBits(m02) * 3L;
        bits ^= Double.doubleToLongBits(m10) * 4L;
        bits ^= Double.doubleToLongBits(m11) * 5L;
        bits ^= Double.doubleToLongBits(m12) * 6L;
        bits ^= Double.doubleToLongBits(m20) * 7L;
        bits ^= Double.doubleToLongBits(m21) * 8L;
        bits ^= Double.doubleToLongBits(m22) * 9L;

        return ((int)bits) ^ ((int)(bits >> 32));
    }

    /**
     * Returns a string representation of the matrix, which is unaffected. For
     * example, an identity matrix would be represented by:
     * <pre>
     * Matrix3d[{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}]
     * </pre>
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        return "Matrix3d[{" + m00 + ", " + m01 + ", " + m02 + "}, {"
                            + m10 + ", " + m11 + ", " + m12 + "}, {"
                            + m20 + ", " + m21 + ", " + m22 + "}]";
    }
}
