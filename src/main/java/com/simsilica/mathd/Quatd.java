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

import com.jme3.math.Quaternion;


/**
 * Used to efficiently represent rotations and orientations in 3-dimensional
 * space, without risk of gimbal lock. Each instance has 4 double-precision
 * components: 3 imaginary components (X, Y, and Z) and a real component (W).
 * <p>
 * Methods with names ending in "Local" modify the current instance. They are
 * used to avoid creating temporary objects.
 * <p>
 * Mathematically, quaternions are an extension of complex numbers. In
 * mathematics texts, W often appears first, but here the conventional order
 * is (X, Y, Z, W).
 *
 *  @version   $Revision: 3951 $
 *  @author    Paul Speed
 */
public final class Quatd implements Cloneable {

    /**
     * The first imaginary (X) component. Not an angle!
     */
    public double x;
    /**
     * The 2nd imaginary (Y) component. Not an angle!
     */
    public double y;
    /**
     * The 3rd imaginary (Z) component. Not an angle!
     */
    public double z;
    /**
     * The real (W) component. Not an angle!
     */
    public double w;

    /**
     * Instantiates an identity quaternion: all components zeroed except
     * {@code w}, which is set to 1.
     */
    public Quatd() {
        this( 0, 0, 0, 1 );
    }

    /**
     * Instantiates a quaternion with the specified components.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     * @param w the desired W component
     */
    public Quatd( double x, double y, double z, double w ) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Instantiates a copy of the argument.
     *
     * @param quat the quaternion to copy (not null, unaffected)
     */
    public Quatd( Quatd quat ) {
        this(quat.x, quat.y, quat.z, quat.w);
    }

    /**
     * Instantiates based on the specified (single-precision) Quaternion.
     *
     * @param quat the input Quaternion (not null, unaffected)
     */
    public Quatd( Quaternion quat ) {
        this.x = quat.getX();
        this.y = quat.getY();
        this.z = quat.getZ();
        this.w = quat.getW();
    }

    /**
     * Creates a copy. The current instance is unaffected.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public final Quatd clone() {
        return new Quatd(x,y,z,w);
    }

    /**
     * Creates a (single-precision) Quaternion that approximates the current
     * instance.
     *
     * @return a new Quaternion
     */
    public Quaternion toQuaternion() {
        return new Quaternion((float)x, (float)y, (float)z, (float)w);
    }

    /**
     * Returns a hash code. If two quaternions have identical values, they
     * will have the same hash code. The current instance is unaffected.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        long bits = Double.doubleToLongBits(x);
        bits ^= Double.doubleToLongBits(y) * 31;
        bits ^= Double.doubleToLongBits(z) * 31;
        bits ^= Double.doubleToLongBits(w) * 31;

        return ((int)bits) ^ ((int)(bits >> 32));
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
    public boolean equals( Object o ) {
        if( o == this )
            return true;
        if( o == null || o.getClass() != getClass() )
            return false;
        Quatd other = (Quatd)o;
        if( Double.compare(x, other.x) != 0 )
            return false;
        if( Double.compare(y, other.y) != 0 )
            return false;
        if( Double.compare(z, other.z) != 0 )
            return false;
        if( Double.compare(w, other.w) != 0 )
            return false;
        return true;
    }

    /**
     * Tests for an identity rotation. The quaternion is unaffected.
     *
     * @return true if W is non-zero and not NaN
     * and X, Y, and Z are all 0 or -0, otherwise false
     */
    public boolean isRotationIdentity() {
        if( x == 0. && y == 0. && z == 0. && w != 0. && !Double.isNaN(w) )
            return true;
        else
            return false;
    }

    /**
     * Tests for a zero value. The quaternion is unaffected.
     *
     * @return true if all components are 0 or -0, otherwise false
     */
    public boolean isZero() {
        if( x == 0. && y == 0. && z == 0. && w == 0. )
            return true;
        else
            return false;
    }

    /**
     * Sets all 4 components to the specified values.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     * @param w the desired W component
     * @return the (modified) current instance (for chaining)
     */
    public final Quatd set( double x, double y, double z, double w ) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }

    /**
     * Copies all 4 components from the argument.
     *
     * @param q the quaternion to copy (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public final Quatd set( Quatd q ) {
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
        return this;
    }

    /**
     * Copies all 4 components of the specified (single-precision) Quaternion to
     * the current instance.
     *
     * @param quat the input Quaternion (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public final Quatd set( Quaternion quat ) {
        this.x = quat.getX();
        this.y = quat.getY();
        this.z = quat.getZ();
        this.w = quat.getW();
        return this;
    }

    /**
     * Takes the Hamilton product of the current instance times the argument to
     * yield a new Quatd. The current instance is unaffected.
     * <p>
     * It IS safe for {@code q} and {@code this} to be the same object.
     * <p>
     * This method is used to combine rotations. Note that the Hamilton product
     * is noncommutative, so generally q * p != p * q.
     *
     * @param q the right factor (not null, unaffected)
     * @return {@code this * q} (a new instance)
     */
    public final Quatd mult( Quatd q ) {
        double qx = q.x;
        double qy = q.y;
        double qz = q.z;
        double qw = q.w;

        double xr = x * qw + y * qz - z * qy + w * qx;
        double yr = -x * qz + y * qw + z * qx + w * qy;
        double zr = x * qy - y * qx + z * qw + w * qz;
        double wr = -x * qx - y * qy - z * qz + w * qw;

        return new Quatd(xr, yr, zr, wr);
    }

    /**
     * Takes the Hamilton product of the current instance times the first
     * argument and returns the product in the 2nd argument. The current
     * instance is unaffected, unless it's {@code result}.
     * <p>
     * This method is used to combine rotations. Note that the Hamilton product
     * is noncommutative, so generally q * p != p * q.
     * <p>
     * It IS safe for any or all of {@code q}, {@code result}, and {@code this}
     * to be the same object.
     *
     * @param q the right factor (not null, unaffected unless it's
     * {@code result})
     * @param result storage for the product, or null for a new Quatd
     * @return {@code this * q} (either {@code result} or a new Quatd)
     */
    public final Quatd mult( Quatd q, Quatd result ) {
        double qx = q.x;
        double qy = q.y;
        double qz = q.z;
        double qw = q.w;

        double xr = x * qw + y * qz - z * qy + w * qx;
        double yr = -x * qz + y * qw + z * qx + w * qy;
        double zr = x * qy - y * qx + z * qw + w * qz;
        double wr = -x * qx - y * qy - z * qz + w * qw;

        if( result == null ) {
            result = new Quatd(xr, yr, zr, wr); 
        } else {
            result.set(xr, yr, zr, wr);
        } 
        return result;
    }

    /**
     * Takes the Hamilton product of the current instance times the argument,
     * in place.
     * <p>
     * This method is used to combine rotations. Note that the Hamilton product
     * is noncommutative, so generally q * p != p * q.
     * <p>
     * It IS safe for {@code q} and {@code this} to be the same object.
     *
     * @param q the right factor (not null, unaffected unless it's {@code this})
     * @return the (modified) current instance (for chaining)
     */
    public final Quatd multLocal( Quatd q ) {
        double qx = q.x;
        double qy = q.y;
        double qz = q.z;
        double qw = q.w;

        double xr = x * qw + y * qz - z * qy + w * qx;
        double yr = -x * qz + y * qw + z * qx + w * qy;
        double zr = x * qy - y * qx + z * qw + w * qz;
        double wr = -x * qx - y * qy - z * qz + w * qw;

        x = xr;
        y = yr;
        z = zr;
        w = wr;
        return this;
    }

    /**
     * Rotates the argument vector to produce a new vector. The quaternion
     * is unaffected.
     *
     * @param v the input vector (not null, unaffected)
     * @return a new instance
     */
    public Vec3d mult( Vec3d v ) {
        if( v.x == 0 && v.y == 0 && v.z == 0 )
            return new Vec3d();

        double vx = v.x;
        double vy = v.y;
        double vz = v.z;

        double rx = w * w * vx + 2 * y * w * vz - 2 * z * w * vy + x * x
                    * vx + 2 * y * x * vy + 2 * z * x * vz - z * z * vx - y
                    * y * vx;
        double ry = 2 * x * y * vx + y * y * vy + 2 * z * y * vz + 2 * w
                    * z * vx - z * z * vy + w * w * vy - 2 * x * w * vz - x
                    * x * vy;
        double rz = 2 * x * z * vx + 2 * y * z * vy + z * z * vz - 2 * w
                    * y * vx - y * y * vz + 2 * w * x * vy - x * x * vz + w
                    * w * vz;

        return new Vec3d(rx, ry, rz);
    }

    /**
     * Rotates a specified vector and return the result in another vector. The
     * quaternion is unaffected.
     * <p>
     * It IS safe for {@code v} and {@code result} to be the same object.
     *
     * @param v the vector to rotate (not null, unaffected unless it's
     * {@code result})
     * @param result storage for the result (not null)
     * @return the (rotated) vector {@code result}
     */
    public Vec3d mult( Vec3d v, Vec3d result ) {
        if( v.x == 0 && v.y == 0 && v.z == 0 ) {
            if( v != result )
                result.set(0,0,0);
            return result;
        }

        double vx = v.x;
        double vy = v.y;
        double vz = v.z;

        double rx = w * w * vx + 2 * y * w * vz - 2 * z * w * vy + x * x
                    * vx + 2 * y * x * vy + 2 * z * x * vz - z * z * vx - y
                    * y * vx;
        double ry = 2 * x * y * vx + y * y * vy + 2 * z * y * vz + 2 * w
                    * z * vx - z * z * vy + w * w * vy - 2 * x * w * vz - x
                    * x * vy;
        double rz = 2 * x * z * vx + 2 * y * z * vy + z * z * vz - 2 * w
                    * y * vx - y * y * vz + 2 * w * x * vy - x * x * vz + w
                    * w * vz;

        result.set(rx, ry, rz);
        return result;
    }

    /**
     * Returns the squared length.
     *
     * @return the squared length (&ge;0)
     */
    public final double lengthSq() {
        return (x * x) + (y * y) + (z * z) + (w * w);
    }

    /**
     * Normalizes the current instance in place.
     *
     * @return the (modified) current instance (for chaining)
     */
    public final Quatd normalizeLocal() {
        double d = lengthSq();
        if( d == 0 ) {
            w = 1;
            return this;
        }

        double s = 1.0 / Math.sqrt(d);
        x *= s;
        y *= s;
        z *= s;
        w *= s;

        return this;
    }

    /**
     * Converts the quaternion to an equivalent rotation matrix. The quaternion
     * is unaffected.
     * <p>
     * Note: the result is created from a normalized version of the current
     * instance.
     *
     * @return a new 3x3 rotation matrix
     */
    public Matrix3d toRotationMatrix() {
        double d = lengthSq();
        double s = 2 / d;

        // Premultiply for better performance
        double xs = x * s;
        double ys = y * s;
        double zs = z * s;
        double xx = x * xs;
        double xy = x * ys;
        double xz = x * zs;
        double xw = w * xs;
        double yy = y * ys;
        double yz = y * zs;
        double yw = w * ys;
        double zz = z * zs;
        double zw = w * zs;

        // using s=2/norm (instead of 1/norm) saves 9 multiplications by 2 here
        double m00 = 1 - (yy + zz);
        double m01 = (xy - zw);
        double m02 = (xz + yw);
        double m10 = (xy + zw);
        double m11 = 1 - (xx + zz);
        double m12 = (yz - xw);
        double m20 = (xz - yw);
        double m21 = (yz + xw);
        double m22 = 1 - (xx + yy);

        return new Matrix3d( m00, m01, m02,
                             m10, m11, m12,
                             m20, m21, m22 );
    }

    /**
     * Returns the multiplicative inverse.  For a quaternion with norm=0, {@code
     * null} is returned. Either way, the current instance is unaffected.
     *
     * @return a new instance, or null if not invertible
     */
    public Quatd inverse() {
        double norm = lengthSq();
        if( norm <= 0 )
            return null;

        double inv = 1 / norm;
        return new Quatd(-x * inv, -y * inv, -z * inv, w * inv);
    }

    /**
     * Builds a Quaternion from the Euler rotation angles (x,y,z) aka
     * (pitch, yaw, roll).  They are applied in order: (y, z, x) aka (yaw, roll, pitch).
     * @see <a href="http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm">http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm</a>
     *
     * @param xAngle the desired rotation about the +X axis (in radians)
     * @param yAngle the desired rotation about the +Y axis (in radians)
     * @param zAngle the desired rotation about the +Z axis (in radians)
     * @return the (modified) current instance (for chaining)
     */
    public Quatd fromAngles( double xAngle, double yAngle, double zAngle ) {
        double a;
        double sinY, sinZ, sinX, cosY, cosZ, cosX;

        a = zAngle * 0.5f;
        sinZ = Math.sin(a);
        cosZ = Math.cos(a);
        a = yAngle * 0.5f;
        sinY = Math.sin(a);
        cosY = Math.cos(a);
        a = xAngle * 0.5f;
        sinX = Math.sin(a);
        cosX = Math.cos(a);

        // premultiply some reused stuff
        double cosYXcosZ = cosY * cosZ;
        double sinYXsinZ = sinY * sinZ;
        double cosYXsinZ = cosY * sinZ;
        double sinYXcosZ = sinY * cosZ;

        w = (cosYXcosZ * cosX - sinYXsinZ * sinX);
        x = (cosYXcosZ * sinX + sinYXsinZ * cosX);
        y = (sinYXcosZ * cosX + cosYXsinZ * sinX);
        z = (cosYXsinZ * cosX - sinYXcosZ * sinX);

        normalizeLocal();
        return this;
    }

    /**
     * Returns a string representation of the quaternion, which is unaffected.
     * For example, the identity quaternion is represented by:
     * <pre>
     * Quatd[0.0, 0.0, 0.0, 1.0]
     * </pre>
     *
     * @return the string representation (not null, not empty)
     */
    @Override
    public String toString() {
        return "Quatd[" + x + ", " + y + ", " + z + ", " + w + "]";
    }
}
