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
 * Represents rotations and orientations in 3-dimensional space, using
 * double-precision components.
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
     * Instantiate an identity Quatd (0,0,0,1).
     */
    public Quatd() {
        this( 0, 0, 0, 1 );
    }

    /**
     * Instantiate a Quatd with the specified components.
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
     * Instantiate a copy of the specified Quatd.
     *
     * @param quat the Quatd to copy (not null, unaffected)
     */
    public Quatd( Quatd quat ) {
        this(quat.x, quat.y, quat.z, quat.w);
    }

    /**
     * Instantiate based on the specified Quaternion.
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
     * Create a copy. The current instance is unaffected.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public final Quatd clone() {
        return new Quatd(x,y,z,w);
    }

    /**
     * Instantiate a Quaternion based on this Quatd.
     *
     * @return a new Quaternion
     */
    public Quaternion toQuaternion() {
        return new Quaternion((float)x, (float)y, (float)z, (float)w);
    }

    /**
     * Generate the hash code for this Quatd.
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
     * Test for strict equality with another object.
     *
     * @param o the object to compare to (may be null, unaffected)
     * @return true if the objects have the same value, otherwise false
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
     * Set all 4 components to the specified values.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     * @param w the desired W component
     * @return this Quatd
     */
    public final Quatd set( double x, double y, double z, double w ) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }

    /**
     * Copy all 4 components from the argument.
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
     * Copy all components of the specified Quaternion to this Quatd.
     *
     * @param quat the input Quaternion (not null, unaffected)
     * @return this Quatd
     */
    public final Quatd set( Quaternion quat ) {
        this.x = quat.getX();
        this.y = quat.getY();
        this.z = quat.getZ();
        this.w = quat.getW();
        return this;
    }

    /**
     * Take the Hamilton product of this Quatd times the specified Quatd to
     * yield a new Quatd.
     *
     * @param q the right factor (not null, unaffected)
     * @return a new instance
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
     * Take the Hamilton product of this Quatd times the specified Quatd in
     * place.
     *
     * It IS safe for q and this to be the same object.
     *
     * @param q the right factor (not null, unaffected unless it's this)
     * @return this Quatd
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
     * Rotate the specified vector by this Quatd to produce a new vector.
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
     * Multiplies by the specified quaternion and returns the product in a 3rd
     * quaternion. The current instance is unaffected, unless it's
     * {@code storeResult}.
     *
     * <p>This method is used to combine rotations. Note that quaternion
     * multiplication is noncommutative, so generally q * p != p * q.
     *
     * <p>It IS safe for {@code q} and {@code storeResult} to be the same
     * object. However, if {@code this} and {@code storeResult} are the same
     * object, the result is undefined.
     *
     * @param q the right factor (not null, unaffected unless it's 
     * {@code storeResult})
     * @param storeResult storage for the product, or null for a new Quatd
     * @return {@code this * q} (either {@code storeResult} or a new Quatd)
     */
    public Quatd mult( Quatd q, Quatd storeResult ) {
        if (storeResult == null) {
            storeResult = new Quatd();
        }
        double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
        storeResult.x = x * qw + y * qz - z * qy + w * qx;
        storeResult.y = -x * qz + y * qw + z * qx + w * qy;
        storeResult.z = x * qy - y * qx + z * qw + w * qz;
        storeResult.w = -x * qx - y * qy - z * qz + w * qw;
        return storeResult;
    }

    /**
     * Rotate the specified vector by this Quatd and store the result in another
     * vector.
     *
     * It IS safe for v and result to be the same object.
     *
     * @param v the input vector (not null, unaffected unless it's result)
     * @param result storage for the result (not null)
     * @return result
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
     * Determine the squared length of this Quatd.
     *
     * @return the squared length (&ge;0)
     */
    public final double lengthSq() {
        return (x * x) + (y * y) + (z * z) + (w * w);
    }

    /**
     * Normalize this Quatd in place.
     *
     * @return this Quatd
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
     * Convert to an equivalent rotation matrix. The current instance is
     * unaffected.
     *
     * <p>Note: the result is created from a normalized version of the current
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
     * Returns the multiplicative inverse.  For a quaternion with norm=0, null is
     * returned. Either way, the current instance is unaffected.
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
     * (pitch, yaw, roll)).  They are applied in order: (y, z, x) aka (yaw, roll, pitch).
     * @see <a href="http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm">http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm</a>
     *
     * @param xAngle the desired rotation about the +X axis (in radians)
     * @param yAngle the desired rotation about the +Y axis (in radians)
     * @param zAngle the desired rotation about the +Z axis (in radians)
     * @return this Quatd
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
     * Represent this Quatd as a String.
     *
     * The format is:
     *
     * Quatd[XX.XXXXXXXXXXXXX, YY.YYYYYYYYYYYYY, ZZ.ZZZZZZZZZZZZZ, WW.WWWWWWWWWWWWW]
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        return "Quatd[" + x + ", " + y + ", " + z + ", " + w + "]";
    }
}
