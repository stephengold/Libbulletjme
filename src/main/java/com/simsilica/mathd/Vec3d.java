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

import com.jme3.math.Vector3f;


/**
 * A vector composed of 3 double-precision components, used to represent
 * locations, offsets, and directions in 3-dimensional space.
 *
 *  @version   $Revision: 3951 $
 *  @author    Paul Speed
 */
public class Vec3d implements Cloneable {

    /**
     * shared instance of the +X direction (1,0,0) - Do not modify!
     */
    public static final Vec3d UNIT_X = new Vec3d(1,0,0);
    /**
     * shared instance of the +Y direction (0,1,0) - Do not modify!
     */
    public static final Vec3d UNIT_Y = new Vec3d(0,1,0);
    /**
     * shared instance of the +Z direction (0,0,1) - Do not modify!
     */
    public static final Vec3d UNIT_Z = new Vec3d(0,0,1);
    /**
     * shared instance of the all-zero vector (0,0,0) - Do not modify!
     */
    public static final Vec3d ZERO = new Vec3d();
    /**
     * X component of the vector
     */
    public double x;
    /**
     * Y component of the vector
     */
    public double y;
    /**
     * Z component of the vector
     */
    public double z;
 
    /**
     * Instantiate a vector with the value (0,0,0).
     */
    public Vec3d() {
    }
    
    /**
     * Instantiate a vector with the specified components.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     */
    public Vec3d( double x, double y, double z ) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Instantiate a copy of the specified Vec3d.
     *
     * @param v the Vec3d to copy (not null, unaffected)
     */
    public Vec3d( Vec3d v ) {
        this(v.x, v.y, v.z);
    }

    /**
     * Instantiate based on the specified Vector3f.
     *
     * @param v the input Vector3f (not null, unaffected)
     */
    public Vec3d( Vector3f v ) {
        this(v.x, v.y, v.z);
    }
 
    /**
     * Instantiate a Vector3f based on this vector.
     *
     * @return a new Vector3f
     */
    public Vector3f toVector3f() {
        return new Vector3f((float)x, (float)y, (float)z);
    }
 
    /**
     * Generate the hash code for this vector.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        long bits = Double.doubleToLongBits(x);
        bits ^= Double.doubleToLongBits(y) * 31;
        bits ^= Double.doubleToLongBits(z) * 31;
        
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
        Vec3d other = (Vec3d)o;
        if( Double.compare(x, other.x) != 0 )
            return false;
        if( Double.compare(y, other.y) != 0 )
            return false;
        if( Double.compare(z, other.z) != 0 )
            return false;
        return true;
    }

    /**
     * Set all components of this vector to the specified values.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     * @return this vector
     */
    public final Vec3d set( double x, double y, double z ) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }
    
    /**
     * Copy all components of the specified vector to this vector.
     *
     * @param v the desired value (not null, unaffected)
     * @return this vector
     */
    public final Vec3d set( Vec3d v ) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
        return this;
    }

    /**
     * Copy all components of the specified Vector3f to this vector.
     *
     * @param v the input Vector3f (not null, unaffected)
     * @return this vector
     */
    public final Vec3d set( Vector3f v ) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
        return this;
    }

    /**
     * Create a copy of this vector.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public final Vec3d clone() {
        return new Vec3d(x,y,z);
    }
 
    /**
     * Determine the indexed component of this vector.
     *
     * @param i 0 for the X component, 1 for the Y component, or 2 for the Z
     * component
     * @return the component value
     * @throws IndexOutOfBoundsException if index is not 0, 1, or 2
     */
    public double get( int i ) {
        switch( i ) {
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                throw new IndexOutOfBoundsException( "Index:" + i );
        }
    }
    
    /**
     * Alter the indexed component of this vector.
     *
     * @param i 0 for the X component, 1 for the Y component, or 2 for the Z
     * component
     * @param d the desired value for the component
     * @return this vector
     * @throws IndexOutOfBoundsException if index is not 0, 1, or 2
     */
    public Vec3d set( int i, double d ) {
        switch( i ) {
            case 0:
                this.x = d;
                break;
            case 1:
                this.y = d;
                break;
            case 2:
                this.z = d;
                break;
            default:
                throw new IndexOutOfBoundsException( "Index:" + i );
        }
        return this;
    }
 
    /**
     * Sum this vector with the specified vector to yield a new vector.
     *
     * @param v the vector to add (not null, unaffected)
     * @return a new instance
     */
    public final Vec3d add( Vec3d v ) {
        return new Vec3d(x + v.x, y + v.y, z + v.z);
    }

    /**
     * Sum this vector with the specified components to yield a new vector.
     *
     * @param vx the amount to increase the X component
     * @param vy the amount to increase the Y component
     * @param vz the amount to increase the Z component
     * @return a new instance
     */
    public final Vec3d add( double vx, double vy, double vz ) {
        return new Vec3d(x + vx, y + vy, z + vz);
    }

    /**
     * Subtract the specified vector to yield a new vector.
     *
     * @param v the vector to subtract (not null, unaffected)
     * @return a new instance
     */
    public final Vec3d subtract( Vec3d v ) {
        return new Vec3d(x - v.x, y - v.y, z - v.z);
    }

    /**
     * Add the specified vector to this vector in place.
     *
     * It IS safe for v and this to be the same object.
     *
     * @param v the vector to add (not null, unaffected unless it's this)
     * @return this vector
     */
    public final Vec3d addLocal( Vec3d v ) {
        x += v.x;
        y += v.y;
        z += v.z;
        return this;
    }

    /**
     * Add the specified components to this vector in place.
     *
     * @param vx the amount to increase the X component
     * @param vy the amount to increase the Y component
     * @param vz the amount to increase the Z component
     * @return this vector
     */
    public final Vec3d addLocal( double vx, double vy, double vz ) {
        x += vx;
        y += vy;
        z += vz;
        return this;
    }

    /**
     * Subtract the specified vector from this vector in place.
     *
     * It IS safe for v and this to be the same object.
     *
     * @param v the vector to subtract (not null, unaffected unless it's this)
     * @return this vector
     */
    public final Vec3d subtractLocal( Vec3d v ) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return this;
    }

    /**
     * Uniformly scale this vector in place.
     * 
     * @param s the scale factor
     * @return this
     */
    public final Vec3d multLocal( double s ) {
        x *= s;
        y *= s;
        z *= s;
        return this;
    }

    /**
     * Scale this vector by the specified vector component-by-component, in
     * place.
     *
     * It IS safe for v and this to be the same object.
     *
     * @param v the vector to multiply (not null, unaffected unless it's this)
     * @return this vector
     */
    public final Vec3d multLocal( Vec3d v ) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return this;
    }

    /**
     * Determine the squared length of this vector.
     *
     * @return the squared length (&ge;0)
     */
    public final double lengthSq() {
        return x * x + y * y + z * z;
    }
    
    /**
     * Determine the length of this vector.
     *
     * @return the length (&ge;0)
     */
    public final double length() {
        return Math.sqrt(lengthSq());                
    }

    /**
     * Normalize this vector in place.
     *
     * @return this
     */
    public final Vec3d normalizeLocal() {
        return multLocal(1.0 / length());
    }
 
    /**
     * Determine the dot product of the specified vector with this vector.
     *
     * @param v the vector to multiply (not null, unaffected)
     * @return the dot product
     */
    public final double dot( Vec3d v ) {
        return x * v.x + y * v.y + z * v.z;
    }
 
    /**
     * Take the cross product of this vector times the specified vector to yield
     * a new vector.
     *
     * @param v the right factor (not null, unaffected)
     * @return a new instance
     */
    public final Vec3d cross( Vec3d v ) {
        double xNew = (y * v.z) - (z * v.y);
        double yNew = (z * v.x) - (x * v.z);
        double zNew = (x * v.y) - (y * v.x);
        return new Vec3d(xNew, yNew, zNew);         
    }

    /**
     * Represent this vector as a String.
     *
     * The format is:
     *
     * Vec3d[XX.XXXXXXXXXXXXX, YY.YYYYYYYYYYYYY, ZZ.ZZZZZZZZZZZZZ]
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        return "Vec3d[" + x + ", " + y + ", " + z + "]";
    }
}
