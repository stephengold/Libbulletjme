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
 *
 *  @version   $Revision: 3951 $
 *  @author    Paul Speed
 */
public final class Quatd implements Cloneable {
    /**
     * X component of the Quatd
     */
    public double x;
    /**
     * Y component of the Quatd
     */
    public double y;
    /**
     * Z component of the Quatd
     */
    public double z;
    /**
     * W component of the Quatd
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
     * Create a copy of this Quatd.
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
        if (Double.compare(x, other.x) != 0) {
            return false;
        }
        if (Double.compare(y, other.y) != 0) {
            return false;
        }
        if (Double.compare(z, other.z) != 0) {
            return false;
        }
        if (Double.compare(w, other.w) != 0) {
            return false;
        }
        return true;
    }
    
    /**
     * Set all components of this Quatd to the specified values.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     * @param w the desired W component
     * @return this vector
     */
    public final Quatd set( double x, double y, double z, double w ) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;        
    } 

    /**
     * Copy all components of the specified Quatd to this Quatd.
     *
     * @param q the desired value (not null, unaffected)
     * @return this Quatd
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
     * @return this vector
     */
    public final Quatd set( Quaternion quat ) {
        this.x = quat.getX();
        this.y = quat.getY();
        this.z = quat.getZ();
        this.w = quat.getW();
        return this;
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
     * @return this
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
