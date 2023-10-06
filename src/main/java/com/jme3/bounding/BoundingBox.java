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
package com.jme3.bounding;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;

/**
 * <code>BoundingBox</code> describes a bounding volume as an axis-aligned box.
 * <br>
 * Instances may be initialized by invoking the <code>containAABB</code> method.
 *
 * @author Joshua Slack
 * @version $Id: BoundingBox.java,v 1.50 2007/09/22 16:46:35 irrisor Exp $
 */
public class BoundingBox {
    final private Vector3f center = new Vector3f();
    /**
     * the X-extent of the box (>=0, may be +Infinity)
     */
    float xExtent;
    /**
     * the Y-extent of the box (>=0, may be +Infinity)
     */
    float yExtent;
    /**
     * the Z-extent of the box (>=0, may be +Infinity)
     */
    float zExtent;

    /**
     * Instantiate a <code>BoundingBox</code> without initializing it.
     */
    public BoundingBox() {
    }

    /**
     * Instantiate a <code>BoundingBox</code> with given center and extents.
     *
     * @param c the coordinates of the center of the box (not null, not altered)
     * @param x the X-extent of the box (0 or greater, may be +Infinity)
     * @param y the Y-extent of the box (0 or greater, may be +Infinity)
     * @param z the Z-extent of the box (0 or greater, may be +Infinity)
     */
    public BoundingBox(Vector3f c, float x, float y, float z) {
        this.center.set(c);
        this.xExtent = x;
        this.yExtent = y;
        this.zExtent = z;
    }

    /**
     * Instantiate a BoundingBox with the specified extremes.
     *
     * @param min the desired minimum coordinate value for each axis (not null,
     * not altered)
     * @param max the desired maximum coordinate value for each axis (not null,
     * not altered)
     */
    public BoundingBox(Vector3f min, Vector3f max) {
        setMinMax(min, max);
    }

    /**
     * Return the location of the center.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (either {@code storeResult} or a new vector)
     */
    public Vector3f getCenter(Vector3f storeResult) {
        if (storeResult == null) {
            return center.clone();
        } else {
            return storeResult.set(center);
        }
    }

    /**
     * Query extent.
     *
     * @param store
     *            where extent gets stored - null to return a new vector
     * @return store / new vector
     */
    public Vector3f getExtent(Vector3f store) {
        if (store == null) {
            store = new Vector3f();
        }
        store.set(xExtent, yExtent, zExtent);
        return store;
    }

    /**
     * Determine the X-axis distance between the center and the boundary.
     *
     * @return the distance
     */
    public float getXExtent() {
        return xExtent;
    }

    /**
     * Determine the Y-axis distance between the center and the boundary.
     *
     * @return the distance
     */
    public float getYExtent() {
        return yExtent;
    }

    /**
     * Determine the Z-axis distance between the center and the boundary.
     *
     * @return the distance
     */
    public float getZExtent() {
        return zExtent;
    }

    /**
     * Alter the X-axis distance between the center and the boundary.
     *
     * @param xExtent the desired distance (&ge;0)
     */
    public void setXExtent(float xExtent) {
        if (xExtent < 0) {
            throw new IllegalArgumentException();
        }

        this.xExtent = xExtent;
    }

    /**
     * Alter the Y-axis distance between the center and the boundary.
     *
     * @param yExtent the desired distance (&ge;0)
     */
    public void setYExtent(float yExtent) {
        if (yExtent < 0) {
            throw new IllegalArgumentException();
        }

        this.yExtent = yExtent;
    }

    /**
     * Alter the Z-axis distance between the center and the boundary.
     *
     * @param zExtent the desired distance (&ge;0)
     */
    public void setZExtent(float zExtent) {
        if (zExtent < 0) {
            throw new IllegalArgumentException();
        }

        this.zExtent = zExtent;
    }

    /**
     * Determine the minimum coordinate value for each axis.
     *
     * @param store storage for the result (modified if not null)
     * @return either storeResult or a new vector
     */
    public Vector3f getMin(Vector3f store) {
        if (store == null) {
            store = new Vector3f();
        }
        store.set(center).subtractLocal(xExtent, yExtent, zExtent);
        return store;
    }

    /**
     * Determine the maximum coordinate value for each axis.
     *
     * @param store storage for the result (modified if not null)
     * @return either storeResult or a new vector
     */
    public Vector3f getMax(Vector3f store) {
        if (store == null) {
            store = new Vector3f();
        }
        store.set(center).addLocal(xExtent, yExtent, zExtent);
        return store;
    }

    /**
     * Reconfigure with the specified extremes.
     *
     * @param min the desired minimum coordinate value for each axis (not null,
     * not altered)
     * @param max the desired maximum coordinate value for each axis (not null,
     * not altered)
     */
    public void setMinMax(Vector3f min, Vector3f max) {
        this.center.set(max).addLocal(min).multLocal(0.5f);
        xExtent = FastMath.abs(max.x - center.x);
        yExtent = FastMath.abs(max.y - center.y);
        zExtent = FastMath.abs(max.z - center.z);
    }
}
