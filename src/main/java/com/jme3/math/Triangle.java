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

import jme3utilities.math.MyVector3f;

/**
 * Describes a triangle in terms of its vertex locations, with auxiliary storage
 * for its normal vector.
 *
 * @author Mark Powell
 * @author Joshua Slack
 */
public class Triangle {
    /**
     * The location of the first vertex in winding order.
     */
    final private Vector3f pointA = new Vector3f();
    /**
     * The location of the 2nd vertex in winding order.
     */
    final private Vector3f pointB = new Vector3f();
    /**
     * The location of the 3rd vertex in winding order.
     */
    final private Vector3f pointC = new Vector3f();
    private transient Vector3f normal;

    /**
     * Instantiate a zero-size triangle at the origin.
     */
    public Triangle() {
    }

    /**
     * Instantiates a triangle with the specified vertex locations. Vertices
     * should be listed in the desired winding order, typically
     * counter-clockwise.
     *
     * @param p1 the location of the first vertex (not null, unaffected)
     * @param p2 the location of the 2nd vertex (not null, unaffected)
     * @param p3 the location of the 3rd vertex (not null, unaffected)
     */
    public Triangle(Vector3f p1, Vector3f p2, Vector3f p3) {
        pointA.set(p1);
        pointB.set(p2);
        pointC.set(p3);
    }

    /**
     * Accesses the location of the indexed vertex.
     *
     * @param i the index of the vertex to access (0, 1, or 2)
     * @return a pre-existing location vector, or null if the index is invalid
     */
    public Vector3f get(int i) {
        switch (i) {
            case 0:
                return pointA;
            case 1:
                return pointB;
            case 2:
                return pointC;
            default:
                return null;
        }
    }

    /**
     * Accesses the location of the first vertex.
     *
     * @return the pre-existing location vector (not null)
     */
    public Vector3f get1() {
        return pointA;
    }

    /**
     * Accesses the location of the 2nd vertex.
     *
     * @return the pre-existing location vector (not null)
     */
    public Vector3f get2() {
        return pointB;
    }

    /**
     * Accesses the location of the 3rd vertex.
     *
     * @return the pre-existing location vector (not null)
     */
    public Vector3f get3() {
        return pointC;
    }

    /**
     * Alters the location of the indexed vertex and deletes the stored normal.
     *
     * @param i the index of the vertex to alter (0, 1, or 2)
     * @param point the desired location (not null, unaffected)
     */
    public void set(int i, Vector3f point) {
        normal = null;

        switch (i) {
            case 0:
                pointA.set(point);
                break;
            case 1:
                pointB.set(point);
                break;
            case 2:
                pointC.set(point);
                break;
        }
    }

    /**
     * Alters the location of the first vertex and deletes the stored normal.
     *
     * @param v the desired location (not null, unaffected)
     */
    public void set1(Vector3f v) {
        normal = null;

        pointA.set(v);
    }

    /**
     * Alters the location of the 2nd vertex and deletes the stored normal.
     *
     * @param v the desired location (not null, unaffected)
     */
    public void set2(Vector3f v) {
        normal = null;

        pointB.set(v);
    }

    /**
     * Alters the location of the 3rd vertex and deletes the stored normal.
     *
     * @param v the desired location (not null, unaffected)
     */
    public void set3(Vector3f v) {
        normal = null;

        pointC.set(v);
    }

    /**
     * Alters the locations of all 3 vertices and deletes the stored normal.
     *
     * @param v1 the desired location of the first vertex (not null, unaffected)
     * @param v2 the desired location of the 2nd vertex (not null, unaffected)
     * @param v3 the desired location of the 3rd vertex (not null, unaffected)
     */
    public void set(Vector3f v1, Vector3f v2, Vector3f v3) {
        normal = null;

        pointA.set(v1);
        pointB.set(v2);
        pointC.set(v3);
    }

    /**
     * Recalculates the stored normal based on the current vertex locations.
     */
    public void calculateNormal() {
        if (normal == null) {
            normal = new Vector3f(pointB);
        } else {
            normal.set(pointB);
        }
        normal.subtractLocal(pointA).crossLocal(pointC.x - pointA.x, pointC.y - pointA.y, pointC.z - pointA.z);
        MyVector3f.normalizeLocal(normal);
    }

    /**
     * Accesses the stored normal, updating it if it is null.
     *
     * @return unit normal vector (an internal vector subject to re-use)
     */
    public Vector3f getNormal() {
        if (normal == null) {
            calculateNormal();
        }
        return normal;
    }
}
