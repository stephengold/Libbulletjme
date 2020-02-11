/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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
package com.jme3.bullet.joints;

import com.jme3.bullet.objects.PhysicsBody;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * The abstract base class for physics joints based on Bullet's
 * btTypedConstraint, btSoftBody::Anchor, or btSoftBody::Joint.
 *
 * @author normenhansen
 */
abstract public class PhysicsJoint implements Comparable<PhysicsJoint> {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsJoint.class.getName());
    // *************************************************************************
    // fields

    /**
     * Unique identifier of the btTypedConstraint, btSoftBody::Anchor, or
     * btSoftBody::Joint. Subtype constructors are responsible for setting this
     * to a non-zero value. After that, the ID never changes.
     */
    private long objectId = 0L;
    /**
     * body A specified in the constructor, or null for a single-ended joint
     * with body B only
     */
    private PhysicsBody bodyA = null;
    /**
     * body B specified in the constructor, or null for a single-ended joint
     * with body A only
     */
    private PhysicsBody bodyB = null;
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many ends this joint has.
     *
     * @return 1 if single-ended, 2 if double-ended
     */
    public int countEnds() {
        if (bodyA == null || bodyB == null) {
            return 1;
        } else {
            return 2;
        }
    }

    /**
     * Remove this joint from the joint lists of both connected bodies.
     */
    public void destroy() {
        if (bodyA != null) {
            bodyA.removeJoint(this);
        }
        if (bodyB != null) {
            bodyB.removeJoint(this);
        }
    }

    /**
     * Access the body at the specified end of this joint.
     *
     * @param end which end of the joint to access (not null)
     * @return the pre-existing body, or null if none
     */
    public PhysicsBody getBody(JointEnd end) {
        Validate.nonNull(end, "end");

        switch (end) {
            case A:
                return bodyA;
            case B:
                return bodyB;
            default:
                throw new IllegalArgumentException("end = " + end);
        }
    }

    /**
     * Access the body at the joint's "A" end.
     *
     * @return the pre-existing body, or null if none
     */
    public PhysicsBody getBodyA() {
        return bodyA;
    }

    /**
     * Access the body at the joint's "B" end.
     *
     * @return the pre-existing body, or null if none
     */
    public PhysicsBody getBodyB() {
        return bodyB;
    }

    /**
     * Read the ID of the btTypedConstraint, btSoftBody::Anchor, or
     * btSoftBody::Joint.
     *
     * @return the unique identifier (not zero)
     */
    final public long getObjectId() {
        assert objectId != 0L;
        return objectId;
    }

    /**
     * Test whether this joint is enabled.
     *
     * @return true if enabled, otherwise false
     */
    abstract public boolean isEnabled();
    // *************************************************************************
    // new protected methods

    /**
     * Specify the body at the joint's "A" end.
     *
     * @param body the desired body (not null, alias created)
     */
    final protected void setBodyA(PhysicsBody body) {
        assert body != null;
        assert bodyA == null : bodyA;
        bodyA = body;
    }

    /**
     * Specify the body at the joint's "B" end.
     *
     * @param body the desired body (not null, alias created)
     */
    final protected void setBodyB(PhysicsBody body) {
        assert body != null;
        assert bodyB == null : bodyB;
        bodyB = body;
    }

    /**
     * Initialize the native ID.
     *
     * @param jointId the unique identifier of the btTypedConstraint,
     * btSoftBody::Anchor, or btSoftBody::Joint (not zero)
     */
    protected void setNativeId(long jointId) {
        assert objectId == 0L : objectId;
        assert jointId != 0L;

        objectId = jointId;
        logger.log(Level.FINE, "Created {0}.", this);
    }
    // *************************************************************************
    // Comparable methods

    /**
     * Compare (by ID) with another joint.
     *
     * @param otherJoint (not null, unaffected)
     * @return 0 if this joint equals other joint; negative if this comes before
     * otherJoint; positive if this comes after otherJoint
     */
    @Override
    public int compareTo(PhysicsJoint otherJoint) {
        long otherId = otherJoint.getObjectId();
        int result = Long.compare(objectId, otherId);

        return result;
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for ID equality.
     *
     * @param otherObject (may be null)
     * @return true if the joints have the same ID, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (this == otherObject) {
            result = true;
        } else if (otherObject instanceof PhysicsJoint) {
            PhysicsJoint other = (PhysicsJoint) otherObject;
            long otherId = other.getObjectId();
            result = (objectId == otherId);
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this joint.
     *
     * @return value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = (int) (objectId >> 4);
        return hash;
    }

    /**
     * Represent this joint as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = getClass().getSimpleName();
        result = result.replace("Joint", "");
        result = result.replace("Physics", "");
        result = result.replace("Point", "P");
        result = result.replace("Six", "6");
        result += "#" + Long.toHexString(objectId);

        return result;
    }
}
