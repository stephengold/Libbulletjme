/*
 * Copyright (c) 2020 jMonkeyEngine
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
package com.jme3.bullet.objects;

import com.jme3.bullet.MultiBody;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision object for a link or base in a MultiBody, based on Bullet's
 * btMultiBodyLinkCollider.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MultiBodyCollider extends PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MultiBodyCollider.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collider for the indexed link in the specified MultiBody.
     * Used internally.
     *
     * @param multiBody (not null, alias created)
     * @param linkIndex the link index, or -1 for the base
     */
    public MultiBodyCollider(MultiBody multiBody, int linkIndex) {
        assert objectId == 0L : objectId;
        long multiBodyId = multiBody.nativeId();
        objectId = createCollider(multiBodyId, linkIndex);
        logger2.log(Level.FINE, "Created {0}.", this);
        assert objectId != 0L;
        assert getInternalType(objectId) == 64 : getInternalType(objectId);

        super.initUserPointer();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Attach the specified collision shape to this collider. Used internally.
     *
     * @param shape the shape to attach (not null, alias created)
     */
    public void attachShape(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        setCollisionShape(shape);
        long shapeId = shape.getObjectId();
        super.attachCollisionShape(objectId, shapeId);
    }
    // *************************************************************************
    // native methods

    native private long createCollider(long multiBodyId, int linkIndex);
}
