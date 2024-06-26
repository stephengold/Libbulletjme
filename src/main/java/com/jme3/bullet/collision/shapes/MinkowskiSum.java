/*
 * Copyright (c) 2023-2024 jMonkeyEngine
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
package com.jme3.bullet.collision.shapes;

import com.jme3.math.Vector3f;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A convex collision shape to represent the Minkowki sum of 2 convex shapes,
 * based on Bullet's {@code btMinkowskiSumShape}.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MinkowskiSum extends ConvexShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(MinkowskiSum.class.getName());
    // *************************************************************************
    // fields

    /**
     * first shape on which this shape is based
     */
    final private ConvexShape shapeA;
    /**
     * 2nd shape on which this shape is based
     */
    final private ConvexShape shapeB;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the Minkowki sum of the specified shapes.
     *
     * @param shapeA the first base shape (not null, convex, alias created)
     * @param shapeB the 2nd base shape (not null, convex, alias created)
     */
    public MinkowskiSum(ConvexShape shapeA, ConvexShape shapeB) {
        Validate.nonNull(shapeA, "shape A");
        Validate.nonNull(shapeA, "shape B");

        this.shapeA = shapeA;
        this.shapeB = shapeB;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the first base shape.
     *
     * @return the pre-existing shape (not null)
     */
    public ConvexShape getShapeA() {
        assert shapeA != null;
        return shapeA;
    }

    /**
     * Access the 2nd base shape.
     *
     * @return the pre-existing shape (not null)
     */
    public ConvexShape getShapeB() {
        assert shapeB != null;
        return shapeB;
    }
    // *************************************************************************
    // ConvexShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For MinkowskiSum shapes, scaling must be unity.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale = super.canScale(scale)
                && MyVector3f.isScaleIdentity(scale);
        return canScale;
    }

    /**
     * Return the collision margin of this shape.
     *
     * @return the margin thickness (in physics-space units, &ge;0)
     */
    @Override
    public float getMargin() {
        this.margin = shapeA.nativeMargin() + shapeB.nativeMargin();
        float result = super.getMargin();

        return result;
    }

    /**
     * Alter the collision margin of this shape. This feature is disabled for
     * MinkowskiSum shapes. The margin of a MinkowskiSum is simply the sum of
     * the (native) margins of the base shapes.
     *
     * @param margin the desired margin thickness (in physics-space units)
     */
    @Override
    public void setMargin(float margin) {
        logger2.log(Level.WARNING,
                "Cannot directly alter the margin of a MinkowskiSum");
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate a {@code btMinkowskiSumShape}.
     */
    private void createShape() {
        long shapeAId = shapeA.nativeId();
        long shapeBId = shapeB.nativeId();
        long shapeId = createShape(shapeAId, shapeBId);
        setNativeId(shapeId);

        setContactFilterEnabled(enableContactFilter);
        setScale(scale);
        this.margin = shapeA.getMargin() + shapeB.getMargin();
    }
    // *************************************************************************
    // native private methods

    native private static long createShape(long shapeAId, long shapeBId);
}
