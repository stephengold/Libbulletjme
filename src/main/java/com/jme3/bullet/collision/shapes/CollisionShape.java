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
package com.jme3.bullet.collision.shapes;

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * The abstract base class for collision shapes based on Bullet's
 * btCollisionShape.
 * <p>
 * Subclasses include ConvexShape and MeshCollisionShape. As suggested in the
 * Bullet manual, a single CollisionShape can be shared among multiple collision
 * objects.
 *
 * @author normenhansen
 */
abstract public class CollisionShape
        extends NativePhysicsObject
        implements Comparable<CollisionShape> {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CollisionShape.class.getName());
    /**
     * local copy of {@link com.jme3.math.Transform#IDENTITY}
     */
    final private static Transform transformIdentity = new Transform();
    // *************************************************************************
    // fields

    /**
     * default margin for new non-sphere/non-capsule shapes (in physics-space
     * units, &gt;0, default=0.04)
     */
    private static float defaultMargin = 0.04f;
    /**
     * copy of collision margin (in physics-space units, &gt;0, default=0.04)
     */
    protected float margin = defaultMargin;
    /**
     * copy of the scale factors, one for each local axis (default=(1,1,1))
     */
    protected Vector3f scale = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // new methods exposed

    /**
     * Calculate an axis-aligned bounding box for this shape with the specified
     * translation and rotation applied to it. Rotation is applied first.
     * Collision margin is included.
     *
     * @param translation the translation to apply (not null, unaffected)
     * @param rotation the rotation to apply (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a bounding box (either storeResult or a new instance, not null)
     */
    public BoundingBox boundingBox(Vector3f translation, Matrix3f rotation,
            BoundingBox storeResult) {
        Validate.finite(translation, "translation");
        Validate.nonNull(rotation, "rotation");
        BoundingBox result
                = (storeResult == null) ? new BoundingBox() : storeResult;

        recalculateAabb();

        long shapeId = nativeId();
        Vector3f maxima = new Vector3f();
        Vector3f minima = new Vector3f();
        getAabb(shapeId, translation, rotation, minima, maxima);
        result.setMinMax(minima, maxima);

        return result;
    }

    /**
     * Calculate an axis-aligned bounding box for this shape with the specified
     * translation and rotation applied to it. Rotation is applied first.
     * Collision margin is included.
     *
     * @param translation the translation to apply (not null, unaffected)
     * @param rotation the rotation to apply (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a bounding box (either storeResult or a new instance, not null)
     */
    public BoundingBox boundingBox(Vector3f translation, Quaternion rotation,
            BoundingBox storeResult) {
        Validate.finite(translation, "translation");
        Validate.nonNull(rotation, "rotation");
        BoundingBox result
                = (storeResult == null) ? new BoundingBox() : storeResult;

        recalculateAabb();

        long shapeId = nativeId();
        Matrix3f basisMatrix = new Matrix3f();
        basisMatrix.set(rotation);
        Vector3f maxima = new Vector3f();
        Vector3f minima = new Vector3f();
        getAabb(shapeId, translation, basisMatrix, minima, maxima);
        result.setMinMax(minima, maxima);

        return result;
    }

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * Subclasses that restrict scaling should override this method.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    public boolean canScale(Vector3f scale) {
        boolean result;
        if (scale == null) {
            result = false;
        } else {
            result = MyVector3f.isAllNonNegative(scale);
        }

        return result;
    }

    /**
     * Read the default margin for new shapes that are neither capsules nor
     * spheres.
     *
     * @return the margin distance (in physics-space units, &gt;0)
     */
    public static float getDefaultMargin() {
        assert defaultMargin > 0f : defaultMargin;
        return defaultMargin;
    }

    /**
     * Read the collision margin for this shape.
     *
     * @return the margin distance (in physics-space units, &ge;0)
     */
    public float getMargin() {
        assert margin > 0f : margin;
        assert margin == getMargin(nativeId());
        return margin;
    }

    /**
     * Read the native ID of the btCollisionShape.
     *
     * @return the identifier (not zero)
     * @deprecated use nativeId()
     */
    @Deprecated
    final public long getObjectId() {
        long shapeId = nativeId();
        return shapeId;
    }

    /**
     * Copy the scale factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scale factor for each local axis (either storeResult or a new
     * vector, not null, no negative component)
     */
    public Vector3f getScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        assert checkScale(result);
        result.set(scale);

        return result;
    }

    /**
     * Test whether this shape has concave type. In Bullet, "concave" is a
     * property of <em>types</em> of shapes. Specific <em>instances</em> of
     * those types might actually be "convex" in the mathematical sense of the
     * word.
     * <p>
     * The only concave types are the empty, gimpact, heightfield, mesh, and
     * plane shapes. Note that compound shapes are neither concave nor convex.
     *
     * @return true if concave type, false otherwise
     */
    public boolean isConcave() {
        long shapeId = nativeId();
        boolean result = isConcave(shapeId);

        return result;
    }

    /**
     * Test whether this shape has convex type. In Bullet, "convex" is a
     * property of <em>types</em> of shapes. Specific <em>instances</em> of
     * non-convex types might still be "convex" in the mathematical sense of the
     * word.
     * <p>
     * The only convex types are the box2d, box, capsule, cone, convex2d,
     * cylinder, hull, multi-sphere, simplex, and sphere shapes. Note that
     * compound shapes are neither concave nor convex.
     *
     * @return true if convex type, false otherwise
     */
    public boolean isConvex() {
        long shapeId = nativeId();
        boolean result = isConvex(shapeId);

        return result;
    }

    /**
     * Test whether this shape's type is infinite. PlaneCollisionShape is the
     * only type of shape that's infinite.
     *
     * @return true if infinite, false otherwise
     */
    public boolean isInfinite() {
        long shapeId = nativeId();
        boolean result = isInfinite(shapeId);

        return result;
    }

    /**
     * Test whether this shape can be applied to a dynamic rigid body. The only
     * non-moving shapes are the empty, heightfield, mesh, and plane shapes.
     *
     * @return true if non-moving, false otherwise
     */
    public boolean isNonMoving() {
        long shapeId = nativeId();
        boolean result = isNonMoving(shapeId);

        return result;
    }

    /**
     * Test whether this shape is convex and defined by polygons. The only
     * polyhedral shapes are the box, hull, and simplex shapes.
     *
     * @return true if polyhedral, false otherwise
     */
    public boolean isPolyhedral() {
        long shapeId = nativeId();
        boolean result = isPolyhedral(shapeId);

        return result;
    }

    /**
     * Estimate how far this shape extends from its center.
     *
     * @return a distance estimate (in physics-space units, &ge;0, may be
     * infinite)
     */
    public float maxRadius() {
        float result = DebugShapeFactory.maxDistance(this, transformIdentity,
                DebugShapeFactory.lowResolution);
        return result;
    }

    /**
     * Alter the default margin for new shapes that are neither capsules nor
     * spheres.
     * <i>From Bullet manual:</i><br>
     * It is best not to modify the default collision margin, and if you do use
     * a positive value: zero margin might introduce problems.
     *
     * @param margin the desired margin distance (in physics-space units, &gt;0,
     * default=0.04)
     */
    public static void setDefaultMargin(float margin) {
        Validate.positive(margin, "margin");
        defaultMargin = margin;
    }

    /**
     * Alter the collision margin of this shape. CAUTION: Margin is applied
     * differently, depending on the type of shape. Generally the collision
     * margin expands the object, creating a gap.
     * <p>
     * <i>From Bullet manual:</i><br>
     * It is best not to modify the default collision margin, and if you do use
     * a positive value: zero margin might introduce problems.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) changes can have unintended consequences.
     *
     * @param margin the desired margin distance (in physics-space units, &gt;0,
     * default=0.04)
     */
    public void setMargin(float margin) {
        Validate.positive(margin, "margin");

        long shapeId = nativeId();
        setMargin(shapeId, margin);
        logger.log(Level.FINE, "Margining {0}.", this);
        this.margin = margin;
    }

    /**
     * Alter the scale of this shape to a uniform factor. CAUTION: Not all
     * shapes can be scaled.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) changes can have unintended consequences.
     *
     * @param factor the desired scale factor for all axes (&ge;0, default=1)
     */
    public void setScale(float factor) {
        Validate.nonNegative(factor, "factor");

        Vector3f scaleVector
                = new Vector3f(factor, factor, factor); // TODO garbage
        setScale(scaleVector);
    }

    /**
     * Alter the scale of this shape. CAUTION: Not all shapes can be scaled
     * arbitrarily.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) changes can have unintended consequences.
     *
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public void setScale(Vector3f scale) {
        Validate.nonNegative(scale, "scale");
        if (!canScale(scale)) {
            String typeName = getClass().getCanonicalName();
            String msg = String.format("%s cannot be scaled to (%s,%s,%s)",
                    typeName, scale.x, scale.y, scale.z);
            throw new IllegalArgumentException(msg);
        }

        long shapeId = nativeId();
        setLocalScaling(shapeId, scale);
        logger.log(Level.FINE, "Scaling {0}.", this);
        this.scale.set(scale);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Read the type of this shape.
     *
     * @param shapeId the ID of the btCollisionShape (not zero)
     * @return the type value (from enum BroadphaseNativeTypes)
     */
    final native protected int getShapeType(long shapeId);

    /**
     * Recalculate this shape's bounding box if necessary. Meant to be
     * overridden.
     */
    protected void recalculateAabb() {
        // do nothing
    }

    /**
     * Synchronize the copied scale factors with the btCollisionShape.
     */
    protected void updateScale() {
        long shapeId = nativeId();
        getLocalScaling(shapeId, scale);
    }
    // *************************************************************************
    // Comparable methods

    /**
     * Compare (by ID) with the specified collision shape.
     *
     * @param other the other shape (not null, unaffected)
     * @return 0 if the shapes have the same ID; negative if this comes before
     * other; positive if this comes after other
     */
    @Override
    public int compareTo(CollisionShape other) {
        long thisId = nativeId();
        long otherId = other.nativeId();
        int result = Long.compare(thisId, otherId);

        return result;
    }
    // *************************************************************************
    // NativePhysicsObject methods

    /**
     * Finalize this shape just before it is destroyed. Should be invoked only
     * by a subclass or by the garbage collector.
     *
     * @throws Throwable ignored by the garbage collector
     */
    @Override
    protected void finalize() throws Throwable {
        try {
            logger.log(Level.FINE, "Finalizing {0}.", this);
            long shapeId = nativeId();
            finalizeNative(shapeId);
        } finally {
            super.finalize();
        }
    }

    /**
     * Initialize the native ID.
     *
     * @param shapeId the identifier of the btCollisionShape (not zero)
     */
    @Override
    protected void setNativeId(long shapeId) {
        super.setNativeId(shapeId);
        logger.log(Level.FINE, "Created {0}.", this);
    }
    // *************************************************************************
    // private methods

    /**
     * Compare Bullet's scale factors to the local copies.
     *
     * @param storeVector caller-allocated temporary storage (not null)
     * @return true if Bullet and the local copy match exactly, otherwise false
     */
    private boolean checkScale(Vector3f storeVector) {
        assert storeVector != null;

        long shapeId = nativeId();
        getLocalScaling(shapeId, storeVector);
        boolean result = scale.equals(storeVector);
        if (!result) {
            logger.log(Level.WARNING,
                    "mismatch detected: shape={0} copy={1} native={2}",
                    new Object[]{this, scale, storeVector});
        }

        return result;
    }
    // *************************************************************************
    // native private methods

    native private void finalizeNative(long shapeId);

    native private void getAabb(long shapeId, Vector3f location,
            Matrix3f basisMatrix, Vector3f storeMinima, Vector3f storeMaxima);

    native private void getLocalScaling(long shapeId, Vector3f storeVector);

    native private float getMargin(long shapeId);

    native private boolean isConcave(long shapeId);

    native private boolean isConvex(long shapeId);

    native private boolean isInfinite(long shapeId);

    native private boolean isNonMoving(long shapeId);

    native private boolean isPolyhedral(long shapeId);

    native private void setLocalScaling(long shapeId, Vector3f scale);

    native private void setMargin(long shapeId, float margin);
}
