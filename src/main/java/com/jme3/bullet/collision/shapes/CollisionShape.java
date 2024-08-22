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
import com.simsilica.mathd.Vec3d;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jme3utilities.minie.MyShape;

/**
 * The abstract base class for collision shapes based on Bullet's
 * {@code btCollisionShape}.
 * <p>
 * Subclasses include {@code ConvexShape} and {@code MeshCollisionShape}. As
 * suggested in the Bullet manual, a single collision shape can be shared among
 * multiple collision objects.
 *
 * @author normenhansen
 */
abstract public class CollisionShape extends NativePhysicsObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CollisionShape.class.getName());
    /**
     * local copy of {@link com.jme3.math.Quaternion#IDENTITY}
     */
    final private static Quaternion rotateIdentity = new Quaternion();
    /**
     * local copy of {@link com.jme3.math.Transform#IDENTITY}
     */
    final private static Transform transformIdentity = new Transform();
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * copy of the contact-filter enable flag
     */
    protected boolean enableContactFilter = false;
    /**
     * default margin for new non-sphere/non-capsule shapes (in physics-space
     * units, &gt;0)
     */
    private static float defaultMargin = 0.04f;
    /**
     * copy of collision margin (in physics-space units, &gt;0, default=0.04)
     */
    protected float margin = defaultMargin;
    /**
     * copy of the scale factors, one for each local axis
     */
    protected Vector3f scale = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collision shape with no tracker and no assigned native
     * object.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    protected CollisionShape() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the center of the shape's axis-aligned bounding box in local
     * coordinates.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location in the local coordinate system (either
     * {@code storeResult} or a new vector)
     */
    public Vector3f aabbCenter(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        BoundingBox aabb = boundingBox(translateIdentity, rotateIdentity, null);
        aabb.getCenter(result);

        return result;
    }

    /**
     * Calculate an axis-aligned bounding box with the specified translation and
     * rotation applied. Rotation is applied first. Collision margin is
     * included.
     *
     * @param translation the translation to apply (not null, unaffected)
     * @param rotation the rotation to apply (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a bounding box (either storeResult or a new instance, not null)
     */
    public BoundingBox boundingBox(
            Vector3f translation, Matrix3f rotation, BoundingBox storeResult) {
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
     * Calculate an axis-aligned bounding box with the specified translation and
     * rotation applied. Rotation is applied first. Collision margin is
     * included.
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
        Matrix3f basisMatrix = new Matrix3f().set(rotation);
        Vector3f maxima = new Vector3f();
        Vector3f minima = new Vector3f();
        getAabb(shapeId, translation, basisMatrix, minima, maxima);
        result.setMinMax(minima, maxima);

        return result;
    }

    /**
     * Test whether the specified scale factors can be applied to the shape.
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
     * Test whether the shape can be split by an arbitrary plane. Meant to be
     * overridden.
     *
     * @return true if splittable, false otherwise
     */
    public boolean canSplit() {
        return false;
    }

    /**
     * Return the default margin for new shapes that are neither capsules nor
     * spheres.
     *
     * @return the margin thickness (in physics-space units, &gt;0)
     */
    public static float getDefaultMargin() {
        assert defaultMargin > 0f : defaultMargin;
        return defaultMargin;
    }

    /**
     * Return the (copied) collision margin of the shape.
     *
     * @return the margin thickness (in physics-space units, &ge;0)
     */
    public float getMargin() {
        assert margin >= 0f : margin;
        assert margin == nativeMargin() : margin + " != " + nativeMargin();
        return margin;
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
     * Copy the scale factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scale factor for each local axis (either {@code storeResult}
     * or a new vector, not null, no negative component)
     */
    public Vec3d getScaleDp(Vec3d storeResult) {
        long shapeId = nativeId();
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;
        getLocalScalingDp(shapeId, result);

        return result;
    }

    /**
     * Return the encoded type of the shape.
     *
     * @return the type code (from Bullet's {@code enum BroadphaseNativeTypes})
     */
    public int getShapeType() {
        long shapeId = nativeId();
        int result = getShapeType(shapeId);

        return result;
    }

    /**
     * Test whether the shape has concave type. In Bullet, "concave" is a
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
     * Test whether contact filtering is enabled for this shape. Contact
     * filtering is implemented only for HeightfieldCollisionShape and
     * MeshCollisionShapes.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isContactFilterEnabled() {
        assert enableContactFilter == isContactFilterEnabled(nativeId()) :
                "copy of flag = " + enableContactFilter;
        return enableContactFilter;
    }

    /**
     * Test whether the shape has convex type. In Bullet, "convex" is a property
     * of <em>types</em> of shapes. Specific <em>instances</em> of non-convex
     * types might still be "convex" in the mathematical sense of the word.
     * <p>
     * Convex types include the box2d, box, capsule, cone, convex2d, cylinder,
     * hull, multi-sphere, simplex, and sphere shapes. Note that compound shapes
     * are neither concave nor convex.
     *
     * @return true if convex type, false otherwise
     */
    public boolean isConvex() {
        long shapeId = nativeId();
        boolean result = isConvex(shapeId);

        return result;
    }

    /**
     * Test whether the shape's type is infinite. {@code PlaneCollisionShape} is
     * the only type of shape that's infinite.
     *
     * @return true if infinite, false otherwise
     */
    public boolean isInfinite() {
        long shapeId = nativeId();
        boolean result = isInfinite(shapeId);

        return result;
    }

    /**
     * Test whether the shape can be applied to a dynamic rigid body. The only
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
     * Estimate how far the scaled shape extends from its center, including
     * margin.
     *
     * @return a distance estimate (in physics-space units, &ge;0, may be
     * infinite)
     */
    public float maxRadius() {
        float result = DebugShapeFactory.maxDistance(
                this, transformIdentity, DebugShapeFactory.lowResolution);
        return result;
    }

    /**
     * Estimate the volume of this shape, including scale and margin. Meant to
     * be overridden.
     *
     * @return the volume (in physics-space units cubed, &ge;0)
     */
    public float scaledVolume() {
        throw new UnsupportedOperationException("Not implemented for: " + this);
    }

    /**
     * Enable/disable contact filtering for this shape.
     *
     * @param setting the desired setting (default=false)
     */
    public void setContactFilterEnabled(boolean setting) {
        long shapeId = nativeId();
        setContactFilterEnabled(shapeId, setting);
        this.enableContactFilter = setting;
    }

    /**
     * Alter the default margin for new shapes that are neither capsules nor
     * spheres.
     * <i>From Bullet manual:</i><br>
     * It is best not to modify the default collision margin, and if you do use
     * a positive value: zero margin might introduce problems.
     *
     * @param margin the desired margin thickness (in physics-space units,
     * &gt;0, default=0.04)
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
     * @param margin the desired margin thickness (in physics-space units,
     * &gt;0, default=0.04)
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
        assert Validate.nonNegative(factor, "factor");

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
        if (!canScale(scale)) {
            String typeName = getClass().getCanonicalName();
            String message = String.format("%s cannot be scaled to (%s,%s,%s)",
                    typeName, scale.x, scale.y, scale.z);
            throw new IllegalArgumentException(message);
        }

        long shapeId = nativeId();
        setLocalScaling(shapeId, scale);
        logger.log(Level.FINE, "Scaling {0}.", this);
        this.scale.set(scale);
    }

    /**
     * Alter the primary user index. Applications may use this parameter for any
     * purpose (native field: m_userIndex).
     *
     * @param index the desired value (default=-1)
     */
    public void setUserIndex(int index) {
        long shapeId = nativeId();
        setUserIndex(shapeId, index);
    }

    /**
     * Alter the secondary user index. Applications may use this parameter for
     * any purpose (native field: m_userIndex2).
     *
     * @param index the desired value (default=-1)
     */
    public void setUserIndex2(int index) {
        long shapeId = nativeId();
        setUserIndex2(shapeId, index);
    }

    /**
     * Approximate this shape with a splittable shape. Meant to be overridden.
     *
     * @return a new splittable shape
     */
    public CollisionShape toSplittableShape() {
        if (canSplit()) {
            return this;
        } else {
            throw new IllegalArgumentException("this = " + this);
        }
    }

    /**
     * Return the primary user index (native field: m_userIndex).
     *
     * @return the value
     */
    public int userIndex() {
        long shapeId = nativeId();
        int result = getUserIndex(shapeId);
        return result;
    }

    /**
     * Return the secondary user index (native field: m_userIndex2).
     *
     * @return the value
     */
    public int userIndex2() {
        long shapeId = nativeId();
        int result = getUserIndex2(shapeId);
        return result;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Return the type of this shape.
     *
     * @param shapeId the ID of the {@code btCollisionShape} (not zero)
     * @return the type value (from Bullet's {@code enum BroadphaseNativeTypes})
     */
    final native protected static int getShapeType(long shapeId);

    /**
     * Return the native collision margin of this shape.
     *
     * @return the margin thickness (in physics-space units, &ge;0)
     */
    final protected float nativeMargin() {
        long shapeId = nativeId();
        float result = getMargin(shapeId);

        assert result >= 0f : result;
        return result;
    }

    /**
     * Recalculate this shape's bounding box if necessary. Meant to be
     * overridden.
     */
    protected void recalculateAabb() {
        // do nothing
    }

    /**
     * Synchronize the copied scale factors with the {@code btCollisionShape}.
     */
    protected void updateScale() {
        long shapeId = nativeId();
        getLocalScaling(shapeId, scale);
    }
    // *************************************************************************
    // NativePhysicsObject methods

    /**
     * Initialize the native ID.
     *
     * @param shapeId the identifier of the {@code btCollisionShape} (not zero)
     */
    @Override
    protected void setNativeId(long shapeId) {
        super.setNativeId(shapeId);
        logger.log(Level.FINE, "Created {0}.", this);
    }

    /**
     * Represent this CollisionShape as a String.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = MyShape.describeType(this);
        long shapeId = nativeId();
        result += "#" + Long.toHexString(shapeId);

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Compare Bullet's scale factors to the local copies.
     *
     * @param storeVector caller-allocated temporary storage (not null)
     * @return true if the scale factors match exactly, otherwise false
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

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param shapeId the native identifier (not zero)
     */
    private static void freeNativeObject(long shapeId) {
        assert shapeId != 0L;
        finalizeNative(shapeId);
    }
    // *************************************************************************
    // native private methods

    native private static void finalizeNative(long shapeId);

    native private static void getAabb(long shapeId, Vector3f location,
            Matrix3f basisMatrix, Vector3f storeMinima, Vector3f storeMaxima);

    native private static void
            getLocalScaling(long shapeId, Vector3f storeVector);

    native private static void
            getLocalScalingDp(long shapeId, Vec3d storeVector);

    native private static float getMargin(long shapeId);

    native private static int getUserIndex(long shapeId);

    native private static int getUserIndex2(long shapeId);

    native private static boolean isConcave(long shapeId);

    native private static boolean isContactFilterEnabled(long shapeId);

    native private static boolean isConvex(long shapeId);

    native private static boolean isInfinite(long shapeId);

    native private static boolean isNonMoving(long shapeId);

    native private static boolean isPolyhedral(long shapeId);

    native private static void
            setContactFilterEnabled(long shapeId, boolean setting);

    native private static void setLocalScaling(long shapeId, Vector3f scale);

    native private static void setMargin(long shapeId, float margin);

    native private static void setUserIndex(long shapeId, int index);

    native private static void setUserIndex2(long shapeId, int index);
}
