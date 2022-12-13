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

import com.jme3.util.TempVars;
import java.util.logging.Logger;

/**
 * Used to efficiently represent rotations and orientations in 3-dimensional
 * space, without risk of gimbal lock. Each instance has 4 single-precision
 * components: 3 imaginary components (X, Y, and Z) and a real component (W).
 *
 * <p>Mathematically, quaternions are an extension of complex numbers. In
 * mathematics texts, W often appears first, but in JME it always comes last.
 *
 * @author Mark Powell
 * @author Joshua Slack
 */
public final class Quaternion implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;

    private static final Logger logger = Logger.getLogger(Quaternion.class.getName());
    /**
     * Shared instance of the identity quaternion (0, 0, 0, 1). Do not modify!
     *
     * <p>This is the usual representation for a null rotation.
     */
    public static final Quaternion IDENTITY = new Quaternion();
    /**
     * Another shared instance of the identity quaternion (0, 0, 0, 1). Do not
     * modify!
     */
    public static final Quaternion DIRECTION_Z = new Quaternion();
    /**
     * Shared instance of the zero quaternion (0, 0, 0, 0). Do not modify!
     *
     * <p>The zero quaternion doesn't represent any valid rotation.
     */
    public static final Quaternion ZERO = new Quaternion(0, 0, 0, 0);

    static {
        DIRECTION_Z.fromAxes(Vector3f.UNIT_X, Vector3f.UNIT_Y, Vector3f.UNIT_Z);
    }
    /**
     * The first imaginary (X) component. Not an angle!
     */
    protected float x;
    /**
     * The 2nd imaginary (Y) component. Not an angle!
     */
    protected float y;
    /**
     * The 3rd imaginary (Z) component. Not an angle!
     */
    protected float z;
    /**
     * The real (W) component. Not an angle!
     */
    protected float w;

    /**
     * Instantiates an identity quaternion: all components zeroed except
     * {@code w}, which is set to 1.
     */
    public Quaternion() {
        x = 0;
        y = 0;
        z = 0;
        w = 1;
    }

    /**
     * Instantiates a quaternion with the specified components.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     * @param w the desired W component
     */
    public Quaternion(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Returns the X component. The quaternion is unaffected.
     *
     * @return the value of the {@link #x} component
     */
    public float getX() {
        return x;
    }

    /**
     * Returns the Y component. The quaternion is unaffected.
     *
     * @return the value of the {@link #y} component
     */
    public float getY() {
        return y;
    }

    /**
     * Returns the Z component. The quaternion is unaffected.
     *
     * @return the value of the {@link #z} component
     */
    public float getZ() {
        return z;
    }

    /**
     * Returns the W (real) component. The quaternion is unaffected.
     *
     * @return the value of the {@link #w} component
     */
    public float getW() {
        return w;
    }

    /**
     * Sets all 4 components to specified values.
     *
     * @param x the desired X component
     * @param y the desired Y component
     * @param z the desired Z component
     * @param w the desired W component
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion set(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }

    /**
     * Copies all 4 components from the argument.
     *
     * @param q the quaternion to copy (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion set(Quaternion q) {
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
        return this;
    }

    /**
     * Instantiates a copy of the argument.
     *
     * @param q the quaternion to copy (not null, unaffected)
     */
    public Quaternion(Quaternion q) {
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
    }

    /**
     * Sets all components to zero except {@code w}, which is set to 1.
     */
    public void loadIdentity() {
        x = y = z = 0;
        w = 1;
    }

    /**
     * Sets the quaternion from the specified Tait-Bryan angles, applying the
     * rotations in x-z-y extrinsic order or y-z'-x" intrinsic order.
     *
     * @see
     * <a href="http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm">http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm</a>
     *
     * @param xAngle the X angle (in radians)
     * @param yAngle the Y angle (in radians)
     * @param zAngle the Z angle (in radians)
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion fromAngles(float xAngle, float yAngle, float zAngle) {
        float angle;
        float sinY, sinZ, sinX, cosY, cosZ, cosX;
        angle = zAngle * 0.5f;
        sinZ = FastMath.sin(angle);
        cosZ = FastMath.cos(angle);
        angle = yAngle * 0.5f;
        sinY = FastMath.sin(angle);
        cosY = FastMath.cos(angle);
        angle = xAngle * 0.5f;
        sinX = FastMath.sin(angle);
        cosX = FastMath.cos(angle);

        // variables used to reduce multiplication calls.
        float cosYXcosZ = cosY * cosZ;
        float sinYXsinZ = sinY * sinZ;
        float cosYXsinZ = cosY * sinZ;
        float sinYXcosZ = sinY * cosZ;

        w = (cosYXcosZ * cosX - sinYXsinZ * sinX);
        x = (cosYXcosZ * sinX + sinYXsinZ * cosX);
        y = (sinYXcosZ * cosX + cosYXsinZ * sinX);
        z = (cosYXsinZ * cosX - sinYXcosZ * sinX);

        normalizeLocal();
        return this;
    }

    /**
     * Sets the quaternion from the specified rotation matrix. Does not verify
     * that the argument is a valid rotation matrix.
     *
     * @param matrix the input matrix (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion fromRotationMatrix(Matrix3f matrix) {
        return fromRotationMatrix(matrix.m00, matrix.m01, matrix.m02, matrix.m10,
                matrix.m11, matrix.m12, matrix.m20, matrix.m21, matrix.m22);
    }

    /**
     * Sets the quaternion from a rotation matrix with the specified elements.
     * Does not verify that the arguments form a valid rotation matrix.
     *
     * @param m00 the matrix element in row 0, column 0
     * @param m01 the matrix element in row 0, column 1
     * @param m02 the matrix element in row 0, column 2
     * @param m10 the matrix element in row 1, column 0
     * @param m11 the matrix element in row 1, column 1
     * @param m12 the matrix element in row 1, column 2
     * @param m20 the matrix element in row 2, column 0
     * @param m21 the matrix element in row 2, column 1
     * @param m22 the matrix element in row 2, column 2
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion fromRotationMatrix(float m00, float m01, float m02,
            float m10, float m11, float m12, float m20, float m21, float m22) {
        // first normalize the forward (F), up (U) and side (S) vectors of the rotation matrix
        // so that the scale does not affect the rotation
        float lengthSquared = m00 * m00 + m10 * m10 + m20 * m20;
        if (lengthSquared != 1f && lengthSquared != 0f) {
            lengthSquared = 1.0f / FastMath.sqrt(lengthSquared);
            m00 *= lengthSquared;
            m10 *= lengthSquared;
            m20 *= lengthSquared;
        }
        lengthSquared = m01 * m01 + m11 * m11 + m21 * m21;
        if (lengthSquared != 1f && lengthSquared != 0f) {
            lengthSquared = 1.0f / FastMath.sqrt(lengthSquared);
            m01 *= lengthSquared;
            m11 *= lengthSquared;
            m21 *= lengthSquared;
        }
        lengthSquared = m02 * m02 + m12 * m12 + m22 * m22;
        if (lengthSquared != 1f && lengthSquared != 0f) {
            lengthSquared = 1.0f / FastMath.sqrt(lengthSquared);
            m02 *= lengthSquared;
            m12 *= lengthSquared;
            m22 *= lengthSquared;
        }

        // Use the Graphics Gems code, from
        // ftp://ftp.cis.upenn.edu/pub/graphics/shoemake/quatut.ps.Z
        // *NOT* the "Matrix and Quaternions FAQ", which has errors!

        // the trace is the sum of the diagonal elements; see
        // http://mathworld.wolfram.com/MatrixTrace.html
        float t = m00 + m11 + m22;

        // we protect the division by s by ensuring that s>=1
        if (t >= 0) { // |w| >= .5
            float s = FastMath.sqrt(t + 1); // |s|>=1 ...
            w = 0.5f * s;
            s = 0.5f / s;                 // so this division isn't bad
            x = (m21 - m12) * s;
            y = (m02 - m20) * s;
            z = (m10 - m01) * s;
        } else if ((m00 > m11) && (m00 > m22)) {
            float s = FastMath.sqrt(1.0f + m00 - m11 - m22); // |s|>=1
            x = s * 0.5f; // |x| >= .5
            s = 0.5f / s;
            y = (m10 + m01) * s;
            z = (m02 + m20) * s;
            w = (m21 - m12) * s;
        } else if (m11 > m22) {
            float s = FastMath.sqrt(1.0f + m11 - m00 - m22); // |s|>=1
            y = s * 0.5f; // |y| >= .5
            s = 0.5f / s;
            x = (m10 + m01) * s;
            z = (m21 + m12) * s;
            w = (m02 - m20) * s;
        } else {
            float s = FastMath.sqrt(1.0f + m22 - m00 - m11); // |s|>=1
            z = s * 0.5f; // |z| >= .5
            s = 0.5f / s;
            x = (m02 + m20) * s;
            y = (m21 + m12) * s;
            w = (m10 - m01) * s;
        }

        return this;
    }

    /**
     * Converts to an equivalent rotation matrix. The current instance is
     * unaffected.
     *
     * <p>Note: the result is created from a normalized version of the current
     * instance.
     *
     * @return a new 3x3 rotation matrix
     */
    public Matrix3f toRotationMatrix() {
        Matrix3f matrix = new Matrix3f();
        return toRotationMatrix(matrix);
    }

    /**
     * Converts to an equivalent rotation matrix. The current instance is
     * unaffected.
     *
     * <p>Note: the result is created from a normalized version of the current
     * instance.
     *
     * @param result storage for the result (not null)
     * @return {@code result}, configured as a 3x3 rotation matrix
     */
    public Matrix3f toRotationMatrix(Matrix3f result) {

        float norm = norm();
        // we explicitly test norm against one here, saving a division
        // at the cost of a test and branch.  Is it worth it?
        float s = (norm == 1f) ? 2f : (norm > 0f) ? 2f / norm : 0;

        // compute xs/ys/zs first to save 6 multiplications, since xs/ys/zs
        // will be used 2-4 times each.
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;
        float xx = x * xs;
        float xy = x * ys;
        float xz = x * zs;
        float xw = w * xs;
        float yy = y * ys;
        float yz = y * zs;
        float yw = w * ys;
        float zz = z * zs;
        float zw = w * zs;

        // using s=2/norm (instead of 1/norm) saves 9 multiplications by 2 here
        result.m00 = 1 - (yy + zz);
        result.m01 = (xy - zw);
        result.m02 = (xz + yw);
        result.m10 = (xy + zw);
        result.m11 = 1 - (xx + zz);
        result.m12 = (yz - xw);
        result.m20 = (xz - yw);
        result.m21 = (yz + xw);
        result.m22 = 1 - (xx + yy);

        return result;
    }

    /**
     * Sets the rotation component of the specified transform matrix. The
     * current instance is unaffected.
     *
     * <p>Note: preserves the translation component of {@code store} but not
     * its scaling component.
     *
     * <p>Note: the result is created from a normalized version of the current
     * instance.
     *
     * @param store storage for the result (not null)
     * @return {@code store}, with 9 of its 16 elements modified
     */
    public Matrix4f toTransformMatrix(Matrix4f store) {

        float norm = norm();
        // we explicitly test norm against one here, saving a division
        // at the cost of a test and branch.  Is it worth it?
        float s = (norm == 1f) ? 2f : (norm > 0f) ? 2f / norm : 0;

        // compute xs/ys/zs first to save 6 multiplications, since xs/ys/zs
        // will be used 2-4 times each.
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;
        float xx = x * xs;
        float xy = x * ys;
        float xz = x * zs;
        float xw = w * xs;
        float yy = y * ys;
        float yz = y * zs;
        float yw = w * ys;
        float zz = z * zs;
        float zw = w * zs;

        // using s=2/norm (instead of 1/norm) saves 9 multiplications by 2 here
        store.m00 = 1 - (yy + zz);
        store.m01 = (xy - zw);
        store.m02 = (xz + yw);
        store.m10 = (xy + zw);
        store.m11 = 1 - (xx + zz);
        store.m12 = (yz - xw);
        store.m20 = (xz - yw);
        store.m21 = (yz + xw);
        store.m22 = 1 - (xx + yy);

        return store;
    }

    /**
     * Sets the rotation component of the specified transform matrix. The
     * current instance is unaffected.
     *
     * <p>Note: preserves the translation and scaling components of
     * {@code result}.
     *
     * <p>Note: the result is created from a normalized version of the current
     * instance.
     *
     * @param result storage for the result (not null)
     * @return {@code result}, with 9 of its 16 elements modified
     */
    public Matrix4f toRotationMatrix(Matrix4f result) {
        TempVars tempv = TempVars.get();
        Vector3f originalScale = tempv.vect1;

        result.toScaleVector(originalScale);
        result.setScale(1, 1, 1);
        float norm = norm();
        // we explicitly test norm against one here, saving a division
        // at the cost of a test and branch.  Is it worth it?
        float s = (norm == 1f) ? 2f : (norm > 0f) ? 2f / norm : 0;

        // compute xs/ys/zs first to save 6 multiplications, since xs/ys/zs
        // will be used 2-4 times each.
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;
        float xx = x * xs;
        float xy = x * ys;
        float xz = x * zs;
        float xw = w * xs;
        float yy = y * ys;
        float yz = y * zs;
        float yw = w * ys;
        float zz = z * zs;
        float zw = w * zs;

        // using s=2/norm (instead of 1/norm) saves 9 multiplications by 2 here
        result.m00 = 1 - (yy + zz);
        result.m01 = (xy - zw);
        result.m02 = (xz + yw);
        result.m10 = (xy + zw);
        result.m11 = 1 - (xx + zz);
        result.m12 = (yz - xw);
        result.m20 = (xz - yw);
        result.m21 = (yz + xw);
        result.m22 = 1 - (xx + yy);

        result.setScale(originalScale);

        tempv.release();

        return result;
    }

    /**
     * Sets the quaternion from the specified rotation angle and normalized axis
     * of rotation.
     *
     * @param angle the desired rotation angle (in radians)
     * @param axis the desired axis of rotation (not null, length=1, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion fromAngleNormalAxis(float angle, Vector3f axis) {
        if (axis.x == 0 && axis.y == 0 && axis.z == 0) {
            loadIdentity();
        } else {
            float halfAngle = 0.5f * angle;
            float sin = FastMath.sin(halfAngle);
            w = FastMath.cos(halfAngle);
            x = sin * axis.x;
            y = sin * axis.y;
            z = sin * axis.z;
        }
        return this;
    }

    /**
     * Adds the argument and returns the (modified) current instance.
     *
     * <p>Seldom used. To combine rotations, use
     * {@link #multLocal(com.jme3.math.Quaternion)} or
     * {@link #mult(com.jme3.math.Quaternion, com.jme3.math.Quaternion)}
     * instead of this method.
     *
     * @param q the quaternion to add (not null, unaffected unless it's
     *     {@code this})
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion addLocal(Quaternion q) {
        this.x += q.x;
        this.y += q.y;
        this.z += q.z;
        this.w += q.w;
        return this;
    }

    /**
     * Multiplies by the argument and returns the product as a new instance.
     * The current instance is unaffected.
     *
     * <p>This method is used to combine rotations. Note that quaternion
     * multiplication is noncommutative, so generally q * p != p * q.
     *
     * @param q the right factor (not null, unaffected)
     * @return {@code this * q} (a new Quaternion)
     */
    public Quaternion mult(Quaternion q) {
        return mult(q, null);
    }

    /**
     * Multiplies by the specified quaternion and returns the product in a 3rd
     * quaternion. The current instance is unaffected, unless it's {@code storeResult}.
     *
     * <p>This method is used to combine rotations. Note that quaternion
     * multiplication is noncommutative, so generally q * p != p * q.
     *
     * <p>It is safe for {@code q} and {@code storeResult} to be the same object.
     * However, if {@code this} and {@code storeResult} are the same object, the result
     * is undefined.
     *
     * @param q the right factor (not null, unaffected unless it's {@code storeResult})
     * @param storeResult storage for the product, or null for a new Quaternion
     * @return {@code this * q} (either {@code storeResult} or a new Quaternion)
     */
    public Quaternion mult(Quaternion q, Quaternion storeResult) {
        if (storeResult == null) {
            storeResult = new Quaternion();
        }
        float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
        storeResult.x = x * qw + y * qz - z * qy + w * qx;
        storeResult.y = -x * qz + y * qw + z * qx + w * qy;
        storeResult.z = x * qy - y * qx + z * qw + w * qz;
        storeResult.w = -x * qx - y * qy - z * qz + w * qw;
        return storeResult;
    }

    /**
     * Sets the quaternion from the specified orthonormal basis.
     *
     * <p>The 3 basis vectors describe the axes of a rotated coordinate system.
     * They are assumed to be normalized, mutually orthogonal, and in right-hand
     * order. No error checking is performed; the caller must ensure that the
     * specified vectors represent a right-handed coordinate system.
     *
     * @param xAxis the X axis of the desired coordinate system (not null,
     *     length=1, unaffected)
     * @param yAxis the Y axis of the desired coordinate system (not null,
     *     length=1, unaffected)
     * @param zAxis the Z axis of the desired coordinate system (not null,
     *     length=1, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion fromAxes(Vector3f xAxis, Vector3f yAxis, Vector3f zAxis) {
        return fromRotationMatrix(xAxis.x, yAxis.x, zAxis.x, xAxis.y, yAxis.y,
                zAxis.y, xAxis.z, yAxis.z, zAxis.z);
    }

    /**
     * Rotates the argument vector. Despite the name, the current instance is
     * unaffected.
     *
     * <p>Despite the name, the result differs from the mathematical definition
     * of vector-quaternion multiplication.
     *
     * @param v the vector to rotate (not null)
     * @return the (modified) vector {@code v}
     */
    public Vector3f multLocal(Vector3f v) {
        float tempX, tempY;
        tempX = w * w * v.x + 2 * y * w * v.z - 2 * z * w * v.y + x * x * v.x
                + 2 * y * x * v.y + 2 * z * x * v.z - z * z * v.x - y * y * v.x;
        tempY = 2 * x * y * v.x + y * y * v.y + 2 * z * y * v.z + 2 * w * z
                * v.x - z * z * v.y + w * w * v.y - 2 * x * w * v.z - x * x
                * v.y;
        v.z = 2 * x * z * v.x + 2 * y * z * v.y + z * z * v.z - 2 * w * y * v.x
                - y * y * v.z + 2 * w * x * v.y - x * x * v.z + w * w * v.z;
        v.x = tempX;
        v.y = tempY;
        return v;
    }

    /**
     * Multiplies by the argument and returns the (modified) current instance.
     *
     * <p>This method is used to combine rotations. Note that quaternion
     * multiplication is noncommutative, so generally q * p != p * q.
     *
     * @param q the right factor (not null, unaffected unless it's {@code this})
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion multLocal(Quaternion q) {
        float x1 = x * q.w + y * q.z - z * q.y + w * q.x;
        float y1 = -x * q.z + y * q.w + z * q.x + w * q.y;
        float z1 = x * q.y - y * q.x + z * q.w + w * q.z;
        w = -x * q.x - y * q.y - z * q.z + w * q.w;
        x = x1;
        y = y1;
        z = z1;
        return this;
    }

    /**
     * Multiplies by a quaternion with the specified components and returns the
     * (modified) current instance.
     *
     * <p>This method is used to combine rotations. Note that quaternion
     * multiplication is noncommutative, so generally q * p != p * q.
     *
     * @param qx the X component of the right factor
     * @param qy the Y component of the right factor
     * @param qz the Z component of the right factor
     * @param qw the W component of the right factor
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion multLocal(float qx, float qy, float qz, float qw) {
        float x1 = x * qw + y * qz - z * qy + w * qx;
        float y1 = -x * qz + y * qw + z * qx + w * qy;
        float z1 = x * qy - y * qx + z * qw + w * qz;
        w = -x * qx - y * qy - z * qz + w * qw;
        x = x1;
        y = y1;
        z = z1;
        return this;
    }

    /**
     * Rotates a specified vector and returns the result in another vector. The
     * current instance is unaffected.
     *
     * <p>It is safe for {@code v} and {@code store} to be the same object.
     *
     * <p>Despite the name, the result differs from the mathematical definition
     * of vector-quaternion multiplication.
     *
     * @param v the vector to rotate (not null, unaffected unless it's
     *     {@code store})
     * @param store storage for the result, or null for a new Vector3f
     * @return the rotated vector (either {@code store} or a new Vector3f)
     */
    public Vector3f mult(Vector3f v, Vector3f store) {
        if (store == null) {
            store = new Vector3f();
        }
        if (v.x == 0 && v.y == 0 && v.z == 0) {
            store.set(0, 0, 0);
        } else {
            float vx = v.x, vy = v.y, vz = v.z;
            store.x = w * w * vx + 2 * y * w * vz - 2 * z * w * vy + x * x
                    * vx + 2 * y * x * vy + 2 * z * x * vz - z * z * vx - y
                    * y * vx;
            store.y = 2 * x * y * vx + y * y * vy + 2 * z * y * vz + 2 * w
                    * z * vx - z * z * vy + w * w * vy - 2 * x * w * vz - x
                    * x * vy;
            store.z = 2 * x * z * vx + 2 * y * z * vy + z * z * vz - 2 * w
                    * y * vx - y * y * vz + 2 * w * x * vy - x * x * vz + w
                    * w * vz;
        }
        return store;
    }

    /**
     * Multiplies by the scalar argument and returns the (modified) current
     * instance.
     *
     * @param scalar the scaling factor
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion multLocal(float scalar) {
        w *= scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return this;
    }

    /**
     * Returns the norm, defined as the dot product of the quaternion with
     * itself. The current instance is unaffected.
     *
     * @return the sum of the squared components (not negative)
     */
    public float norm() {
        return w * w + x * x + y * y + z * z;
    }

    /**
     * Scales the quaternion to have norm=1 and returns the (modified) current
     * instance. For a quaternion with norm=0, the result is undefined.
     *
     * @return the (modified) current instance (for chaining)
     */
    public Quaternion normalizeLocal() {
        float n = 1f / FastMath.sqrt(norm());
        x *= n;
        y *= n;
        z *= n;
        w *= n;
        return this;
    }

    /**
     * Returns the multiplicative inverse. For a quaternion with norm=0, null is
     * returned. Either way, the current instance is unaffected.
     *
     * @return a new Quaternion or null
     */
    public Quaternion inverse() {
        float norm = norm();
        if (norm > 0.0) {
            float invNorm = 1.0f / norm;
            return new Quaternion(-x * invNorm, -y * invNorm, -z * invNorm, w
                    * invNorm);
        }
        // return an invalid result to flag the error
        return null;
    }

    /**
     * Returns a string representation of the quaternion, which is unaffected.
     * For example, the identity quaternion is represented by:
     * <pre>
     * (0.0, 0.0, 0.0, 1.0)
     * </pre>
     *
     * @return the string representation (not null, not empty)
     */
    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ", " + w + ")";
    }

    /**
     * Tests for exact equality with the argument, distinguishing -0 from 0. If
     * {@code o} is null, false is returned. Either way, the current instance is
     * unaffected.
     *
     * @param o the object to compare (may be null, unaffected)
     * @return true if {@code this} and {@code o} have identical values,
     *     otherwise false
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Quaternion)) {
            return false;
        }

        if (this == o) {
            return true;
        }

        Quaternion comp = (Quaternion) o;
        if (Float.compare(x, comp.x) != 0) {
            return false;
        }
        if (Float.compare(y, comp.y) != 0) {
            return false;
        }
        if (Float.compare(z, comp.z) != 0) {
            return false;
        }
        if (Float.compare(w, comp.w) != 0) {
            return false;
        }
        return true;
    }

    /**
     * Returns a hash code. If two quaternions have identical values, they
     * will have the same hash code. The current instance is unaffected.
     *
     * @return a 32-bit value for use in hashing
     * @see java.lang.Object#hashCode()
     */
    @Override
    public int hashCode() {
        int hash = 37;
        hash = 37 * hash + Float.floatToIntBits(x);
        hash = 37 * hash + Float.floatToIntBits(y);
        hash = 37 * hash + Float.floatToIntBits(z);
        hash = 37 * hash + Float.floatToIntBits(w);
        return hash;

    }

    /**
     * Creates a copy. The current instance is unaffected.
     *
     * @return a new instance, equivalent to the current one
     */
    @Override
    public Quaternion clone() {
        try {
            return (Quaternion) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new AssertionError(); // can not happen
        }
    }
}
