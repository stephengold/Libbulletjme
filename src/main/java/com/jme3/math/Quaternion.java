/*
 * Copyright (c) 2009-2020 jMonkeyEngine
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
 * <code>Quaternion</code> defines a single example of a more general class of
 * hypercomplex numbers. Quaternions extends a rotation in three dimensions to a
 * rotation in four dimensions. This avoids "gimbal lock" and allows for smooth
 * continuous rotation.
 *
 * <code>Quaternion</code> is defined by four floating point numbers: {x y z w}.
 *
 * @author Mark Powell
 * @author Joshua Slack
 */
public final class Quaternion implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;

    private static final Logger logger = Logger.getLogger(Quaternion.class.getName());
    /**
     * Represents the identity quaternion rotation (0, 0, 0, 1).
     */
    public static final Quaternion IDENTITY = new Quaternion();
    /**
     * another instance of the identity Quaternion (0, 0, 0, 1)
     */
    public static final Quaternion DIRECTION_Z = new Quaternion();
    /**
     * The zero quaternion (0, 0, 0, 0) doesn't represent any rotation.
     */
    public static final Quaternion ZERO = new Quaternion(0, 0, 0, 0);

    static {
        DIRECTION_Z.fromAxes(Vector3f.UNIT_X, Vector3f.UNIT_Y, Vector3f.UNIT_Z);
    }
    /**
     * the X component (not an angle!)
     */
    protected float x;
    /**
     * the Y component (not an angle!)
     */
    protected float y;
    /**
     * the Z component (not an angle!)
     */
    protected float z;
    /**
     * the W (real) component (not an angle!)
     */
    protected float w;

    /**
     * Constructor instantiates a new <code>Quaternion</code> object
     * initializing all values to zero, except w which is initialized to 1.
     *
     */
    public Quaternion() {
        x = 0;
        y = 0;
        z = 0;
        w = 1;
    }

    /**
     * Constructor instantiates a new <code>Quaternion</code> object from the
     * given list of parameters.
     *
     * @param x   the x value of the quaternion.
     * @param y   the y value of the quaternion.
     * @param z   the z value of the quaternion.
     * @param w   the w value of the quaternion.
     */
    public Quaternion(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Determine the X component.
     *
     * @return x
     */
    public float getX() {
        return x;
    }

    /**
     * Determine the Y component.
     *
     * @return y
     */
    public float getY() {
        return y;
    }

    /**
     * Determine the Z component.
     *
     * @return z
     */
    public float getZ() {
        return z;
    }

    /**
     * Determine the W (real) component.
     *
     * @return w
     */
    public float getW() {
        return w;
    }

    /**
     * sets the data in a <code>Quaternion</code> object from the given list of
     * parameters.
     *
     * @param x   the x value of the quaternion.
     * @param y   the y value of the quaternion.
     * @param z   the z value of the quaternion.
     * @param w   the w value of the quaternion.
     * @return this
     */
    public Quaternion set(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }

    /**
     * Sets the data in this <code>Quaternion</code> object to be equal to the
     * passed <code>Quaternion</code> object. The values are copied producing
     * a new object.
     *
     * @param q   The Quaternion to copy values from.
     * @return this
     */
    public Quaternion set(Quaternion q) {
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
        return this;
    }

    /**
     * Sets this Quaternion to {0, 0, 0, 1}. Same as calling set(0,0,0,1).
     */
    public void loadIdentity() {
        x = y = z = 0;
        w = 1;
    }

    /**
     * <code>fromAngles</code> builds a Quaternion from the Euler rotation
     * angles (x,y,z) aka (pitch, yaw, roll)).
     * Note that we are applying in order: (y, x, z) aka (yaw, pitch, roll)
     * but we've ordered them in x, y, and z for convenience.
     *
     * @see <a href="http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm">http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm</a>
     *
     * @param xAngle
     *            the Euler pitch of rotation (in radians). (aka Attitude, often rot
     *            around x)
     * @param yAngle
     *            the Euler yaw of rotation (in radians). (aka Heading, often
     *            rot around y)
     * @param zAngle
     *            the Euler roll of rotation (in radians). (aka Bank, often
     *            rot around z)
     * @return this
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
     * <code>fromRotationMatrix</code> generates a quaternion from a supplied
     * matrix. This matrix is assumed to be a rotational matrix.
     *
     * @param matrix
     *            the matrix that defines the rotation.
     * @return this
     */
    public Quaternion fromRotationMatrix(Matrix3f matrix) {
        return fromRotationMatrix(matrix.m00, matrix.m01, matrix.m02, matrix.m10,
                matrix.m11, matrix.m12, matrix.m20, matrix.m21, matrix.m22);
    }

    /**
     * Set this quaternion based on a rotation matrix with the specified
     * elements.
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
     * @return this Quaternion
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
     * <code>toRotationMatrix</code> converts this quaternion to a rotational
     * matrix. Note: the result is created from a normalized version of this quat.
     *
     * @return the rotation matrix representation of this quaternion.
     */
    public Matrix3f toRotationMatrix() {
        Matrix3f matrix = new Matrix3f();
        return toRotationMatrix(matrix);
    }

    /**
     * <code>toRotationMatrix</code> converts this quaternion to a rotational
     * matrix. The result is stored in result.
     *
     * @param result
     *            The Matrix3f to store the result in.
     * @return the rotation matrix representation of this quaternion.
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
     * <code>toTransformMatrix</code> converts this quaternion to a transform
     * matrix. The result is stored in result.
     * Note this method won't preserve the scale of the given matrix.
     *
     * @param store The Matrix3f to store the result in.
     * @return the transform matrix with the rotation representation of this quaternion.
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
     * <code>toRotationMatrix</code> converts this quaternion to a rotational
     * matrix. The result is stored in result. 4th row and 4th column values are
     * untouched. Note: the result is created from a normalized version of this quat.
     * Note that this method will preserve the scale of the given matrix
     *
     * @param result
     *            The Matrix4f to store the result in.
     * @return the rotation matrix representation of this quaternion.
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
     * <code>fromAngleNormalAxis</code> sets this quaternion to the values
     * specified by an angle and a normalized axis of rotation.
     *
     * @param angle
     *            the angle to rotate (in radians).
     * @param axis
     *            the axis of rotation (already normalized).
     * @return this
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
     * <code>add</code> adds the values of this quaternion to those of the
     * parameter quaternion. The result is stored in this Quaternion.
     *
     * @param q   the quaternion to add to this.
     * @return This Quaternion after addition.
     */
    public Quaternion addLocal(Quaternion q) {
        this.x += q.x;
        this.y += q.y;
        this.z += q.z;
        this.w += q.w;
        return this;
    }

    /**
     * <code>mult</code> multiplies this quaternion by a parameter quaternion.
     * The result is returned as a new quaternion. It should be noted that
     * quaternion multiplication is not commutative so q * p != p * q.
     *
     * @param q   the quaternion to multiply this quaternion by.
     * @return the new quaternion.
     */
    public Quaternion mult(Quaternion q) {
        return mult(q, null);
    }

    /**
     * <code>mult</code> multiplies this quaternion by a parameter quaternion.
     * The result is returned as a new quaternion. It should be noted that
     * quaternion multiplication is not commutative so q * p != p * q.
     *
     * It IS safe for q and res to be the same object.
     * It IS NOT safe for this and res to be the same object.
     *
     * @param q   the quaternion to multiply this quaternion by.
     * @param res
     *            the quaternion to store the result in.
     * @return the new quaternion.
     */
    public Quaternion mult(Quaternion q, Quaternion res) {
        if (res == null) {
            res = new Quaternion();
        }
        float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
        res.x = x * qw + y * qz - z * qy + w * qx;
        res.y = -x * qz + y * qw + z * qx + w * qy;
        res.z = x * qy - y * qx + z * qw + w * qz;
        res.w = -x * qx - y * qy - z * qz + w * qw;
        return res;
    }

    /**
     *
     * <code>fromAxes</code> creates a <code>Quaternion</code> that
     * represents the coordinate system defined by three axes. These axes are
     * assumed to be orthogonal and no error checking is applied. Thus, the user
     * must insure that the three axes being provided indeed represents a proper
     * right handed coordinate system.
     *
     * @param xAxis vector representing the x-axis of the coordinate system.
     * @param yAxis vector representing the y-axis of the coordinate system.
     * @param zAxis vector representing the z-axis of the coordinate system.
     * @return this
     */
    public Quaternion fromAxes(Vector3f xAxis, Vector3f yAxis, Vector3f zAxis) {
        return fromRotationMatrix(xAxis.x, yAxis.x, zAxis.x, xAxis.y, yAxis.y,
                zAxis.y, xAxis.z, yAxis.z, zAxis.z);
    }

    /**
     * <code>mult</code> multiplies this quaternion by a parameter vector. The
     * result is stored in the supplied vector
     *
     * @param v   the vector to multiply this quaternion by.
     * @return v
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
     * Multiplies this Quaternion by the supplied quaternion. The result is
     * stored in this Quaternion, which is also returned for chaining. Similar
     * to this *= q.
     *
     * @param q   The Quaternion to multiply this one by.
     * @return This Quaternion, after multiplication.
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
     * <code>mult</code> multiplies this quaternion by a parameter vector. The
     * result is returned as a new vector.
     *
     * @param v
     *            the vector to multiply this quaternion by.
     * @param store
     *            the vector to store the result in. It IS safe for v and store
     *            to be the same object.
     * @return the result vector.
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
     * <code>mult</code> multiplies this quaternion by a parameter scalar. The
     * result is stored locally.
     *
     * @param scalar
     *            the quaternion to multiply this quaternion by.
     * @return this.
     */
    public Quaternion multLocal(float scalar) {
        w *= scalar;
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return this;
    }

    /**
     * <code>norm</code> returns the norm of this quaternion. This is the dot
     * product of this quaternion with itself.
     *
     * @return the norm of the quaternion.
     */
    public float norm() {
        return w * w + x * x + y * y + z * z;
    }

    /**
     * <code>normalize</code> normalizes the current <code>Quaternion</code>.
     * The result is stored internally.
     *
     * @return this
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
     * <code>inverse</code> returns the inverse of this quaternion as a new
     * quaternion. If this quaternion does not have an inverse (if its normal is
     * 0 or less), then null is returned.
     *
     * @return the inverse of this quaternion or null if the inverse does not
     *         exist.
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
     * <code>toString</code> returns a string representation of this
     * <code>Quaternion</code>. The format is:
     *
     * (X.XXXX, Y.YYYY, Z.ZZZZ, W.WWWW)
     *
     * @return the string representation of this object.
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ", " + w + ")";
    }

    /**
     * <code>equals</code> determines if two quaternions are logically equal,
     * that is, if the values of (x, y, z, w) are the same for both quaternions.
     *
     * @param o   the object to compare for equality
     * @return true if they are equal, false otherwise.
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
     * <code>hashCode</code> returns the hash code value as an integer and is
     * supported for the benefit of hashing based collection classes such as
     * Hashtable, HashMap, HashSet etc.
     *
     * @return the hashcode for this instance of Quaternion.
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
     * Create a copy of this quaternion.
     *
     * @return a new instance, equivalent to this one
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
