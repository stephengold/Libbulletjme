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
package com.jme3.math;

import com.jme3.util.TempVars;

/**
 * Started Date: Jul 16, 2004<br><br>
 * Represents a translation, rotation and scale in one object.
 * 
 * @author Jack Lindamood
 * @author Joshua Slack
 */
public final class Transform implements Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;

    public static final Transform IDENTITY = new Transform();

    private Quaternion rot = new Quaternion();
    private Vector3f translation = new Vector3f();
    private Vector3f scale = new Vector3f(1, 1, 1);

    public Transform(Vector3f translation, Quaternion rot){
        this.translation.set(translation);
        this.rot.set(rot);
    }
    
    public Transform(Vector3f translation, Quaternion rot, Vector3f scale){
        this(translation, rot);
        this.scale.set(scale);
    }

    public Transform(){
        this(Vector3f.ZERO, Quaternion.IDENTITY);
    }

    /**
     * Sets this translation to the given value.
     * @param trans The new translation for this matrix.
     * @return this
     */
    public Transform setTranslation(Vector3f trans) {
        this.translation.set(trans);
        return this;
    }

    /**
     * Return the translation vector in this matrix.
     * @return translation vector.
     */
    public Vector3f getTranslation() {
        return translation;
    }

    /**
     * Sets this scale to the given value.
     * @param scale The new scale for this matrix.
     * @return this
     */
    public Transform setScale(float scale) {
        this.scale.set(scale, scale, scale);
        return this;
    }

    /**
     * Return the scale vector in this matrix.
     * @return scale vector.
     */
    public Vector3f getScale() {
        return scale;
    }

    /**
     * Return the rotation quaternion in this matrix.
     * @return rotation quaternion.
     */
    public Quaternion getRotation() {
        return rot;
    } 
    
    /**
     * Changes the values of this matrix according to its parent.  Very similar to the concept of Node/Spatial transforms.
     * @param parent The parent matrix.
     * @return This matrix, after combining.
     */
    public Transform combineWithParent(Transform parent) {
        //applying parent scale to local scale
        scale.multLocal(parent.scale);
        //applying parent rotation to local rotation.
        parent.rot.mult(rot, rot);
        //applying parent scale to local translation.
        translation.multLocal(parent.scale);
        //applying parent rotation to local translation, then applying parent translation to local translation.
        //Note that parent.rot.multLocal(translation) doesn't modify "parent.rot" but "translation"
        parent
            .rot
            .multLocal(translation)
            .addLocal(parent.translation);

        return this;
    }

    public Vector3f transformVector(final Vector3f in, Vector3f store){
        if (store == null)
            store = new Vector3f();

        // multiply with scale first, then rotate, finally translate (cf.
        // Eberly)
        return rot.mult(store.set(in).multLocal(scale), store).addLocal(translation);
    }

    public Matrix4f toTransformMatrix() {
        return toTransformMatrix(null);
    }

    public Matrix4f toTransformMatrix(Matrix4f store) {
        if (store == null) {
            store = new Matrix4f();
        }
        store.setTranslation(translation);
        rot.toTransformMatrix(store);
        store.setScale(scale);
        return store;
    }
    
    public void fromTransformMatrix(Matrix4f mat) {
        TempVars vars = TempVars.get();
        translation.set(mat.toTranslationVector(vars.vect1));
        rot.set(mat.toRotationQuat(vars.quat1));
        scale.set(mat.toScaleVector(vars.vect2));
        vars.release();
    }
    
    public Transform invert() {
        Transform t = new Transform();
        t.fromTransformMatrix(toTransformMatrix().invertLocal());
        return t;
    }
    
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 89 * hash + rot.hashCode();
        hash = 89 * hash + translation.hashCode();
        hash = 89 * hash + scale.hashCode();
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Transform other = (Transform) obj;
        return this.translation.equals(other.translation)
                && this.scale.equals(other.scale)
                && this.rot.equals(other.rot);
    }

    @Override
    public String toString(){
        return getClass().getSimpleName() + "[ " + translation.x + ", " + translation.y + ", " + translation.z + "]\n"
                                          + "[ " + rot.x + ", " + rot.y + ", " + rot.z + ", " + rot.w + "]\n"
                                          + "[ " + scale.x + " , " + scale.y + ", " + scale.z + "]";
    }

    @Override
    public Transform clone() {
        try {
            Transform tq = (Transform) super.clone();
            tq.rot = rot.clone();
            tq.scale = scale.clone();
            tq.translation = translation.clone();
            return tq;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
