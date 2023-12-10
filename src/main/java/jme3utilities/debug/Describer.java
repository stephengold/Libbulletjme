/*
 Copyright (c) 2013-2023, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.debug;

import com.jme3.bounding.BoundingBox;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * Generate compact textual descriptions of Libbulletjme objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Describer implements Cloneable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(Describer.class.getName());
    /**
     * separator between items in lists (not null, may be empty)
     */
    private String listSeparator = " ";
    // *************************************************************************
    // constructors

    /**
     * A no-arg constructor to avoid javadoc warnings from JDK 18.
     */
    public Describer() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a compact, textual description of the specified BoundingBox.
     *
     * @param aabb the box to describe (not null, unaffected)
     * @return description (not null, not empty)
     */
    public String describe(BoundingBox aabb) {
        StringBuilder builder = new StringBuilder(80);

        builder.append("loc[");
        Vector3f location = new Vector3f();
        aabb.getCenter(location);
        String desc = MyVector3f.describe(location);
        builder.append(desc);
        builder.append(']');

        Vector3f he = aabb.getExtent(null);
        desc = describeHalfExtents(he);
        builder.append(desc);

        return builder.toString();
    }

    /**
     * Generate a compact, textual description of the specified scale vector.
     *
     * @param vector the vector to describe (not null, unaffected)
     * @return a description (not null, may be empty)
     */
    public String describeScale(Vector3f vector) {
        Validate.nonNull(vector, "vector");
        StringBuilder result = new StringBuilder(30);

        if (!MyVector3f.isScaleIdentity(vector)) {
            result.append("scale[");
            String vectorText = MyVector3f.describe(vector);
            result.append(vectorText);
            result.append(']');
        }

        return result.toString();
    }

    /**
     * Return the list separator.
     *
     * @return separator text string (not null, may be empty, default=" ")
     */
    public String listSeparator() {
        assert listSeparator != null;
        return listSeparator;
    }

    /**
     * Alter the list separator.
     *
     * @param newSeparator (not null, may be empty)
     */
    public void setListSeparator(String newSeparator) {
        Validate.nonNull(newSeparator, "new separator");
        this.listSeparator = newSeparator;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Describe the specified half extents.
     *
     * @param he the half extent for each axis (not null, unaffected)
     * @return a bracketed description (not null, not empty)
     */
    protected String describeHalfExtents(Vector3f he) {
        String desc = MyVector3f.describe(he);
        String result = String.format(" he[%s]", desc);

        return result;
    }
    // *************************************************************************
    // Cloneable methods

    /**
     * Create a copy of this Describer.
     *
     * @return a new instance, equivalent to this one
     * @throws CloneNotSupportedException if the superclass isn't cloneable
     */
    @Override
    public Describer clone() throws CloneNotSupportedException {
        Describer clone = (Describer) super.clone();
        return clone;
    }
}
