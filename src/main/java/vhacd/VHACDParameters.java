/*
 Copyright (c) 2016, Riccardo Balbo
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

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
package vhacd;

import com.jme3.bullet.NativePhysicsObject;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A set of tuning parameters for convex decomposition, based on V-HACD's
 * IVHACD::Parameters.
 */
public class VHACDParameters
        extends NativePhysicsObject
        implements Cloneable {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(VHACDParameters.class.getName());
    // *************************************************************************
    // fields

    /**
     * true&rarr;enable debug output
     */
    private boolean debug;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the default tuning parameters.
     */
    public VHACDParameters() {
        long objectId = create();
        super.setNativeId(objectId);

        setMaxNumVerticesPerCH(objectId, 32);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Read selected parameters from an InputStream.
     *
     * @param is (not null)
     * @throws IOException from DataInputStream
     */
    public void fromInputStream(InputStream is) throws IOException {
        DataInputStream dis = new DataInputStream(is);
        setVoxelResolution(dis.readInt());
        setMaxVerticesPerHull(dis.readInt());
    }

    /**
     * Test whether debug output is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean getDebugEnabled() {
        return debug;
    }

    /**
     * Read the maximum number of vertices per hull (native field:
     * m_maxNumVerticesPerCH).
     *
     * @return the limit (&ge;4, &le;1024)
     */
    public int getMaxVerticesPerHull() {
        long objectId = nativeId();
        int result = getMaxNumVerticesPerCH(objectId);

        return result;
    }

    /**
     * Read the maximum number of voxels generated during the voxelization stage
     * (native field: m_resolution).
     *
     * @return number (&ge;10000, &le;64000000)
     */
    public int getVoxelResolution() {
        long objectId = nativeId();
        int result = getResolution(objectId);

        return result;
    }

    /**
     * Alter whether debug output is enabled.
     *
     * @param d true &rarr; enable, false &rarr; disable (default=false)
     */
    public void setDebugEnabled(boolean d) {
        debug = d;
    }

    /**
     * Set maximum number of vertices per convex-hull (native field:
     * m_maxNumVerticesPerCH).
     *
     * @param v default = 32, min = 4, max = 1024)
     */
    public void setMaxVerticesPerHull(int v) {
        Validate.inRange(v, "max vertices", 4, 1024);

        long objectId = nativeId();
        setMaxNumVerticesPerCH(objectId, v);
    }

    /**
     * Set maximum number of voxels generated during the voxelization stage
     * (native field: m_resolution).
     *
     * @param v default = 100_000, min = 10_000, max = 64_000_000
     */
    public void setVoxelResolution(int v) {
        Validate.inRange(v, "maxVoxels", 10_000, 64_000_000);

        long objectId = nativeId();
        setResolution(objectId, v);
    }

    /**
     * Write selected parameters to an OutputStream.
     *
     * @param os (not null)
     * @throws IOException from DataOutputStream
     */
    public void toOutputStream(OutputStream os) throws IOException {
        DataOutputStream dos = new DataOutputStream(os);
        dos.writeInt(getVoxelResolution());
        dos.writeInt(getMaxVerticesPerHull());
    }
    // *************************************************************************
    // NativePhysicsObject methods

    /**
     * Create a copy of these parameters.
     *
     * @return a new instance, equivalent to this one
     */
    @Override
    public VHACDParameters clone() {
        try {
            VHACDParameters clone = (VHACDParameters) super.clone();
            long objectId = create();
            clone.reassignNativeId(objectId);

            clone.setMaxVerticesPerHull(getMaxVerticesPerHull());
            clone.setVoxelResolution(getVoxelResolution());
            return clone;

        } catch (CloneNotSupportedException exception) {
            throw new RuntimeException(exception);
        }
    }

    /**
     * Test for exact equivalence with another Object.
     *
     * @param otherObject (may be null, unaffected)
     * @return true if the objects are equivalent, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;
        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            VHACDParameters other = (VHACDParameters) otherObject;
            result = getDebugEnabled() == other.getDebugEnabled()
                    && getMaxVerticesPerHull() == other.getMaxVerticesPerHull()
                    && getVoxelResolution() == other.getVoxelResolution();
        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this object.
     *
     * @return value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = 5;
        hash = 83 * hash + (debug ? 1 : 0);
        hash = 83 * hash + getMaxVerticesPerHull();
        hash = 83 * hash + getVoxelResolution();

        return hash;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Free the identified tracked native object. Invoked by reflection.
     *
     * @param objectId the native identifier (not zero)
     */
    private static void freeNativeObject(long objectId) {
        assert objectId != 0L;
        finalizeNative(objectId);
    }
    // *************************************************************************
    // native private methods

    native private static long create();

    native private static void finalizeNative(long objectId);

    native private static int getMaxNumVerticesPerCH(long objectId);

    native private static int getResolution(long objectId);

    native private static void setMaxNumVerticesPerCH(long objectId,
            int numVertices);

    native private static void setResolution(long objectId, int maxVoxels);
}
