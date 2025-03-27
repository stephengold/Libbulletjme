/*
 Copyright (c) 2021-2025 Stephen Gold
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

import com.jme3.bullet.util.NativeLibrary;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.io.File;
import java.util.logging.Logger;
import org.junit.Assert;

/**
 * Utility methods for automated testing of the Libbulletjme Physics Library.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class Utils {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(Utils.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private Utils() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Verify that 2 double-precision quaternions are equal to within some
     * tolerance.
     *
     * @param x the expected X component
     * @param y the expected Y component
     * @param z the expected Z component
     * @param w the expected W component
     * @param actual the Quaternion to test (not null, unaffected)
     * @param tolerance the allowable difference for each component (&ge;0)
     */
    public static void assertEquals(double x, double y, double z, double w,
            Quatd actual, double tolerance) {
        Assert.assertEquals("x component", x, actual.x, tolerance);
        Assert.assertEquals("y component", y, actual.y, tolerance);
        Assert.assertEquals("z component", z, actual.z, tolerance);
        Assert.assertEquals("w component", w, actual.w, tolerance);
    }

    /**
     * Verify that 2 single-precision quaternions are equal to within some
     * tolerance.
     *
     * @param x the expected X component
     * @param y the expected Y component
     * @param z the expected Z component
     * @param w the expected W component
     * @param actual the Quaternion to test (not null, unaffected)
     * @param tolerance the allowable difference for each component (&ge;0)
     */
    public static void assertEquals(float x, float y, float z, float w,
            Quaternion actual, float tolerance) {
        Assert.assertEquals("x component", x, actual.getX(), tolerance);
        Assert.assertEquals("y component", y, actual.getY(), tolerance);
        Assert.assertEquals("z component", z, actual.getZ(), tolerance);
        Assert.assertEquals("w component", w, actual.getW(), tolerance);
    }

    /**
     * Verify that 2 double-precision vectors are equal to within some
     * tolerance.
     *
     * @param x the expected X component
     * @param y the expected Y component
     * @param z the expected Z component
     * @param actual the vector to test (not null, unaffected)
     * @param tolerance the allowable difference for each component (&ge;0)
     */
    public static void assertEquals(
            double x, double y, double z, Vec3d actual, double tolerance) {
        Assert.assertEquals("x component", x, actual.x, tolerance);
        Assert.assertEquals("y component", y, actual.y, tolerance);
        Assert.assertEquals("z component", z, actual.z, tolerance);
    }

    /**
     * Verify that 2 single-precision vectors are equal to within some
     * tolerance.
     *
     * @param x the expected X component
     * @param y the expected Y component
     * @param z the expected Z component
     * @param actual the vector to test (not null, unaffected)
     * @param tolerance the allowable difference for each component (&ge;0)
     */
    public static void assertEquals(
            float x, float y, float z, Vector3f actual, float tolerance) {
        Assert.assertEquals("x component", x, actual.x, tolerance);
        Assert.assertEquals("y component", y, actual.y, tolerance);
        Assert.assertEquals("z component", z, actual.z, tolerance);
    }

    /**
     * Verify that 2 vectors are equal to within some tolerance.
     *
     * @param expected the expected value (not null, unaffected)
     * @param actual the vector to test (not null, unaffected)
     * @param tolerance the allowable difference for each component (&ge;0)
     */
    public static void assertEquals(
            Vector3f expected, Vector3f actual, float tolerance) {
        assertEquals(expected.x, expected.y, expected.z, actual, tolerance);
    }

    /**
     * Load a Debug native library and verify its properties.
     * <p>
     * The search order is:
     * <ol>
     * <li>DebugSpMt</li>
     * <li>DebugSp</li>
     * <li>DebugDp</li>
     * </ol>
     */
    public static void loadNativeLibrary() {
        boolean fromDist = false;

        File directory;
        if (fromDist) {
            directory = new File("dist");
        } else {
            directory = new File("build/libs/bulletjme/shared");
        }

        boolean success = NativeLibraryLoader.loadLibbulletjme(
                fromDist, directory, "Debug", "SpMt");
        if (success) {
            Assert.assertFalse(NativeLibrary.isDoublePrecision());
            Assert.assertTrue(NativeLibrary.isThreadSafe());

        } else { // fallback to Sp-flavored library
            success = NativeLibraryLoader.loadLibbulletjme(
                    fromDist, directory, "Debug", "Sp");
            if (success) {
                Assert.assertFalse(NativeLibrary.isDoublePrecision());
                Assert.assertFalse(NativeLibrary.isThreadSafe());
            }
        }

        if (!success) { // fallback to Dp-flavored library
            success = NativeLibraryLoader.loadLibbulletjme(
                    fromDist, directory, "Debug", "Dp");
            if (success) {
                Assert.assertTrue(NativeLibrary.isDoublePrecision());
                Assert.assertFalse(NativeLibrary.isThreadSafe());
            }
        }

        Assert.assertTrue(success);
        Assert.assertTrue(NativeLibrary.countThreads() > 0);
        Assert.assertTrue(NativeLibrary.isDebug());
        Assert.assertFalse(NativeLibrary.versionNumber().isEmpty());
    }
}
