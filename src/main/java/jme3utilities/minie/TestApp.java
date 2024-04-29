/*
 Copyright (c) 2024 Stephen Gold
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
package jme3utilities.minie;

import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.infos.BoundingValueHierarchy;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.math.Vector3f;
import com.jme3.system.JmeSystem;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.system.Platform;
import java.io.File;
import java.util.logging.Logger;

/**
 * Command-line application for testing Libbulletjme.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class TestApp {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(TestApp.class.getName());
    /**
     * vertex positions of the 2nd mesh from "pc-screen.gltf" by joliver82
     */
    final private static Vector3f[] positionArray = {
        new Vector3f(-0.22671932f, 0.5148976f, -0.52107906f),
        new Vector3f(-0.22671932f, 0.014897585f, -0.52107906f),
        new Vector3f(-0.22671932f, 0.014897585f, -0.45209116f),
        new Vector3f(-0.22671932f, 0.5148976f, -0.45209116f),
        new Vector3f(0.22671932f, 0.5148976f, -0.52107906f),
        new Vector3f(0.22671932f, 0.014897585f, -0.52107906f),
        new Vector3f(0.035729814f, 0.014897585f, -0.52107906f),
        new Vector3f(0.035729814f, 0.5148976f, -0.52107906f),
        new Vector3f(0.22671932f, 0.5148976f, 0.5755005f),
        new Vector3f(0.22671932f, 0.014897585f, 0.5755005f),
        new Vector3f(0.22671932f, 0.014897585f, -0.2206986f),
        new Vector3f(0.22671932f, 0.5148976f, -0.2206986f),
        new Vector3f(-0.22671932f, 0.5148976f, 0.5755005f),
        new Vector3f(-0.22671932f, 0.014897585f, 0.5755005f),
        new Vector3f(-0.035729814f, 0.014897585f, 0.5755005f),
        new Vector3f(-0.035729814f, 0.5148976f, 0.5755005f),
        new Vector3f(0.035729814f, 0.014897585f, 0.5755005f),
        new Vector3f(0.035729814f, 0.014897585f, -0.2206986f),
        new Vector3f(0.22671932f, 0.014897585f, -0.2206986f),
        new Vector3f(0.22671932f, 0.014897585f, 0.5755005f),
        new Vector3f(-0.035729814f, 0.5148976f, 0.5755005f),
        new Vector3f(-0.035729814f, 0.5148976f, -0.2206986f),
        new Vector3f(-0.22671932f, 0.5148976f, -0.2206986f),
        new Vector3f(-0.22671932f, 0.5148976f, 0.5755005f),
        new Vector3f(-0.035729814f, 0.014897585f, 0.5755005f),
        new Vector3f(-0.035729814f, 0.014897585f, -0.2206986f),
        new Vector3f(0.035729814f, 0.014897585f, 0.5755005f),
        new Vector3f(0.035729814f, 0.5148976f, 0.5755005f),
        new Vector3f(-0.035729814f, 0.014897585f, -0.52107906f),
        new Vector3f(-0.035729814f, 0.5148976f, -0.52107906f),
        new Vector3f(0.22671932f, 0.5148976f, 0.5755005f),
        new Vector3f(0.22671932f, 0.5148976f, -0.2206986f),
        new Vector3f(0.035729814f, 0.5148976f, -0.2206986f),
        new Vector3f(0.035729814f, 0.5148976f, 0.5755005f),
        new Vector3f(-0.22671932f, 0.014897585f, 0.5755005f),
        new Vector3f(-0.22671932f, 0.014897585f, -0.2206986f),
        new Vector3f(-0.035729814f, 0.014897585f, -0.45209116f),
        new Vector3f(0.035729814f, 0.014897585f, -0.45209116f),
        new Vector3f(0.22671932f, 0.014897585f, -0.45209116f),
        new Vector3f(0.22671932f, 0.014897585f, -0.45209116f),
        new Vector3f(0.22671932f, 0.5148976f, -0.45209116f),
        new Vector3f(-0.22671932f, 0.014897585f, -0.2206986f),
        new Vector3f(-0.22671932f, 0.5148976f, -0.2206986f),
        new Vector3f(0.035729814f, 0.5148976f, -0.45209116f),
        new Vector3f(0.035729814f, 0.5148976f, -0.52107906f),
        new Vector3f(-0.035729814f, 0.5148976f, -0.52107906f),
        new Vector3f(-0.035729814f, 0.5148976f, -0.45209116f),
        new Vector3f(-0.22671932f, 0.014897585f, 0.5755005f),
        new Vector3f(-0.22671932f, 0.5148976f, 0.5755005f),
        new Vector3f(-0.22671932f, 0.014897585f, -0.45209116f),
        new Vector3f(0.22671932f, 0.5148976f, -0.45209116f),
        new Vector3f(-0.025823109f, 2.5102012f, -0.15026632f),
        new Vector3f(0.025823109f, 2.5102012f, -0.15026632f),
        new Vector3f(-0.22671932f, 0.5148976f, -0.45209116f),
        new Vector3f(0.035729814f, 0.014897585f, -0.52107906f),
        new Vector3f(0.22671932f, 0.014897585f, -0.52107906f),
        new Vector3f(-0.22671932f, 0.5148976f, -0.52107906f),
        new Vector3f(-0.035729814f, 0.014897585f, -0.52107906f),
        new Vector3f(0.025823109f, 2.5102012f, -0.32126576f),
        new Vector3f(-0.025823109f, 2.5102012f, -0.32126576f),
        new Vector3f(0.22671932f, 0.5148976f, -0.52107906f),
        new Vector3f(-0.22671932f, 0.014897585f, -0.52107906f)
    };
    /**
     * vertex indices of the 2nd mesh from "pc-screen.gltf" by joliver82
     */
    final private static int[] indexArray = {
        0, 1, 2,
        0, 2, 3,
        4, 5, 6,
        4, 6, 7,
        8, 9, 10,
        8, 10, 11,
        12, 13, 14,
        12, 14, 15,
        16, 17, 18,
        16, 18, 19,
        20, 21, 22,
        20, 22, 23,
        24, 25, 17,
        24, 17, 16,
        15, 14, 26,
        15, 26, 27,
        7, 6, 28,
        7, 28, 29,
        27, 26, 9,
        27, 9, 8,
        30, 31, 32,
        30, 32, 33,
        29, 28, 1,
        29, 1, 0,
        34, 35, 25,
        34, 25, 24,
        25, 36, 37,
        25, 37, 17,
        17, 37, 38,
        17, 38, 18,
        11, 10, 39,
        11, 39, 40,
        3, 2, 41,
        3, 41, 42,
        43, 44, 45,
        43, 45, 46,
        42, 41, 47,
        42, 47, 48,
        35, 49, 36,
        35, 36, 25,
        31, 50, 43,
        31, 43, 32,
        51, 21, 32,
        51, 32, 52,
        21, 46, 53,
        21, 53, 22,
        40, 39, 5,
        40, 5, 4,
        37, 54, 55,
        37, 55, 38,
        46, 45, 56,
        46, 56, 53,
        36, 57, 54,
        36, 54, 37,
        52, 58, 59,
        52, 59, 51,
        50, 60, 44,
        50, 44, 43,
        58, 43, 46,
        58, 46, 59,
        49, 61, 57,
        49, 57, 36,
        52, 32, 43,
        52, 43, 58,
        59, 46, 21,
        59, 21, 51,
        33, 32, 21,
        33, 21, 20
    };
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private TestApp() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TestApp application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        loadNativeLibrary();

        boolean dp = NativeLibrary.isDoublePrecision();
        System.out.println("dp = " + dp);

        Platform platform = JmeSystem.getPlatform();
        System.out.println("platform = " + platform);

        IndexedMesh submesh = new IndexedMesh(positionArray, indexArray);
        MeshCollisionShape shape = new MeshCollisionShape(true, submesh);
        BoundingValueHierarchy bvh = shape.getBvh();

        byte[] bytes = bvh.serialize();
        int numBytes = bytes.length;
        System.out.println("numBytes = " + numBytes);
    }
    // *************************************************************************
    // private methods

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
    private static void loadNativeLibrary() {
        boolean fromDist = false;

        File directory;
        if (fromDist) {
            directory = new File("dist");
        } else {
            directory = new File("build/libs/bulletjme/shared");
        }

        boolean success = NativeLibraryLoader.loadLibbulletjme(
                fromDist, directory, "Debug", "SpMt");
        if (!success) { // fallback to Sp-flavored library
            success = NativeLibraryLoader.loadLibbulletjme(
                    fromDist, directory, "Debug", "Sp");
        }
        if (!success) { // fallback to Dp-flavored library
            success = NativeLibraryLoader.loadLibbulletjme(
                    fromDist, directory, "Debug", "Dp");
        }
        if (!success) {
            System.exit(1);
        }
    }
}
