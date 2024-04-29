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

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.infos.BoundingValueHierarchy;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.math.Vector3f;
import com.jme3.system.JmeSystem;
import com.jme3.system.Platform;
import java.util.logging.Logger;
import org.junit.Assert;
import org.junit.Test;

/**
 * JUnit automated tests for MeshCollisionShape and BoundedValueHierarchy.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestBvh {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(TestBvh.class.getName());
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
    // new methods exposed

    /**
     * Generate a non-trivial quantized BVH, serialize it, and de-serialize it.
     */
    @Test
    public void test020() {
        Utils.loadNativeLibrary();

        IndexedMesh submesh = new IndexedMesh(positionArray, indexArray);
        MeshCollisionShape shape = new MeshCollisionShape(true, submesh);
        BoundingValueHierarchy bvh = shape.getBvh();
        int numNodes = bvh.countNodes();
        Assert.assertEquals(135, numNodes);
        Assert.assertEquals(2, bvh.countSubtreeHeaders());
        Assert.assertEquals(135, bvh.escapeIndex(0));
        Assert.assertEquals(51, bvh.escapeIndex(1));
        Assert.assertEquals(19, bvh.escapeIndex(2));
        Assert.assertEquals(9, bvh.escapeIndex(3));
        Assert.assertEquals(3, bvh.escapeIndex(4));

        // Serialize the BVH:
        byte[] bytes = bvh.serialize();
        int numBytes = bytes.length;

        // Check the number of bytes:
        boolean dp = NativeLibrary.isDoublePrecision();
        Platform platform = JmeSystem.getPlatform();
        switch (platform) {
            case Linux_ARM32:
                Assert.assertEquals(dp ? 2448 : 2396, numBytes);
                break;
            case Linux64:
            case Linux_ARM64:
                Assert.assertEquals(dp ? 2520 : 2472, numBytes);
                break;
            case MacOSX64:
            case MacOSX_ARM64:
                Assert.assertEquals(dp ? 2520 : 2480, numBytes);
                break;
            case Windows64:
                Assert.assertEquals(dp ? 2528 : 2480, numBytes);
                break;
            default: // TODO more platforms
        }
        // Clone the BVH by de-serializing the bytes:
        BoundingValueHierarchy b2 = new BoundingValueHierarchy(bytes);

        // Compare the cloned BVH with the original:
        BoundingBox aabb = bvh.copyAabb(null);
        Vector3f aabbMax = aabb.getMax(null);
        Vector3f aabbMin = aabb.getMin(null);
        BoundingBox aabb2 = b2.copyAabb(null);
        Assert.assertEquals(aabbMax, aabb2.getMax(null));
        Assert.assertEquals(aabbMin, aabb2.getMin(null));

        Vector3f quantization = bvh.copyQuantization(null);
        Assert.assertEquals(quantization, b2.copyQuantization(null));

        Assert.assertEquals(numNodes, b2.countNodes());
        Assert.assertEquals(bvh.isCompressed(), b2.isCompressed());
        Assert.assertEquals(
                bvh.countSubtreeHeaders(), b2.countSubtreeHeaders());
        Assert.assertEquals(bvh.countLeafNodes(), b2.countLeafNodes());
        Assert.assertEquals(bvh.traversalMode(), b2.traversalMode());

        for (int i = 0; i < numNodes; ++i) {
            boolean isLeaf = bvh.isLeafNode(i);
            Assert.assertEquals(isLeaf, b2.isLeafNode(i));
            Assert.assertEquals(bvh.triangleIndex(i), b2.triangleIndex(i));
            Assert.assertEquals(bvh.partId(i), b2.partId(i));
            Assert.assertEquals(bvh.escapeIndex(i), b2.escapeIndex(i));
        }

        System.gc();
    }
}
