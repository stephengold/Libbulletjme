/*
 Copyright (c) 2020, Stephen Gold
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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import java.io.File;
import java.util.List;
import org.junit.Assert;
import org.junit.Test;
import vhacd.VHACD;
import vhacd.VHACDHull;
import vhacd.VHACDParameters;

/**
 * JUnit automated tests for Libbulletjme.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestLibbulletjme {

    @Test
    public void test001() {
        loadNativeLibrary();
        /*
         * Create a PhysicsSpace using DBVT for broadphase.
         */
        PhysicsSpace space = new PhysicsSpace(
                new Vector3f(-10000f, -10000f, -10000f),
                new Vector3f(10000f, 10000f, 10000f),
                PhysicsSpace.BroadphaseType.DBVT);
        /*
         * Add a static horizontal plane at y=-1.
         */
        Plane plane = new Plane(Vector3f.UNIT_Y, -1f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        PhysicsRigidBody floorPrb = new PhysicsRigidBody(pcs, 0f);
        space.addCollisionObject(floorPrb);
        /*
         * Add a box-shaped dynamic rigid body at y=0.
         */
        CollisionShape bcs = new BoxCollisionShape(0.1f, 0.2f, 0.3f);
        PhysicsRigidBody prb = new PhysicsRigidBody(bcs, 1f);
        space.addCollisionObject(prb);
        /*
         * 50 iterations with a 20-msec timestep
         */
        for (int i = 0; i < 50; ++i) {
            space.update(0.02f, 0);
            //System.out.printf("location = %s%n", prb.getPhysicsLocation());
        }
        /*
         * Check the final location of the box.
         */
        Vector3f location = prb.getPhysicsLocation();
        Assert.assertEquals(0f, location.x, 0.2f);
        Assert.assertEquals(-0.8f, location.y, 0.04f);
        Assert.assertEquals(0f, location.z, 0.2f);
    }

    @Test
    public void test002() {
        loadNativeLibrary();
        /*
         * Generate an L-shaped mesh: 12 vertices, 20 triangles
         */
        float[] positionArray = new float[]{
            0f, 0f, 0f,
            2f, 0f, 0f,
            2f, 1f, 0f,
            1f, 1f, 0f,
            1f, 3f, 0f,
            0f, 3f, 1f,
            0f, 0f, 1f,
            2f, 0f, 1f,
            2f, 1f, 1f,
            1f, 1f, 1f,
            1f, 3f, 1f,
            0f, 3f, 1f
        };
        int[] indexArray = new int[]{
            0, 1, 7, 0, 7, 6,
            0, 6, 11, 0, 11, 5,
            4, 5, 11, 4, 11, 10,
            3, 4, 10, 3, 10, 9,
            2, 3, 9, 2, 9, 8,
            1, 2, 8, 1, 8, 7,
            0, 3, 2, 0, 2, 1,
            0, 5, 4, 0, 4, 3,
            6, 8, 9, 6, 7, 8,
            6, 10, 11, 6, 9, 10
        };
        /*
         * Generate a hulls for the mesh.
         */
        VHACDParameters parameters = new VHACDParameters();
        List<VHACDHull> hulls
                = VHACD.compute(positionArray, indexArray, parameters);
        /*
         * Check the results.
         */
        Assert.assertEquals(2, hulls.size());
    }
    // *************************************************************************
    // private methods

    private void loadNativeLibrary() {
        boolean loadFromDist = false;

        File directory;
        if (loadFromDist) {
            directory = new File("dist");
        } else {
            directory = new File("build/libs/bulletjme/shared");
        }
        NativeLibraryLoader.loadLibbulletjme(loadFromDist, directory,
                "Debug", "Sp");
    }
}
