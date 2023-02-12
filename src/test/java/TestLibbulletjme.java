/*
 Copyright (c) 2020-2023, Stephen Gold
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

import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.DeformableSpace;
import com.jme3.bullet.FillMode;
import com.jme3.bullet.MultiBody;
import com.jme3.bullet.MultiBodyJointType;
import com.jme3.bullet.MultiBodyLink;
import com.jme3.bullet.MultiBodySpace;
import com.jme3.bullet.PhysicsSoftSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.RayTestFlag;
import com.jme3.bullet.RotationOrder;
import com.jme3.bullet.SolverInfo;
import com.jme3.bullet.SolverType;
import com.jme3.bullet.collision.Activation;
import com.jme3.bullet.collision.ManifoldPoints;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.Box2dShape;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.ConeCollisionShape;
import com.jme3.bullet.collision.shapes.Convex2dShape;
import com.jme3.bullet.collision.shapes.ConvexShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.EmptyShape;
import com.jme3.bullet.collision.shapes.GImpactCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.MultiSphere;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.joints.New6Dof;
import com.jme3.bullet.objects.MultiBodyCollider;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsCharacter;
import com.jme3.bullet.objects.PhysicsGhostObject;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsSoftBody;
import com.jme3.bullet.objects.infos.RigidBodyMotionState;
import com.jme3.bullet.objects.infos.SoftBodyConfig;
import com.jme3.bullet.objects.infos.SoftBodyMaterial;
import com.jme3.bullet.util.DebugShapeFactory;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.bullet.util.NativeSoftBodyUtil;
import com.jme3.math.Matrix3f;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import com.simsilica.mathd.Matrix3d;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.io.File;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import jme3utilities.Validate;
import org.junit.Assert;
import org.junit.Test;
import vhacd.ACDMode;
import vhacd.VHACD;
import vhacd.VHACDHull;
import vhacd.VHACDParameters;
import vhacd4.Vhacd4;
import vhacd4.Vhacd4Hull;
import vhacd4.Vhacd4Parameters;

/**
 * JUnit automated tests for Libbulletjme.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestLibbulletjme {
    // *************************************************************************
    // constants and loggers

    /**
     * number of vertices per triangle
     */
    final private static int vpt = 3;
    // *************************************************************************
    // fields

    /**
     * true if the expected collision in performDropTest() has been detected
     */
    private static boolean dropAndFloorHaveCollided = false;
    /**
     * dynamic rigid body in performDropTest()
     */
    private static PhysicsRigidBody drop;
    /**
     * static rigid body in performDropTest()
     */
    private static PhysicsRigidBody floor;
    // *************************************************************************
    // new methods exposed

    /**
     * Drop a dynamic box on a static plane.
     */
    @Test
    public void test001() {
        loadNativeLibrary();

        // Create a PhysicsSpace using DBVT for broadphase.
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);

        // Add a static horizontal plane at y=-1.
        Plane plane = new Plane(Vector3f.UNIT_Y, -1f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        PhysicsRigidBody floorPrb = new PhysicsRigidBody(pcs, 0f);
        testPco(floorPrb);
        space.addCollisionObject(floorPrb);

        Assert.assertEquals(2, floorPrb.proxyGroup().intValue());
        Assert.assertEquals(-3, floorPrb.proxyMask().intValue());

        // Add a box-shaped dynamic rigid body at y=0.
        CollisionShape bcs = new BoxCollisionShape(0.1f, 0.2f, 0.3f);
        PhysicsRigidBody prb = new PhysicsRigidBody(bcs, 1f);
        testPco(prb);
        space.addCollisionObject(prb);

        Assert.assertEquals(1, prb.proxyGroup().intValue());
        Assert.assertEquals(-1, prb.proxyMask().intValue());

        // 50 iterations with a 20-msec timestep
        for (int i = 0; i < 50; ++i) {
            space.update(0.02f, 0);
        }

        // Check the final location of the box.
        Vector3f location = prb.getPhysicsLocation(null);
        Assert.assertEquals(0f, location.x, 0.2f);
        Assert.assertEquals(-0.8f, location.y, 0.04f);
        Assert.assertEquals(0f, location.z, 0.2f);

        space = null;
        System.gc();
    }

    /**
     * Generate a collision shape using V-HACD.
     */
    @Test
    public void test002() {
        loadNativeLibrary();

        // Generate an L-shaped mesh: 12 vertices, 20 triangles
        float[] positionArray = {
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
        int[] indexArray = {
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

        // Verify the Vhacd4Parameters defaults.
        Vhacd4Parameters parameters4 = new Vhacd4Parameters();
        Assert.assertTrue(parameters4.isAsync());
        Assert.assertFalse(parameters4.getDebugEnabled());
        Assert.assertEquals(FillMode.FloodFill, parameters4.getFillMode());
        Assert.assertFalse(parameters4.isFindBestPlane());
        Assert.assertEquals(64, parameters4.getMaxHulls());
        Assert.assertEquals(14, parameters4.getMaxRecursion());
        Assert.assertEquals(32, parameters4.getMaxVerticesPerHull());
        Assert.assertEquals(2, parameters4.getMinEdgeLength());
        Assert.assertTrue(parameters4.isShrinkWrap());
        Assert.assertEquals(1., parameters4.getVolumePercentError(), 0.);
        Assert.assertEquals(100_000, parameters4.getVoxelResolution());

        // Generate hulls for the mesh.
        parameters4.setMaxRecursion(2);
        List<Vhacd4Hull> vhacd4Hulls
                = Vhacd4.compute(positionArray, indexArray, parameters4);
        Assert.assertEquals(6, vhacd4Hulls.size());

        CompoundCollisionShape compound = new CompoundCollisionShape();
        int numHullVertices = 0;
        for (Vhacd4Hull vhacdHull : vhacd4Hulls) {
            HullCollisionShape hullShape = new HullCollisionShape(vhacdHull);
            numHullVertices += hullShape.countHullVertices();
            compound.addChildShape(hullShape);
        }
        Assert.assertEquals(58, numHullVertices);

        // Verify the VHACDParameters defaults.
        VHACDParameters parameters = new VHACDParameters();
        Assert.assertFalse(parameters.getDebugEnabled());
        Assert.assertEquals(ACDMode.VOXEL, parameters.getACDMode());
        Assert.assertEquals(0.05, parameters.getAlpha(), 0.);
        Assert.assertEquals(0.05, parameters.getBeta(), 0.);
        Assert.assertEquals(4, parameters.getConvexHullDownSampling());
        Assert.assertEquals(0.0025, parameters.getMaxConcavity(), 0.);
        Assert.assertEquals(32, parameters.getMaxVerticesPerHull());
        Assert.assertEquals(0.0001, parameters.getMinVolumePerHull(), 0.);
        Assert.assertFalse(parameters.getPCA());
        Assert.assertEquals(4, parameters.getPlaneDownSampling());
        Assert.assertEquals(100_000, parameters.getVoxelResolution());

        List<VHACDHull> vhacdHulls
                = VHACD.compute(positionArray, indexArray, parameters);
        Assert.assertEquals(2, vhacdHulls.size());

        compound = new CompoundCollisionShape();
        numHullVertices = 0;
        for (VHACDHull vhacdHull : vhacdHulls) {
            HullCollisionShape hullShape = new HullCollisionShape(vhacdHull);
            numHullVertices += hullShape.countHullVertices();
            compound.addChildShape(hullShape);
        }
        Assert.assertEquals(25, numHullVertices);

        vhacd4Hulls = null;
        vhacdHulls = null;
        System.gc();
    }

    /**
     * Generate various collision shapes and verify their properties.
     */
    @Test
    public void test003() {
        loadNativeLibrary();
        FloatBuffer buf;

        // Box2d
        Box2dShape box2d = new Box2dShape(1f, 2f);
        verifyCollisionShapeDefaults(box2d);
        Assert.assertEquals(0.04f, box2d.getMargin(), 0f);
        Assert.assertFalse(box2d.isConcave());
        Assert.assertTrue(box2d.isConvex());
        Assert.assertFalse(box2d.isInfinite());
        Assert.assertFalse(box2d.isNonMoving());
        Assert.assertFalse(box2d.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(box2d, DebugShapeFactory.lowResolution);
        Assert.assertEquals(108, buf.capacity());

        // Box
        BoxCollisionShape box = new BoxCollisionShape(1f);
        verifyCollisionShapeDefaults(box);
        Assert.assertEquals(0.04f, box.getMargin(), 0f);
        Assert.assertFalse(box.isConcave());
        Assert.assertTrue(box.isConvex());
        Assert.assertFalse(box.isInfinite());
        Assert.assertFalse(box.isNonMoving());
        Assert.assertTrue(box.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(box, DebugShapeFactory.lowResolution);
        Assert.assertEquals(108, buf.capacity());

        // Capsule
        CapsuleCollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        verifyCollisionShapeDefaults(capsule);
        Assert.assertEquals(PhysicsSpace.AXIS_Y, capsule.getAxis());
        Assert.assertEquals(1f, capsule.getHeight(), 0f);
        Assert.assertEquals(0f, capsule.getMargin(), 0f);
        Assert.assertFalse(capsule.isConcave());
        Assert.assertTrue(capsule.isConvex());
        Assert.assertFalse(capsule.isInfinite());
        Assert.assertFalse(capsule.isNonMoving());
        Assert.assertFalse(capsule.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(capsule, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());

        // Compound without added children
        CompoundCollisionShape compound = new CompoundCollisionShape();
        verifyCollisionShapeDefaults(compound);
        Assert.assertEquals(0, compound.countChildren());
        Assert.assertEquals(0.04f, compound.getMargin(), 0f);
        Assert.assertFalse(compound.isConcave());
        Assert.assertFalse(compound.isConvex());
        Assert.assertFalse(compound.isInfinite());
        Assert.assertFalse(compound.isNonMoving());
        Assert.assertFalse(compound.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(compound, DebugShapeFactory.lowResolution);
        Assert.assertEquals(0, buf.capacity());

        // Cone
        ConeCollisionShape cone = new ConeCollisionShape(1f, 1f);
        verifyCollisionShapeDefaults(cone);
        Assert.assertEquals(PhysicsSpace.AXIS_Y, cone.getAxis());
        Assert.assertEquals(1f, cone.getHeight(), 0f);
        Assert.assertEquals(0.04f, cone.getMargin(), 0f);
        Assert.assertFalse(cone.isConcave());
        Assert.assertTrue(cone.isConvex());
        Assert.assertFalse(cone.isInfinite());
        Assert.assertFalse(cone.isNonMoving());
        Assert.assertFalse(cone.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(cone, DebugShapeFactory.lowResolution);
        Assert.assertEquals(702, buf.capacity());

        // Convex2d
        ConeCollisionShape flatCone
                = new ConeCollisionShape(10f, 0f, PhysicsSpace.AXIS_Z);
        Convex2dShape convex2d = new Convex2dShape(flatCone);
        verifyCollisionShapeDefaults(convex2d);
        Assert.assertEquals(0.04f, convex2d.getMargin(), 0f);
        Assert.assertFalse(convex2d.isConcave());
        Assert.assertTrue(convex2d.isConvex());
        Assert.assertFalse(convex2d.isInfinite());
        Assert.assertFalse(convex2d.isNonMoving());
        Assert.assertFalse(convex2d.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(convex2d, DebugShapeFactory.lowResolution);
        Assert.assertEquals(504, buf.capacity());

        // Cylinder
        CylinderCollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        verifyCollisionShapeDefaults(cylinder);
        Assert.assertEquals(PhysicsSpace.AXIS_Z, cylinder.getAxis());
        Assert.assertEquals(2f, cylinder.getHeight(), 0f);
        Assert.assertEquals(0.04f, cylinder.getMargin(), 0f);
        Assert.assertFalse(cylinder.isConcave());
        Assert.assertTrue(cylinder.isConvex());
        Assert.assertFalse(cylinder.isInfinite());
        Assert.assertFalse(cylinder.isNonMoving());
        Assert.assertFalse(cylinder.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(cylinder, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());

        // Empty
        EmptyShape empty = new EmptyShape(true);
        verifyCollisionShapeDefaults(empty);
        Assert.assertEquals(0.04f, empty.getMargin(), 0f);
        Assert.assertTrue(empty.isConcave());
        Assert.assertFalse(empty.isConvex());
        Assert.assertFalse(empty.isInfinite());
        Assert.assertTrue(empty.isNonMoving());
        Assert.assertFalse(empty.isPolyhedral());
        Assert.assertEquals(0f, empty.unscaledVolume(), 0f);
        buf = DebugShapeFactory
                .getDebugTriangles(empty, DebugShapeFactory.lowResolution);
        Assert.assertEquals(0, buf.capacity());

        // Heightfield
        float[] nineHeights = {1f, 0f, 1f, 0f, 0.5f, 0f, 1f, 0f, 1f};
        HeightfieldCollisionShape hcs
                = new HeightfieldCollisionShape(nineHeights);
        verifyCollisionShapeDefaults(hcs);
        Assert.assertEquals(9, hcs.countMeshVertices());
        Assert.assertEquals(0.04f, hcs.getMargin(), 0f);
        Assert.assertTrue(hcs.isConcave());
        Assert.assertFalse(hcs.isConvex());
        Assert.assertFalse(hcs.isInfinite());
        Assert.assertTrue(hcs.isNonMoving());
        Assert.assertFalse(hcs.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(hcs, DebugShapeFactory.lowResolution);
        Assert.assertEquals(72, buf.capacity());

        // Hull
        List<Vector3f> prismVertices = new ArrayList<>(6);
        prismVertices.add(new Vector3f(1f, 1f, 1f));
        prismVertices.add(new Vector3f(1f, 1f, -1f));
        prismVertices.add(new Vector3f(-1f, 1f, 0f));
        prismVertices.add(new Vector3f(1f, -1f, 1f));
        prismVertices.add(new Vector3f(1f, -1f, -1f));
        prismVertices.add(new Vector3f(-1f, -1f, 0f));
        HullCollisionShape hull = new HullCollisionShape(prismVertices);
        verifyCollisionShapeDefaults(hull);
        Assert.assertEquals(8f, hull.aabbVolume(), 0.001f);
        Assert.assertEquals(6, hull.countHullVertices());
        Assert.assertEquals(6, hull.countMeshVertices());
        Assert.assertEquals(0.04f, hull.getMargin(), 0f);
        Assert.assertFalse(hull.isConcave());
        Assert.assertTrue(hull.isConvex());
        Assert.assertFalse(hull.isInfinite());
        Assert.assertFalse(hull.isNonMoving());
        Assert.assertTrue(hull.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(hull, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());

        // MultiSphere
        MultiSphere multiSphere = new MultiSphere(1f);
        verifyCollisionShapeDefaults(multiSphere);
        Utils.assertEquals(0f, 0f, 0f, multiSphere.copyCenter(0, null), 0f);
        Assert.assertEquals(1, multiSphere.countSpheres());
        Assert.assertEquals(1f, multiSphere.getRadius(0), 0f);
        Assert.assertFalse(multiSphere.isConcave());
        Assert.assertTrue(multiSphere.isConvex());
        Assert.assertFalse(multiSphere.isInfinite());
        Assert.assertFalse(multiSphere.isNonMoving());
        Assert.assertFalse(multiSphere.isPolyhedral());
        buf = DebugShapeFactory.getDebugTriangles(
                multiSphere, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());

        // Plane
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        PlaneCollisionShape pcs = new PlaneCollisionShape(plane);
        verifyCollisionShapeDefaults(pcs);
        Assert.assertEquals(0.04f, pcs.getMargin(), 0f);
        Assert.assertEquals(0f, pcs.getPlane().getConstant(), 0f);
        Utils.assertEquals(0f, 1f, 0f, pcs.getPlane().getNormal(), 0f);
        Assert.assertTrue(pcs.isConcave());
        Assert.assertFalse(pcs.isConvex());
        Assert.assertTrue(pcs.isInfinite());
        Assert.assertTrue(pcs.isNonMoving());
        Assert.assertFalse(pcs.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(pcs, DebugShapeFactory.lowResolution);
        Assert.assertEquals(18, buf.capacity());

        // Simplex of 1 vertex
        SimplexCollisionShape simplex1
                = new SimplexCollisionShape(new Vector3f(0f, 0f, 0f));
        verifySimplexDefaults(simplex1);
        Assert.assertEquals(1, simplex1.countMeshVertices());
        Utils.assertEquals(0f, 0f, 0f, simplex1.copyVertex(0, null), 0f);
        Utils.assertEquals(0f, 0f, 0f, simplex1.getHalfExtents(null), 0f);
        buf = DebugShapeFactory
                .getDebugTriangles(simplex1, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());

        // Simplex of 2 vertices
        SimplexCollisionShape simplex2 = new SimplexCollisionShape(
                new Vector3f(1f, 0f, 0f), new Vector3f(-1, 0f, 0f));
        verifySimplexDefaults(simplex2);
        Assert.assertEquals(2, simplex2.countMeshVertices());
        Utils.assertEquals(1f, 0f, 0f, simplex2.copyVertex(0, null), 0f);
        Utils.assertEquals(-1f, 0f, 0f, simplex2.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, 0f, 0f, simplex2.getHalfExtents(null), 0f);
        buf = DebugShapeFactory
                .getDebugTriangles(simplex2, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());

        // Simplex of 3 vertices
        SimplexCollisionShape simplex3 = new SimplexCollisionShape(
                new Vector3f(0f, 1f, 1f),
                new Vector3f(1f, 1f, 0f),
                new Vector3f(1f, 0f, 1f)
        );
        verifySimplexDefaults(simplex3);
        Assert.assertEquals(3, simplex3.countMeshVertices());
        Utils.assertEquals(0f, 1f, 1f, simplex3.copyVertex(0, null), 0f);
        Utils.assertEquals(1f, 1f, 0f, simplex3.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, 0f, 1f, simplex3.copyVertex(2, null), 0f);
        Utils.assertEquals(1f, 1f, 1f, simplex3.getHalfExtents(null), 0f);
        buf = DebugShapeFactory
                .getDebugTriangles(simplex3, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());

        // Simplex of 4 vertices
        SimplexCollisionShape simplex4 = new SimplexCollisionShape(
                new Vector3f(0f, 1f, 1f),
                new Vector3f(0f, 1f, -1f),
                new Vector3f(1f, -1f, 0f),
                new Vector3f(-1f, -1f, 0f)
        );
        verifySimplexDefaults(simplex4);
        Assert.assertEquals(4, simplex4.countMeshVertices());
        Utils.assertEquals(0f, 1f, 1f, simplex4.copyVertex(0, null), 0f);
        Utils.assertEquals(0f, 1f, -1f, simplex4.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, -1f, 0f, simplex4.copyVertex(2, null), 0f);
        Utils.assertEquals(-1f, -1f, 0f, simplex4.copyVertex(3, null), 0f);
        Utils.assertEquals(1f, 1f, 1f, simplex4.getHalfExtents(null), 0f);
        buf = DebugShapeFactory
                .getDebugTriangles(simplex4, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());

        // Sphere
        SphereCollisionShape sphere = new SphereCollisionShape(1f);
        verifyCollisionShapeDefaults(sphere);
        Assert.assertEquals(0f, sphere.getMargin(), 0f);
        Assert.assertFalse(sphere.isConcave());
        Assert.assertTrue(sphere.isConvex());
        Assert.assertFalse(sphere.isInfinite());
        Assert.assertFalse(sphere.isNonMoving());
        Assert.assertFalse(sphere.isPolyhedral());
        buf = DebugShapeFactory
                .getDebugTriangles(sphere, DebugShapeFactory.lowResolution);
        Assert.assertEquals(720, buf.capacity());
    }

    /**
     * Perform ray tests against a unit sphere in various collision spaces.
     */
    @Test
    public void test004() {
        loadNativeLibrary();

        float radius = 1f;
        CollisionShape sphereShape = new SphereCollisionShape(radius);

        CollisionSpace space = new CollisionSpace(Vector3f.ZERO,
                Vector3f.ZERO, PhysicsSpace.BroadphaseType.SIMPLE);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new CollisionSpace(
                new Vector3f(-10f, -10f, -10f), new Vector3f(10f, 10f, 10f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new CollisionSpace(
                new Vector3f(-10f, -10f, -10f), new Vector3f(10f, 10f, 10f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3_32);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new CollisionSpace(
                Vector3f.ZERO, Vector3f.ZERO, PhysicsSpace.BroadphaseType.DBVT);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        // Physics spaces with various broadphase accelerators.
        space = new PhysicsSpace(PhysicsSpace.BroadphaseType.SIMPLE);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new PhysicsSpace(
                new Vector3f(-10f, -10f, -10f), new Vector3f(10f, 10f, 10f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new PhysicsSpace(
                new Vector3f(-10f, -10f, -10f), new Vector3f(10f, 10f, 10f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3_32);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        // Soft spaces with various broadphase accelerators.
        space = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.SIMPLE);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new PhysicsSoftSpace(
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3_32);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.DBVT);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        // Multibody spaces with various broadphase accelerators.
        space = new MultiBodySpace(Vector3f.ZERO, Vector3f.ZERO,
                PhysicsSpace.BroadphaseType.SIMPLE);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new MultiBodySpace(
                new Vector3f(-10f, -10f, -10f), new Vector3f(10f, 10f, 10f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new MultiBodySpace(
                new Vector3f(-10f, -10f, -10f), new Vector3f(10f, 10f, 10f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3_32);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new MultiBodySpace(Vector3f.ZERO, Vector3f.ZERO,
                PhysicsSpace.BroadphaseType.DBVT);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        // Deformable spaces with various broadphase accelerators.
        space = new DeformableSpace(Vector3f.ZERO, Vector3f.ZERO,
                PhysicsSpace.BroadphaseType.DBVT, SolverType.SI);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = new DeformableSpace(
                new Vector3f(-10f, -10f, -10f), new Vector3f(10f, 10f, 10f),
                PhysicsSpace.BroadphaseType.AXIS_SWEEP_3, SolverType.SI);
        verifyCollisionSpaceDefaults(space);
        performRayTests(sphereShape, space);

        space = null;
        System.gc();
    }

    /**
     * Perform drop tests using a box-shaped rigid body.
     */
    @Test
    public void test005() {
        loadNativeLibrary();

        float radius = 0.2f;
        CollisionShape boxShape = new BoxCollisionShape(radius);

        // Perform drop tests with various broadphase accelerators.
        performDropTests(boxShape, PhysicsSpace.BroadphaseType.SIMPLE);
        performDropTests(boxShape, PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        performDropTests(boxShape, PhysicsSpace.BroadphaseType.AXIS_SWEEP_3_32);
        performDropTests(boxShape, PhysicsSpace.BroadphaseType.DBVT);
    }

    /**
     * Construct a MultiBody and add it to a MultiBodySpace.
     */
    @Test
    public void test006() {
        loadNativeLibrary();

        int numLinks = 5;
        float baseMass = 1f;
        Vector3f baseInertia = Vector3f.UNIT_XYZ;
        boolean fixedBase = true;
        boolean canSleep = true;
        MultiBody multiBody = new MultiBody(
                numLinks, baseMass, baseInertia, fixedBase, canSleep);

        Assert.assertEquals(0.04f, multiBody.angularDamping(), 0f);
        Utils.assertEquals(0f, 0f, 0f, multiBody.baseForce(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, multiBody.baseLocation(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, 1f, multiBody.baseOrientation(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, multiBody.baseTorque(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, multiBody.baseVelocity(null), 0f);
        Assert.assertTrue(multiBody.canSleep());
        Assert.assertTrue(multiBody.canWakeup());
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                multiBody.collideWithGroups());
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                multiBody.collisionGroup());
        Assert.assertEquals(0, multiBody.countConfiguredLinks());
        Assert.assertEquals(0, multiBody.countDofs());
        Assert.assertEquals(0, multiBody.countPositionVariables());
        Assert.assertNull(multiBody.getBaseCollider());
        Assert.assertNotEquals(0L, multiBody.nativeId());
        Assert.assertFalse(multiBody.isUsingGlobalVelocities());
        Assert.assertTrue(multiBody.isUsingGyroTerm());
        Assert.assertFalse(multiBody.isUsingRK4());
        Assert.assertEquals(0.04f, multiBody.linearDamping(), 0f);
        Assert.assertEquals(1000f, multiBody.maxAppliedImpulse(), 0f);
        Assert.assertEquals(100f, multiBody.maxCoordinateVelocity(), 0f);
        Assert.assertEquals(0L, multiBody.spaceId());

        float radius = 0.4f;
        CollisionShape shape = new SphereCollisionShape(radius);

        multiBody.addBaseCollider(shape);

        Assert.assertNotNull(multiBody.getBaseCollider());
        testPco(multiBody.getBaseCollider());

        float linkMass = 0.1f;
        Vector3f linkInertia = new Vector3f(0.1f, 0.1f, 0.1f);

        MultiBodyLink link0 = multiBody.configureFixedLink(
                linkMass, linkInertia, null, Quaternion.IDENTITY,
                Vector3f.UNIT_X, Vector3f.UNIT_X);
        Assert.assertNull(link0.getCollider());
        link0.addCollider(shape);

        Assert.assertNotNull(link0.getCollider());
        testPco(link0.getCollider());
        Assert.assertEquals(link0, multiBody.getLink(0));
        Utils.assertEquals(0f, 0f, 0f, link0.appliedForce(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, link0.appliedTorque(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, link0.constraintForce(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, link0.constraintTorque(null), 0f);
        Assert.assertEquals(0, link0.countDofs());
        Assert.assertEquals(0, link0.countPositionVariables());
        Assert.assertNotNull(link0.getCollider());
        Assert.assertEquals(multiBody, link0.getMultiBody());
        Assert.assertNull(link0.getParentLink());
        Assert.assertEquals(0, link0.index());
        Utils.assertEquals(0.1f, 0.1f, 0.1f, link0.inertia(null), 0f);
        Assert.assertFalse(link0.isCollisionWithParent());
        Assert.assertEquals(MultiBodyJointType.Fixed, link0.jointType());
        Assert.assertEquals(linkMass, link0.mass(), 0f);
        Utils.assertEquals(0f, 0f, 0f, 1f, link0.orientation(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link0.parent2Pivot(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link0.pivot2Link(null), 0f);

        boolean disableCollision = true;
        MultiBodyLink link1 = multiBody.configurePlanarLink(
                linkMass, linkInertia, link0, Quaternion.IDENTITY,
                Vector3f.UNIT_Y, Vector3f.UNIT_X, disableCollision);

        Assert.assertEquals(link1, multiBody.getLink(1));
        Utils.assertEquals(0f, 1f, 0f, link1.axis(null), 1e-6f);
        Assert.assertEquals(3, link1.countDofs());
        Assert.assertEquals(3, link1.countPositionVariables());
        Assert.assertEquals(link0, link1.getParentLink());
        Assert.assertEquals(1, link1.index());
        Assert.assertFalse(link1.isCollisionWithParent());
        Assert.assertEquals(0f, link1.jointPosition(2), 0f);
        Assert.assertEquals(0f, link1.jointTorque(2), 0f);
        Assert.assertEquals(MultiBodyJointType.Planar, link1.jointType());
        Assert.assertEquals(0f, link1.jointVelocity(2), 0f);
        Utils.assertEquals(0f, 0f, 0f, 1f, link1.orientation(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link1.parent2Link(null), 0f);

        MultiBodyLink link2 = multiBody.configurePrismaticLink(linkMass,
                linkInertia, link1, Quaternion.IDENTITY, Vector3f.UNIT_Y,
                Vector3f.UNIT_X, Vector3f.UNIT_X, disableCollision);
        link2.addCollider(shape);

        Assert.assertEquals(link2, multiBody.getLink(2));
        Utils.assertEquals(0f, 0f, 0f, link2.appliedForce(null), 0f);
        Utils.assertEquals(0f, 1f, 0f, link2.axis(null), 1e-6f);
        Assert.assertEquals(1, link2.countDofs());
        Assert.assertEquals(1, link2.countPositionVariables());
        Assert.assertFalse(link2.isCollisionWithParent());
        Assert.assertEquals(MultiBodyJointType.Prismatic, link2.jointType());
        Utils.assertEquals(0f, 0f, 0f, 1f, link2.orientation(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link2.parent2Pivot(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link2.pivot2Link(null), 0f);

        MultiBodyLink link3 = multiBody.configureRevoluteLink(linkMass,
                linkInertia, link2, Quaternion.IDENTITY, Vector3f.UNIT_Y,
                Vector3f.UNIT_X, Vector3f.UNIT_X, disableCollision);

        Assert.assertEquals(link3, multiBody.getLink(3));
        Utils.assertEquals(0f, 0f, 0f, link3.appliedForce(null), 0f);
        Utils.assertEquals(0f, 1f, 0f, link3.axis(null), 1e-6f);
        Assert.assertEquals(1, link3.countDofs());
        Assert.assertEquals(1, link3.countPositionVariables());
        Assert.assertNull(link3.getCollider());
        Assert.assertFalse(link3.isCollisionWithParent());
        Assert.assertEquals(MultiBodyJointType.Revolute, link3.jointType());
        Utils.assertEquals(0f, 0f, 0f, 1f, link3.orientation(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link3.parent2Pivot(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link3.pivot2Link(null), 0f);

        boolean enableCollision = false;
        MultiBodyLink link4 = multiBody.configureSphericalLink(
                linkMass, linkInertia, link3, Quaternion.IDENTITY,
                Vector3f.UNIT_X, Vector3f.UNIT_X, enableCollision);
        link4.addCollider(shape);

        Assert.assertEquals(link4, multiBody.getLink(4));
        Utils.assertEquals(0f, 0f, 0f, link4.appliedForce(null), 0f);
        Assert.assertEquals(3, link4.countDofs());
        Assert.assertEquals(4, link4.countPositionVariables());
        Assert.assertTrue(link4.isCollisionWithParent());
        Assert.assertEquals(MultiBodyJointType.Spherical, link4.jointType());
        Utils.assertEquals(0f, 0f, 0f, 1f, link4.orientation(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link4.parent2Pivot(null), 0f);
        Utils.assertEquals(1f, 0f, 0f, link4.pivot2Link(null), 0f);

        Assert.assertEquals(5, multiBody.countConfiguredLinks());
        Assert.assertEquals(8, multiBody.countDofs());
        Assert.assertEquals(9, multiBody.countPositionVariables());

        MultiBodySpace space = new MultiBodySpace(
                Vector3f.ZERO, Vector3f.ZERO, PhysicsSpace.BroadphaseType.DBVT);
        verifyCollisionSpaceDefaults(space);

        space.add(multiBody);

        Assert.assertEquals(space.nativeId(), multiBody.spaceId());
        Assert.assertEquals(4, space.countCollisionObjects());
        Assert.assertEquals(0, space.countJoints());
        Assert.assertEquals(1, space.countMultiBodies());
        Assert.assertEquals(0, space.countRigidBodies());
        Assert.assertFalse(space.isEmpty());

        Assert.assertEquals(
                2, multiBody.getBaseCollider().proxyGroup().intValue());
        Assert.assertEquals(
                -3, multiBody.getBaseCollider().proxyMask().intValue());
        Assert.assertEquals(1, link0.getCollider().proxyGroup().intValue());
        Assert.assertEquals(-1, link0.getCollider().proxyMask().intValue());

        space.remove(multiBody);

        Assert.assertEquals(0L, multiBody.spaceId());
        Assert.assertEquals(0, space.countCollisionObjects());
        Assert.assertEquals(0, space.countMultiBodies());
        Assert.assertTrue(space.isEmpty());

        space = null;
        System.gc();
    }

    /**
     * A simple cloth simulation with a pinned node. For your eyes only!
     */
    @Test
    public void test007() {
        loadNativeLibrary();

        PhysicsSoftSpace physicsSpace
                = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.DBVT);

        // Create a static, rigid sphere and add it to the physics space.
        float radius = 1f;
        SphereCollisionShape shape = new SphereCollisionShape(radius);
        PhysicsRigidBody sphere
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);
        testPco(sphere);
        physicsSpace.addCollisionObject(sphere);

        // Generate a subdivided square mesh with alternating diagonals.
        int numLines = 41;
        float lineSpacing = 0.1f; // mesh units
        IndexedMesh squareGrid
                = createClothGrid(numLines, numLines, lineSpacing);

        // Create a soft square and add it to the physics space.
        PhysicsSoftBody cloth = new PhysicsSoftBody();
        testPco(cloth);
        NativeSoftBodyUtil.appendFromNativeMesh(squareGrid, cloth);
        physicsSpace.addCollisionObject(cloth);

        Assert.assertEquals(1, cloth.proxyGroup().intValue());
        Assert.assertEquals(-1, cloth.proxyMask().intValue());

        // Pin one of the corner nodes by setting its mass to zero.
        int nodeIndex = 0;
        cloth.setNodeMass(nodeIndex, PhysicsBody.massForStatic);

        // Make the cloth flexible by altering the angular stiffness
        // of its material.
        SoftBodyMaterial mat = cloth.getSoftMaterial();
        mat.setAngularStiffness(0f); // default=1

        // Improve simulation accuracy by increasing
        // the number of position-solver iterations for the cloth.
        SoftBodyConfig config = cloth.getSoftConfig();
        config.setPositionIterations(9);  // default=1

        // Translate the cloth upward to its starting location.
        cloth.applyTranslation(new Vector3f(0f, 2f, 0f));

        // 50 iterations with a 20-msec timestep
        for (int i = 0; i < 50; ++i) {
            physicsSpace.update(0.02f, 0);
        }

        physicsSpace = null;
        System.gc();
    }

    /**
     * Make sure ignore lists work for rigid bodies.
     */
    @Test
    public void test008() {
        loadNativeLibrary();

        PhysicsSoftSpace physicsSpace
                = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.DBVT);

        // Stack 2 rigid boxes.
        float halfExtent = 1f;
        BoxCollisionShape boxShape = new BoxCollisionShape(halfExtent);

        float mass = 1f;
        PhysicsRigidBody rigid1 = new PhysicsRigidBody(boxShape, mass);
        testPco(rigid1);
        rigid1.setPhysicsLocation(new Vector3f(0f, halfExtent, 0f));
        physicsSpace.addCollisionObject(rigid1);

        PhysicsRigidBody rigid2
                = new PhysicsRigidBody(boxShape, PhysicsBody.massForStatic);
        testPco(rigid2);
        rigid2.setPhysicsLocation(new Vector3f(0f, -halfExtent, 0f));
        rigid2.addToIgnoreList(rigid1);
        physicsSpace.addCollisionObject(rigid2);
        Assert.assertTrue(rigid1.ignores(rigid2));
        Assert.assertEquals(1, rigid2.countIgnored());

        // 50 iterations with a 20-msec timestep
        for (int i = 0; i < 50; ++i) {
            physicsSpace.update(0.02f, 0);
        }
        Vector3f location = rigid1.getPhysicsLocation(null);
        Utils.assertEquals(0f, -4f, 0f, location, 0.1f);

        physicsSpace = null;
        System.gc();
    }

    /**
     * Make sure ignore lists work for characters.
     */
    @Test
    public void test009() {
        loadNativeLibrary();

        PhysicsSoftSpace physicsSpace
                = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.DBVT);

        // Stack a spherical character on a box.
        float radius = 0.5f;
        SphereCollisionShape ballShape = new SphereCollisionShape(radius);

        float stepHeight = 0.1f;
        PhysicsCharacter character
                = new PhysicsCharacter(ballShape, stepHeight);
        testPco(character);
        character.setPhysicsLocation(new Vector3f(0f, radius, 0f));
        physicsSpace.addCollisionObject(character);

        float halfExtent = 1f;
        BoxCollisionShape boxShape = new BoxCollisionShape(halfExtent);

        PhysicsRigidBody rigid2
                = new PhysicsRigidBody(boxShape, PhysicsBody.massForStatic);
        testPco(rigid2);
        rigid2.setPhysicsLocation(new Vector3f(0f, -halfExtent, 0f));
        rigid2.addToIgnoreList(character);
        physicsSpace.addCollisionObject(rigid2);
        Assert.assertTrue(character.ignores(rigid2));
        Assert.assertEquals(1, rigid2.countIgnored());

        // 50 iterations with a 20-msec timestep
        for (int i = 0; i < 50; ++i) {
            physicsSpace.update(0.02f, 0);
        }
        Vector3f result = character.getPhysicsLocation(null);
        Utils.assertEquals(0f, -14.5f, 0f, result, 0.1f);

        physicsSpace = null;
        System.gc();
    }

    /**
     * Make sure ignore lists work for ghosts.
     */
    @Test
    public void test010() {
        loadNativeLibrary();

        PhysicsSoftSpace physicsSpace
                = new PhysicsSoftSpace(PhysicsSpace.BroadphaseType.DBVT);

        // A ghost sphere overlapping with a rigid box.
        float radius = 0.5f;
        SphereCollisionShape ballShape = new SphereCollisionShape(radius);

        PhysicsGhostObject ghost = new PhysicsGhostObject(ballShape);
        testPco(ghost);
        physicsSpace.addCollisionObject(ghost);

        float halfExtent = 1f;
        BoxCollisionShape boxShape = new BoxCollisionShape(halfExtent);

        PhysicsRigidBody rigid2
                = new PhysicsRigidBody(boxShape, PhysicsBody.massForStatic);
        testPco(rigid2);
        rigid2.setPhysicsLocation(new Vector3f(0f, -halfExtent, 0f));
        rigid2.addToIgnoreList(ghost);
        physicsSpace.addCollisionObject(rigid2);
        Assert.assertTrue(ghost.ignores(rigid2));
        Assert.assertEquals(1, rigid2.countIgnored());
        int n = ghost.getOverlappingCount();
        Assert.assertEquals(0, n);

        physicsSpace = null;
        System.gc();
    }

    /**
     * Test the matrixToEuler() method.
     */
    @Test
    public void test011() {
        loadNativeLibrary();

        Quaternion q = new Quaternion().fromAngles(0.3f, 0.7f, 1f);
        Matrix3f rotMatrix = new Matrix3f().set(q);

        Vector3f euler = RotationOrder.XZY.matrixToEuler(rotMatrix, null);
        Utils.assertEquals(-0.3f, -0.7f, -1f, euler, 1e-5f);

        q = null;
        rotMatrix = null;
        euler = null;
        System.gc();
    }

    /**
     * Verify the dispatch matrix.
     */
    @Test
    public void test012() {
        loadNativeLibrary();

        Vector3f min = new Vector3f(-10f, -10f, -10f);
        Vector3f max = new Vector3f(10f, 10f, 10f);
        CollisionSpace space = new CollisionSpace(
                min, max, PhysicsSpace.BroadphaseType.DBVT);

        CollisionShape box = new BoxCollisionShape(0.1f, 0.2f, 0.3f);
        CollisionShape compound = new CompoundCollisionShape();
        CollisionShape gimpact = new GImpactCollisionShape();

        float[] hf = {0f, 0f, 0f, 0f};
        CollisionShape heightfield = new HeightfieldCollisionShape(hf);

        int[] indexArray = {0, 0, 0};
        Vector3f[] positionArray = {new Vector3f(0f, 0f, 0f)};
        IndexedMesh indexedMesh = new IndexedMesh(positionArray, indexArray);
        CollisionShape mesh = new MeshCollisionShape(true, indexedMesh);

        Plane pl = new Plane(Vector3f.UNIT_Y, 0f);
        CollisionShape plane = new PlaneCollisionShape(pl);

        Assert.assertTrue(space.hasClosest(box, box));
        Assert.assertTrue(space.hasClosest(box, compound));
        Assert.assertTrue(space.hasClosest(box, gimpact));
        Assert.assertTrue(space.hasClosest(box, heightfield));
        Assert.assertTrue(space.hasClosest(box, mesh));
        Assert.assertTrue(space.hasClosest(box, plane));

        Assert.assertTrue(space.hasClosest(gimpact, box));
        Assert.assertTrue(space.hasClosest(gimpact, compound));
        Assert.assertTrue(space.hasClosest(gimpact, gimpact));
        Assert.assertTrue(space.hasClosest(gimpact, heightfield));
        Assert.assertTrue(space.hasClosest(gimpact, mesh));
        Assert.assertTrue(space.hasClosest(gimpact, plane));

        Assert.assertTrue(space.hasClosest(heightfield, box));
        Assert.assertTrue(space.hasClosest(heightfield, compound));
        Assert.assertTrue(space.hasClosest(heightfield, gimpact));
        Assert.assertFalse(space.hasClosest(heightfield, heightfield));
        Assert.assertFalse(space.hasClosest(heightfield, mesh));
        Assert.assertFalse(space.hasClosest(heightfield, plane));

        Assert.assertTrue(space.hasClosest(mesh, box));
        Assert.assertTrue(space.hasClosest(mesh, compound));
        Assert.assertTrue(space.hasClosest(mesh, gimpact));
        Assert.assertFalse(space.hasClosest(mesh, heightfield));
        Assert.assertFalse(space.hasClosest(mesh, mesh));
        Assert.assertFalse(space.hasClosest(mesh, plane));

        Assert.assertTrue(space.hasClosest(plane, box));
        Assert.assertTrue(space.hasClosest(plane, compound));
        Assert.assertTrue(space.hasClosest(plane, gimpact));
        Assert.assertFalse(space.hasClosest(plane, heightfield));
        Assert.assertFalse(space.hasClosest(plane, mesh));
        Assert.assertFalse(space.hasClosest(plane, plane));

        Assert.assertTrue(space.hasContact(box, box));
        Assert.assertTrue(space.hasContact(box, compound));
        Assert.assertTrue(space.hasContact(box, gimpact));
        Assert.assertTrue(space.hasContact(box, heightfield));
        Assert.assertTrue(space.hasContact(box, mesh));
        Assert.assertTrue(space.hasContact(box, plane));

        Assert.assertTrue(space.hasContact(gimpact, box));
        Assert.assertTrue(space.hasContact(gimpact, compound));
        Assert.assertTrue(space.hasContact(gimpact, gimpact));
        Assert.assertTrue(space.hasContact(gimpact, heightfield));
        Assert.assertTrue(space.hasContact(gimpact, mesh));
        Assert.assertTrue(space.hasContact(gimpact, plane));

        Assert.assertTrue(space.hasContact(heightfield, box));
        Assert.assertTrue(space.hasContact(heightfield, compound));
        Assert.assertTrue(space.hasContact(heightfield, gimpact));
        Assert.assertFalse(space.hasContact(heightfield, heightfield));
        Assert.assertFalse(space.hasContact(heightfield, mesh));
        Assert.assertFalse(space.hasContact(heightfield, plane));

        Assert.assertTrue(space.hasContact(mesh, box));
        Assert.assertTrue(space.hasContact(mesh, compound));
        Assert.assertTrue(space.hasContact(mesh, gimpact));
        Assert.assertFalse(space.hasContact(mesh, heightfield));
        Assert.assertFalse(space.hasContact(mesh, mesh));
        Assert.assertFalse(space.hasContact(mesh, plane));

        Assert.assertTrue(space.hasContact(plane, box));
        Assert.assertTrue(space.hasContact(plane, compound));
        Assert.assertTrue(space.hasContact(plane, gimpact));
        Assert.assertFalse(space.hasContact(plane, heightfield));
        Assert.assertFalse(space.hasContact(plane, mesh));
        Assert.assertFalse(space.hasContact(plane, plane));
    }

    /**
     * Test double-precision accessors.
     */
    @Test
    public void test013() {
        loadNativeLibrary();

        // Generate a subdivided square mesh with alternating diagonals.
        int numLines = 3;
        float lineSpacing = 0.1f; // mesh units
        IndexedMesh squareGrid
                = createClothGrid(numLines, numLines, lineSpacing);

        // Create a soft square and add it to the physics space.
        PhysicsSoftBody cloth = new PhysicsSoftBody();
        testPco(cloth);
        NativeSoftBodyUtil.appendFromNativeMesh(squareGrid, cloth);

        Vec3d xIn = new Vec3d(7.01234567, 6.01234567, 0.01234567);
        cloth.setPhysicsLocationDp(xIn);

        Vector3f xOut3 = cloth.getPhysicsLocation(null);
        Utils.assertEquals(7.01234567f, 6.01234567f, 0.01234567f, xOut3, 1e-6f);
        Vec3d xOut2 = cloth.getPhysicsLocationDp(null);
        if (NativeLibrary.isDoublePrecision()) {
            Utils.assertEquals(xIn.x, xIn.y, xIn.z, xOut2, 1e-15);
        } else {
            Utils.assertEquals(xIn.x, xIn.y, xIn.z, xOut2, 1e-6);
        }

        ConvexShape shape = new MultiSphere(0.1f);
        shape.setScale(new Vector3f(0.2f, 0.3f, 0.4f));
        Vec3d sc = shape.getScaleDp(null);
        Utils.assertEquals(0.2, 0.3, 0.4, sc, 1e-6);

        // Create a sphere-shaped character.
        PhysicsCharacter character = new PhysicsCharacter(shape, 1f);
        testPco(character);
        character.setPhysicsLocationDp(xIn);
        Vec3d xOut4 = character.getPhysicsLocationDp(null);
        if (NativeLibrary.isDoublePrecision()) {
            Assert.assertEquals(xIn, xOut4);
        } else {
            Utils.assertEquals(xIn.x, xIn.y, xIn.z, xOut4, 1e-6);
        }

        // Create a sphere-shaped ghost object.
        PhysicsGhostObject ghost = new PhysicsGhostObject(shape);
        testPco(ghost);
        ghost.setPhysicsLocationDp(xIn);
        Vec3d xOut1 = ghost.getPhysicsLocationDp(null);
        if (NativeLibrary.isDoublePrecision()) {
            Assert.assertEquals(xIn, xOut1);
        } else {
            Utils.assertEquals(xIn.x, xIn.y, xIn.z, xOut1, 1e-6);
        }

        Quatd qIn2 = new Quatd(-0.5, 0.5, 0.5, 0.5);
        ghost.setPhysicsRotationDp(qIn2);
        Quatd qOut2 = ghost.getPhysicsRotationDp(null);
        Utils.assertEquals(0.5, -0.5, -0.5, -0.5, qOut2, 0.);

        Matrix3d mIn = new Matrix3d(0., 0., 1., 1., 0., 0., 0., 1., 0.);
        ghost.setPhysicsRotationDp(mIn);
        Matrix3d mOut2 = ghost.getPhysicsRotationMatrixDp(null);
        Assert.assertEquals(mIn, mOut2);

        // Create a sphere-shaped dynamic rigid body.
        PhysicsRigidBody body = new PhysicsRigidBody(shape, 1f);
        testPco(body);
        Vec3d gIn = new Vec3d(9.01234567, 0.98765433, -0.01234567);
        Quatd qIn = new Quatd(7.01234567, 8.01234567, -1.01234567, -2.01234567);
        qIn.normalizeLocal();
        Vec3d vIn = new Vec3d(4.01234567, 2.01234567, 3.01234567);
        Vec3d wIn = new Vec3d(5.01234567, 1.01234567, 8.01234567);
        body.setGravityDp(gIn);
        body.setPhysicsRotationDp(qIn);
        body.setLinearVelocityDp(vIn);
        body.setAngularVelocityDp(wIn);
        body.setPhysicsLocationDp(xIn);

        Vec3d gOut = body.getGravityDp(null);
        Quatd qOut = body.getPhysicsRotationDp(null);
        Vec3d vOut = body.getLinearVelocityDp(null);
        Vec3d wOut = body.getAngularVelocityDp(null);
        Vec3d xOut = body.getPhysicsLocationDp(null);

        if (NativeLibrary.isDoublePrecision()) {
            Assert.assertEquals(gIn, gOut);
            Utils.assertEquals(qIn.x, qIn.y, qIn.z, qIn.w, qOut, 1e-16);
            Assert.assertEquals(vIn, vOut);
            Assert.assertEquals(wIn, wOut);
            Assert.assertEquals(xIn, xOut);
        } else {
            Utils.assertEquals(gIn.x, gIn.y, gIn.z, gOut, 1e-6);
            Utils.assertEquals(qIn.x, qIn.y, qIn.z, qIn.w, qOut, 1e-6);
            Utils.assertEquals(vIn.x, vIn.y, vIn.z, vOut, 1e-6);
            Utils.assertEquals(wIn.x, wIn.y, wIn.z, wOut, 1e-6);
            Utils.assertEquals(xIn.x, xIn.y, xIn.z, xOut, 1e-6);
        }

        body.setPhysicsRotationDp(mIn);
        Matrix3d mOut3 = body.getPhysicsRotationMatrixDp(null);
        Assert.assertEquals(mIn, mOut3);

        // Create a sphere-shaped base collider.
        int numLinks = 0;
        float baseMass = 1f;
        Vector3f baseInertia = Vector3f.UNIT_XYZ;
        boolean fixedBase = true;
        boolean canSleep = true;
        MultiBody multiBody = new MultiBody(
                numLinks, baseMass, baseInertia, fixedBase, canSleep);
        multiBody.addBaseCollider(shape);
        MultiBodyCollider collider = multiBody.getBaseCollider();
        testPco(collider);

        collider.setPhysicsLocationDp(xIn);
        collider.setPhysicsRotationDp(mIn);

        Matrix3d mOut5 = collider.getPhysicsRotationMatrixDp(null);
        Assert.assertEquals(mIn, mOut5);
        Quatd qOut5 = collider.getPhysicsRotationDp(null);
        Utils.assertEquals(0.5, 0.5, 0.5, 0.5, qOut5, 0.);
        Vec3d xOut5 = collider.getPhysicsLocationDp(null);
        if (NativeLibrary.isDoublePrecision()) {
            Assert.assertEquals(xIn, xOut5);
        } else {
            Utils.assertEquals(xIn.x, xIn.y, xIn.z, xOut5, 1e-6);
        }
    }

    /**
     * Test btTriangleShape::isInside().
     */
    @Test
    public void test014() {
        loadNativeLibrary();

        // near p0 (isosceles triangle)
        boolean isInside0 = NativeLibrary.isInsideTriangle(
                new Vector3f(10f, 0.1f, 0.1f), 0.2f,
                new Vector3f(10f, 0f, 0f),
                new Vector3f(-10f, 0f, -10f),
                new Vector3f(-10f, 0f, 10f));
        Assert.assertTrue(isInside0);

        // outside an acute corner p0 (isosceles triangle)
        boolean isInside1 = NativeLibrary.isInsideTriangle(
                new Vector3f(12.1f, 10f, 0f), 2f,
                new Vector3f(10f, 10f, 0f),
                new Vector3f(-10f, 10f, -5f),
                new Vector3f(-10f, 10f, 5f));
        Assert.assertFalse(isInside1);

        // near an interior point (isosceles triangle)
        boolean isInside2 = NativeLibrary.isInsideTriangle(
                new Vector3f(0f, 3.05f, 0f), 0.1f,
                new Vector3f(10f, 3f, 0f),
                new Vector3f(-10f, 3f, -10f),
                new Vector3f(-10f, 3f, 10f));
        Assert.assertTrue(isInside2);

        // near the hypotenuse (right triangle)
        boolean isInside3 = NativeLibrary.isInsideTriangle(
                new Vector3f(1f, 7f, -1f), 1.5f,
                new Vector3f(10f, 7f, 10f),
                new Vector3f(-10f, 7f, -10f),
                new Vector3f(-10f, 7f, 10f));
        Assert.assertTrue(isInside3);

        // outside the hypotenuse (right triangle)
        boolean isInside4 = NativeLibrary.isInsideTriangle(
                new Vector3f(1f, 4f, -1f), 1.4f,
                new Vector3f(10f, 4f, 10f),
                new Vector3f(-10f, 4f, -10f),
                new Vector3f(-10f, 4f, 10f));
        Assert.assertFalse(isInside4);

        // (degenerate triangle)
        boolean isInside5 = NativeLibrary.isInsideTriangle(
                new Vector3f(1f, 5f, -1f), 1.5f,
                new Vector3f(10f, 5f, 10f),
                new Vector3f(-10f, 5f, -10f),
                new Vector3f(-10f, 5f, -10f));
        Assert.assertTrue(isInside5);
    }

    /**
     * Test ManifoldPoints.
     */
    @Test
    public void test015() {
        loadNativeLibrary();

        Assert.assertTrue(ManifoldPoints.isContactCalcArea3Points());
        ManifoldPoints.setContactCalcArea3Points(false);
        Assert.assertFalse(ManifoldPoints.isContactCalcArea3Points());
        ManifoldPoints.setContactCalcArea3Points(true);

        long nativeId = ManifoldPoints.createTestPoint();
        Vector3f tmpVector = new Vector3f();

        // Verify that the test point is sufficiently zeroed.
        Assert.assertEquals(0, ManifoldPoints.getFlags(nativeId));
        Assert.assertEquals(0, ManifoldPoints.getLifeTime(nativeId));

        Assert.assertEquals(
                0f, ManifoldPoints.getAppliedImpulse(nativeId), 0f);
        Assert.assertEquals(
                0f, ManifoldPoints.getAppliedImpulseLateral1(nativeId), 0f);
        Assert.assertEquals(
                0f, ManifoldPoints.getAppliedImpulseLateral2(nativeId), 0f);
        Assert.assertEquals(
                0f, ManifoldPoints.getContactMotion1(nativeId), 0f);
        Assert.assertEquals(
                0f, ManifoldPoints.getContactMotion2(nativeId), 0f);

        // Invoke all the setters.
        ManifoldPoints.setAppliedImpulse(nativeId, 1f);
        ManifoldPoints.setAppliedImpulseLateral1(nativeId, 2f);
        ManifoldPoints.setAppliedImpulseLateral2(nativeId, 3f);
        ManifoldPoints.setCombinedFriction(nativeId, 4f);
        ManifoldPoints.setCombinedRestitution(nativeId, 5f);
        ManifoldPoints.setCombinedRollingFriction(nativeId, 6f);
        ManifoldPoints.setCombinedSpinningFriction(nativeId, 7f);
        ManifoldPoints.setContactMotion1(nativeId, 8f);
        ManifoldPoints.setContactMotion2(nativeId, 9f);
        ManifoldPoints.setDistance1(nativeId, 10f);
        ManifoldPoints.setFlags(nativeId, 11);
        ManifoldPoints
                .setLateralFrictionDir1(nativeId, new Vector3f(12f, 13f, 14f));
        ManifoldPoints
                .setLateralFrictionDir2(nativeId, new Vector3f(15f, 16f, 17f));
        ManifoldPoints.setLocalPointA(nativeId, new Vector3f(18f, 19f, 20f));
        ManifoldPoints.setLocalPointB(nativeId, new Vector3f(21f, 22f, 23f));
        ManifoldPoints.setNormalWorldOnB(nativeId, new Vector3f(24f, 25f, 26f));
        ManifoldPoints
                .setPositionWorldOnA(nativeId, new Vector3f(27f, 28f, 29f));
        ManifoldPoints
                .setPositionWorldOnB(nativeId, new Vector3f(30f, 31f, 32f));

        // Verify the resulting point.
        Assert.assertEquals(11, ManifoldPoints.getFlags(nativeId));
        Assert.assertEquals(0, ManifoldPoints.getLifeTime(nativeId));

        Assert.assertEquals(
                1f, ManifoldPoints.getAppliedImpulse(nativeId), 0f);
        Assert.assertEquals(
                2f, ManifoldPoints.getAppliedImpulseLateral1(nativeId), 0f);
        Assert.assertEquals(
                3f, ManifoldPoints.getAppliedImpulseLateral2(nativeId), 0f);
        Assert.assertEquals(
                4f, ManifoldPoints.getCombinedFriction(nativeId), 0f);
        Assert.assertEquals(
                5f, ManifoldPoints.getCombinedRestitution(nativeId), 0f);
        Assert.assertEquals(
                6f, ManifoldPoints.getCombinedRollingFriction(nativeId), 0f);
        Assert.assertEquals(
                7f, ManifoldPoints.getCombinedSpinningFriction(nativeId), 0f);
        Assert.assertEquals(
                8f, ManifoldPoints.getContactMotion1(nativeId), 0f);
        Assert.assertEquals(
                9f, ManifoldPoints.getContactMotion2(nativeId), 0f);
        Assert.assertEquals(
                10f, ManifoldPoints.getDistance1(nativeId), 0f);

        ManifoldPoints.getLateralFrictionDir1(nativeId, tmpVector);
        Utils.assertEquals(12f, 13f, 14f, tmpVector, 0f);
        ManifoldPoints.getLateralFrictionDir2(nativeId, tmpVector);
        Utils.assertEquals(15f, 16f, 17f, tmpVector, 0f);
        ManifoldPoints.getLocalPointA(nativeId, tmpVector);
        Utils.assertEquals(18f, 19f, 20f, tmpVector, 0f);
        ManifoldPoints.getLocalPointB(nativeId, tmpVector);
        Utils.assertEquals(21f, 22f, 23f, tmpVector, 0f);
        ManifoldPoints.getNormalWorldOnB(nativeId, tmpVector);
        Utils.assertEquals(24f, 25f, 26f, tmpVector, 0f);
        ManifoldPoints.getPositionWorldOnA(nativeId, tmpVector);
        Utils.assertEquals(27f, 28f, 29f, tmpVector, 0f);
        ManifoldPoints.getPositionWorldOnB(nativeId, tmpVector);
        Utils.assertEquals(30f, 31f, 32f, tmpVector, 0f);

        Vec3d tmpVec = new Vec3d();
        ManifoldPoints.getPositionWorldOnADp(nativeId, tmpVec);
        Utils.assertEquals(27., 28., 29., tmpVec, 0.);
        ManifoldPoints.getPositionWorldOnBDp(nativeId, tmpVec);
        Utils.assertEquals(30., 31., 32., tmpVec, 0.);
    }

    /**
     * Test SolverInfo.
     */
    @Test
    public void test016() {
        loadNativeLibrary();

        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        SolverInfo info = space.getSolverInfo();

        // Invoke all the setters.
        info.setContactErp(2f);
        info.setGlobalCfm(3f);
        info.setJointErp(4f);
        info.setMinBatch(5);
        info.setMode(0x6);
        info.setNumIterations(7);
        info.setSplitImpulseEnabled(false);
        info.setSplitImpulseErp(8f);
        info.setSplitImpulseThreshold(9f);

        // Verify the resulting SolverInfo.
        Assert.assertEquals(2f, info.contactErp(), 0f);
        Assert.assertEquals(3f, info.globalCfm(), 0f);
        Assert.assertEquals(4f, info.jointErp(), 0f);
        Assert.assertEquals(5, info.minBatch());
        Assert.assertEquals(0x6, info.mode());
        Assert.assertEquals(7, info.numIterations());
        Assert.assertFalse(info.isSplitImpulseEnabled());
        Assert.assertEquals(8f, info.splitImpulseErp(), 0f);
        Assert.assertEquals(9f, info.splitImpulseThreshold(), 0f);
    }

    /**
     * Test accessors for globals (default margin, deactivation deadline, and
     * deactivation enabled flag).
     */
    @Test
    public void test017() {
        loadNativeLibrary();

        // default margin for collision shapes
        float margin = CollisionShape.getDefaultMargin();
        Assert.assertEquals(0.04f, margin, 0f);

        CollisionShape.setDefaultMargin(2.1f);
        margin = CollisionShape.getDefaultMargin();
        Assert.assertEquals(2.1f, margin, 0f);

        // deactivation deadline
        float deadline = PhysicsBody.getDeactivationDeadline();
        Assert.assertEquals(2f, deadline, 0f);

        PhysicsBody.setDeactivationDeadline(17f);
        deadline = PhysicsBody.getDeactivationDeadline();
        Assert.assertEquals(17f, deadline, 0f);

        // deactivation enabled flag
        boolean enabled = PhysicsBody.isDeactivationEnabled();
        Assert.assertTrue(enabled);

        PhysicsBody.setDeactivationEnabled(false);
        enabled = PhysicsBody.isDeactivationEnabled();
        Assert.assertFalse(enabled);
    }

    /**
     * Test a single-ended New6Dof joint, in a pendulum.
     */
    @Test
    public void test018() {
        loadNativeLibrary();
        PhysicsSpace space = new PhysicsSpace(PhysicsSpace.BroadphaseType.DBVT);
        space.setAccuracy(0.001f);
        space.setMaxSubSteps(1001);
        space.getSolverInfo().setJointErp(1f);

        CollisionShape shape = new SphereCollisionShape(0.1f);
        PhysicsRigidBody b = new PhysicsRigidBody(shape, 1f);
        b.setPhysicsLocation(new Vector3f(4f, 0f, 0f));
        space.addCollisionObject(b);

        New6Dof p2p = new New6Dof(
                b, new Vector3f(-4f, 0f, 0f), new Vector3f(0f, 0f, 0f),
                new Matrix3f(), new Matrix3f(), RotationOrder.ZYX);
        space.addJoint(p2p);

        Assert.assertEquals(1, p2p.countEnds());
        Assert.assertTrue(p2p.checkRotationOrder());
        Utils.assertEquals(-4f, 0f, 0f, p2p.getPivotB(null), 0f);

        space.update(1f);
        Vector3f location = b.getMotionState().getLocation(null);
        Utils.assertEquals(1.572f, -3.68f, 0f, location, 0.01f);

        space.update(1f);
        b.getMotionState().getLocation(location);
        Utils.assertEquals(-3.944f, -0.666f, 0f, location, 0.01f);
    }

    /**
     * Test the RigidBodyMotionState defaults.
     */
    @Test
    public void test019() {
        loadNativeLibrary();
        RigidBodyMotionState state = new RigidBodyMotionState();

        Vector3f x = state.getLocation(null);
        Utils.assertEquals(0f, 0f, 0f, x, 0f);

        Vec3d xx = state.getLocationDp(null);
        Utils.assertEquals(0., 0., 0., xx, 0.);

        Matrix3f m = new Matrix3f();
        state.getOrientation(m);
        Assert.assertTrue(m.isIdentity());

        Quaternion q = new Quaternion();
        state.getOrientation(q);
        Utils.assertEquals(0f, 0f, 0f, 1f, q, 0f);

        Matrix3d mm = state.getOrientationMatrixDp(null);
        Assert.assertTrue(mm.isIdentity());

        Quatd qq = state.getOrientationQuaternionDp(null);
        Utils.assertEquals(0., 0., 0., 1., qq, 0.);

        Assert.assertFalse(state.isApplyPhysicsLocal());

        Transform t = state.physicsTransform(null);
        Assert.assertTrue(t.equals(Transform.IDENTITY));
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate a grid in the X-Z plane, centered on (0,0,0).
     *
     * @param xLines the desired number of grid lines parallel to the X axis
     * (&ge;2)
     * @param zLines the desired number of grid lines parallel to the Z axis
     * (&ge;2)
     * @param lineSpacing the desired initial distance between adjacent grid
     * lines (in mesh units, &gt;0)
     * @return a new IndexedMesh
     */
    private static IndexedMesh
            createClothGrid(int xLines, int zLines, float lineSpacing) {
        Validate.inRange(xLines, "X lines", 2, Integer.MAX_VALUE);
        Validate.inRange(zLines, "Z lines", 2, Integer.MAX_VALUE);
        Validate.positive(lineSpacing, "line spacing");

        int numVertices = xLines * zLines;
        Vector3f[] positionArray = new Vector3f[numVertices];

        // Write the vertex locations:
        int vectorIndex = 0;
        for (int xIndex = 0; xIndex < zLines; ++xIndex) {
            float x = (2 * xIndex - zLines + 1) * lineSpacing / 2f;
            for (int zIndex = 0; zIndex < xLines; ++zIndex) {
                float z = (2 * zIndex - xLines + 1) * lineSpacing / 2f;
                positionArray[vectorIndex] = new Vector3f(x, 0f, z);
                ++vectorIndex;
            }
        }
        assert vectorIndex == positionArray.length;

        int numTriangles = 2 * (xLines - 1) * (zLines - 1);
        int numIndices = vpt * numTriangles;
        int[] indexArray = new int[numIndices];

        // Write vertex indices for triangles:
        int intIndex = 0;
        for (int zIndex = 0; zIndex < xLines - 1; ++zIndex) {
            for (int xIndex = 0; xIndex < zLines - 1; ++xIndex) {
                // 4 vertices and 2 triangles forming a square
                int vi0 = zIndex + xLines * xIndex;
                int vi1 = vi0 + 1;
                int vi2 = vi0 + xLines;
                int vi3 = vi1 + xLines;
                if ((xIndex + zIndex) % 2 == 0) {
                    // major diagonal: joins vi1 to vi2
                    indexArray[intIndex] = vi0;
                    indexArray[intIndex + 1] = vi1;
                    indexArray[intIndex + 2] = vi2;

                    indexArray[intIndex + 3] = vi3;
                    indexArray[intIndex + 4] = vi2;
                    indexArray[intIndex + 5] = vi1;
                } else {
                    // minor diagonal: joins vi0 to vi3
                    indexArray[intIndex] = vi0;
                    indexArray[intIndex + 1] = vi1;
                    indexArray[intIndex + 2] = vi3;

                    indexArray[intIndex + 3] = vi3;
                    indexArray[intIndex + 4] = vi2;
                    indexArray[intIndex + 5] = vi0;
                }
                intIndex += 6;
            }
        }
        assert intIndex == indexArray.length;

        IndexedMesh result = new IndexedMesh(positionArray, indexArray);
        return result;
    }

    private static void loadNativeLibrary() {
        boolean fromDist = false;

        File directory;
        if (fromDist) {
            directory = new File("dist");
        } else {
            directory = new File("build/libs/bulletjme/shared");
        }

        boolean success = NativeLibraryLoader
                .loadLibbulletjme(fromDist, directory, "Debug", "SpMt");
        if (success) {
            Assert.assertFalse(NativeLibrary.isDoublePrecision());
            Assert.assertTrue(NativeLibrary.isThreadSafe());

        } else { // fallback to Sp-flavored library
            success = NativeLibraryLoader
                    .loadLibbulletjme(fromDist, directory, "Debug", "Sp");
            if (success) {
                Assert.assertFalse(NativeLibrary.isDoublePrecision());
                Assert.assertFalse(NativeLibrary.isThreadSafe());
            }
        }

        if (!success) { // fallback to Dp-flavored library
            success = NativeLibraryLoader
                    .loadLibbulletjme(fromDist, directory, "Debug", "Dp");
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

    /**
     * Perform drop tests using the specified shape and broadphase.
     *
     * @param dropShape the shape to drop (not null)
     * @param broadphase the broadphase accelerator to use (not null)
     */
    private static void performDropTests(
            CollisionShape dropShape, PhysicsSpace.BroadphaseType broadphase) {
        performDropTests(dropShape, broadphase, SolverType.Dantzig);
        performDropTests(dropShape, broadphase, SolverType.Lemke);
        performDropTests(dropShape, broadphase, SolverType.NNCG);
        performDropTests(dropShape, broadphase, SolverType.PGS);
        performDropTests(dropShape, broadphase, SolverType.SI);
    }

    /**
     * Perform drop tests using the specified shape, broadphase, and solver.
     *
     * @param dropShape the shape to drop (not null)
     * @param broadphase the broadphase accelerator to use (not null)
     * @param solver the contact-and-constraint solver to use (not null)
     */
    private static void performDropTests(CollisionShape dropShape,
            PhysicsSpace.BroadphaseType broadphase, SolverType solver) {

        Vector3f min = new Vector3f(-10f, -10f, -10f);
        Vector3f max = new Vector3f(10f, 10f, 10f);
        PhysicsSpace space;

        space = new PhysicsSpace(min, max, broadphase, solver) {
            @Override
            public void onContactProcessed(PhysicsCollisionObject a,
                    PhysicsCollisionObject b, long manifoldPointId) {
                Assert.assertTrue(a == floor && b == drop
                        || a == drop && b == floor);
                dropAndFloorHaveCollided = true;
            }
        };
        performDropTest(dropShape, space);

        if (solver == SolverType.SI) {
            space = new PhysicsSoftSpace(broadphase) {
                @Override
                public void onContactProcessed(PhysicsCollisionObject a,
                        PhysicsCollisionObject b, long manifoldPointId) {
                    Assert.assertTrue(a == floor && b == drop
                            || a == drop && b == floor);
                    dropAndFloorHaveCollided = true;
                }
            };
            performDropTest(dropShape, space);
        }

        if (solver != SolverType.NNCG) {
            space = new MultiBodySpace(min, max, broadphase, solver) {
                @Override
                public void onContactProcessed(PhysicsCollisionObject a,
                        PhysicsCollisionObject b, long manifoldPointId) {
                    Assert.assertTrue(a == floor && b == drop
                            || a == drop && b == floor);
                    dropAndFloorHaveCollided = true;
                }
            };
            performDropTest(dropShape, space);

            space = new DeformableSpace(min, max, broadphase, solver) {
                @Override
                public void onContactProcessed(PhysicsCollisionObject a,
                        PhysicsCollisionObject b, long manifoldPointId) {
                    Assert.assertTrue(a == floor && b == drop
                            || a == drop && b == floor);
                    dropAndFloorHaveCollided = true;
                }
            };
            performDropTest(dropShape, space);
        }

        space = null;
        System.gc();
    }

    /**
     * Perform a drop test using the specified shape in the specified space.
     *
     * @param dropShape the shape to drop (not null)
     * @param space the space in which to perform the test (not null, modified)
     */
    private static void
            performDropTest(CollisionShape dropShape, PhysicsSpace space) {
        verifyPhysicsSpaceDefaults(space);

        if (space.getSolverType() == SolverType.Lemke) {
            space.getSolverInfo().setGlobalCfm(0.001f);
        }

        // Add a static horizontal plane at y=-1.
        Plane plane = new Plane(Vector3f.UNIT_Y, -1f);
        CollisionShape floorShape = new PlaneCollisionShape(plane);
        final PhysicsRigidBody floorBody
                = new PhysicsRigidBody(floorShape, PhysicsBody.massForStatic);

        testPco(floorBody);
        Assert.assertTrue(floorBody.isContactResponse());
        Assert.assertFalse(floorBody.isDynamic());
        Assert.assertFalse(floorBody.isKinematic());
        Assert.assertTrue(floorBody.isStatic());

        space.addCollisionObject(floorBody);

        Assert.assertSame(space, floorBody.getCollisionSpace());
        Assert.assertEquals(space.nativeId(), floorBody.spaceId());
        Assert.assertTrue(floorBody.isInWorld());

        Assert.assertFalse(space.isEmpty());
        Assert.assertEquals(1, space.countCollisionObjects());
        Assert.assertEquals(1, space.countRigidBodies());
        Assert.assertTrue(space.contains(floorBody));

        // Add a dynamic rigid body at y=0.
        float mass = 1f;
        final PhysicsRigidBody dropBody = new PhysicsRigidBody(dropShape, mass);

        testPco(dropBody);
        Assert.assertTrue(dropBody.isContactResponse());
        Assert.assertTrue(dropBody.isDynamic());
        Assert.assertFalse(dropBody.isKinematic());
        Assert.assertFalse(dropBody.isStatic());

        space.addCollisionObject(dropBody);

        Assert.assertSame(space, dropBody.getCollisionSpace());
        Assert.assertEquals(space.nativeId(), dropBody.spaceId());
        Assert.assertTrue(dropBody.isInWorld());

        Assert.assertFalse(space.isEmpty());
        Assert.assertEquals(2, space.countCollisionObjects());
        Assert.assertEquals(2, space.countRigidBodies());
        Assert.assertTrue(space.contains(floorBody));
        Assert.assertTrue(space.contains(dropBody));

        if (space instanceof PhysicsSpace) {
            dropAndFloorHaveCollided = false;
            drop = dropBody;
            floor = floorBody;
        }

        // 50 iterations with a 20-msec timestep
        for (int i = 0; i < 50; ++i) {
            space.update(0.02f, 0, false, true, false);
        }

        if (space instanceof PhysicsSpace) {
            Assert.assertTrue(dropAndFloorHaveCollided);
        }

        // Check the final location of the dynamic body.
        Vector3f location = dropBody.getPhysicsLocation(null);
        Assert.assertEquals(0f, location.x, 0.2f);
        Assert.assertEquals(0f, location.z, 0.2f);

        space.removeCollisionObject(floorBody);

        Assert.assertNull(floorBody.getCollisionSpace());
        Assert.assertTrue(floorBody.isContactResponse());
        Assert.assertFalse(floorBody.isDynamic());
        Assert.assertFalse(floorBody.isInWorld());
        Assert.assertFalse(floorBody.isKinematic());
        Assert.assertTrue(floorBody.isStatic());
        Assert.assertEquals(0L, floorBody.spaceId());

        space.removeCollisionObject(dropBody);

        Assert.assertNull(dropBody.getCollisionSpace());
        Assert.assertTrue(dropBody.isContactResponse());
        Assert.assertTrue(dropBody.isDynamic());
        Assert.assertFalse(dropBody.isInWorld());
        Assert.assertFalse(dropBody.isKinematic());
        Assert.assertFalse(dropBody.isStatic());
        Assert.assertEquals(0L, dropBody.spaceId());

        Assert.assertTrue(space.isEmpty());
    }

    /**
     * Perform ray tests against the specified shape in the specified space.
     *
     * @param shape the shape to add to the space (not null)
     * @param space the space in which to perform the tests (not null, modified)
     */
    private static void
            performRayTests(CollisionShape shape, CollisionSpace space) {
        Assert.assertTrue(space.isEmpty());

        PhysicsGhostObject ghost = new PhysicsGhostObject(shape);

        testPco(ghost);
        Assert.assertFalse(ghost.isContactResponse());
        Assert.assertTrue(ghost.isStatic());

        space.addCollisionObject(ghost);
        Assert.assertEquals(1, ghost.proxyGroup().intValue());
        Assert.assertEquals(-1, ghost.proxyMask().intValue());

        Assert.assertSame(space, ghost.getCollisionSpace());
        Assert.assertEquals(space.nativeId(), ghost.spaceId());
        Assert.assertTrue(ghost.isInWorld());
        Assert.assertTrue(space.contains(ghost));
        Assert.assertEquals(1, space.countCollisionObjects());

        List<PhysicsRayTestResult> results0 = space.rayTest(
                new Vector3f(0.8f, 0.8f, 2f), new Vector3f(0.8f, 0.8f, 0f));
        Assert.assertEquals(0, results0.size());

        List<PhysicsRayTestResult> results1 = space.rayTest(
                new Vector3f(0.7f, 0.7f, 2f), new Vector3f(0.7f, 0.7f, 0f));
        Assert.assertEquals(1, results1.size());

        List<PhysicsRayTestResult> results2 = space.rayTest(
                new Vector3f(0.7f, 0.7f, 2f), new Vector3f(0.7f, 0.7f, -2f));
        Assert.assertEquals(1, results2.size());

        List<PhysicsRayTestResult> results0d = space.rayTestDp(
                new Vec3d(0.8, 0.8, 2.), new Vec3d(0.8, 0.8, 0.),
                new ArrayList<>(2));
        Assert.assertEquals(0, results0d.size());

        List<PhysicsRayTestResult> results1d = space.rayTestDp(
                new Vec3d(0.7, 0.7, 2.), new Vec3d(0.7, 0.7, 0.),
                new ArrayList<>(2));
        Assert.assertEquals(1, results1d.size());

        List<PhysicsRayTestResult> results2d = space.rayTestDp(
                new Vec3d(0.7f, 0.7, 2.), new Vec3d(0.7, 0.7, -2.),
                new ArrayList<>(2));
        Assert.assertEquals(1, results2d.size());

        space.removeCollisionObject(ghost);

        Assert.assertNull(ghost.getCollisionSpace());
        Assert.assertEquals(0L, ghost.spaceId());
        Assert.assertFalse(ghost.isInWorld());
        Assert.assertFalse(space.contains(ghost));
        Assert.assertEquals(0, space.countCollisionObjects());
        Assert.assertTrue(space.isEmpty());

        List<PhysicsRayTestResult> results3 = space.rayTest(
                new Vector3f(0.7f, 0.7f, 2f), new Vector3f(0.7f, 0.7f, -2f));
        Assert.assertEquals(0, results3.size());
    }

    /**
     * Test the defaults that are common to all newly-created collision objects.
     *
     * @param pco the object to test (not null, unaffected)
     */
    private static void testPco(PhysicsCollisionObject pco) {
        long nativeId = pco.nativeId();
        Assert.assertNotEquals(0L, nativeId);
        Assert.assertEquals(pco, PhysicsCollisionObject.findInstance(nativeId));
        Assert.assertEquals(0, pco.countIgnored());
        Assert.assertNull(pco.getCollisionSpace());
        Assert.assertFalse(pco.isInWorld());
        Assert.assertEquals(0L, pco.spaceId());

        Assert.assertTrue(pco.isActive());
        Utils.assertEquals(1f, 1f, 1f, pco.getAnisotropicFriction(null), 0f);
        Assert.assertFalse(pco.hasAnisotropicFriction(1));
        Assert.assertFalse(pco.hasAnisotropicFriction(2));
        Assert.assertEquals(0f, pco.getCcdMotionThreshold(), 0f);
        Assert.assertEquals(0f, pco.getCcdSweptSphereRadius(), 0f);
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                pco.getCollideWithGroups());
        Assert.assertEquals(PhysicsCollisionObject.COLLISION_GROUP_01,
                pco.getCollisionGroup());
        Assert.assertEquals(0.1f, pco.getContactDamping(), 0f);

        float largeFloat = NativeLibrary.isDoublePrecision() ? 1e30f : 1e18f;
        Assert.assertEquals(
                largeFloat, pco.getContactProcessingThreshold(), 0f);
        Assert.assertEquals(largeFloat, pco.getContactStiffness(), 0f);

        Assert.assertEquals(0f, pco.getDeactivationTime(), 0f);
        Assert.assertEquals(0.5f, pco.getFriction(), 0f);

        Utils.assertEquals(0f, 0f, 0f, pco.getPhysicsLocation(null), 0f);
        Utils.assertEquals(0., 0., 0., pco.getPhysicsLocationDp(null), 0.);
        Utils.assertEquals(0f, 0f, 0f, 1f, pco.getPhysicsRotation(null), 0f);
        Utils.assertEquals(0., 0., 0., 1., pco.getPhysicsRotationDp(null), 0.);
        Assert.assertEquals(
                Matrix3f.IDENTITY, pco.getPhysicsRotationMatrix(null));
        Assert.assertEquals(
                new Matrix3d(), pco.getPhysicsRotationMatrixDp(null));

        Assert.assertNull(pco.proxyGroup());
        Assert.assertNull(pco.proxyMask());
        Assert.assertEquals(0f, pco.getRestitution(), 0f);
        Assert.assertEquals(0f, pco.getRollingFriction(), 0f);
        Assert.assertEquals(0f, pco.getSpinningFriction(), 0f);
        Assert.assertNull(pco.getUserObject());

        if (pco instanceof PhysicsRigidBody) {
            PhysicsRigidBody body = (PhysicsRigidBody) pco;

            Assert.assertEquals(0, body.countJoints());
            Assert.assertEquals(Activation.active, body.getActivationState());
            Assert.assertEquals(0f, body.getAngularDamping(), 0f);
            Utils.assertEquals(1f, 1f, 1f, body.getAngularFactor(null), 0f);
            Assert.assertEquals(1f, body.getAngularSleepingThreshold(), 0f);
            Utils.assertEquals(0f, 0f, 0f, body.getGravity(null), 0f);
            Utils.assertEquals(0., 0., 0., body.getGravityDp(null), 0.);
            Assert.assertEquals(0f, body.getLinearDamping(), 0f);
            Utils.assertEquals(1f, 1f, 1f, body.getLinearFactor(null), 0f);
            Assert.assertEquals(0.8f, body.getLinearSleepingThreshold(), 0f);
            Assert.assertTrue(body.isContactResponse());
            Assert.assertFalse(body.isGravityProtected());
            Assert.assertFalse(body.isKinematic());
            Assert.assertEquals(0, body.listJoints().length);
            Utils.assertEquals(0f, 0f, 0f, body.totalAppliedForce(null), 0f);
            Utils.assertEquals(0f, 0f, 0f, body.totalAppliedTorque(null), 0f);
            if (body.isDynamic()) {
                Utils.assertEquals(
                        0f, 0f, 0f, body.getAngularVelocity(null), 0f);
                Utils.assertEquals(
                        0., 0., 0., body.getAngularVelocityDp(null), 0.);
                Utils.assertEquals(
                        0f, 0f, 0f, body.getAngularVelocityLocal(null), 0f);
                Utils.assertEquals(
                        0f, 0f, 0f, body.getLinearVelocity(null), 0f);
                Utils.assertEquals(
                        0., 0., 0., body.getLinearVelocityDp(null), 0.);
                Assert.assertEquals(0f, body.getSquaredSpeed(), 0f);
                Assert.assertFalse(body.isStatic());
                Assert.assertEquals(0f, body.kineticEnergy(), 0f);
                Assert.assertEquals(0f, body.mechanicalEnergy(), 0f);
            } else {
                Assert.assertEquals(0f, body.getMass(), 0f);
                Assert.assertTrue(body.isStatic());
            }

        } else if (pco instanceof MultiBodyCollider) {
            MultiBodyCollider collider = (MultiBodyCollider) pco;
            Assert.assertNotNull(collider.getMultiBody());
        }
    }

    /**
     * Verify defaults common to all newly-created collision shapes.
     *
     * @param shape the shape to test (not null, unaffected)
     */
    private static void verifyCollisionShapeDefaults(CollisionShape shape) {
        Assert.assertNotNull(shape);
        Assert.assertNotEquals(0L, shape.nativeId());
        Utils.assertEquals(1f, 1f, 1f, shape.getScale(null), 0f);
        Utils.assertEquals(1., 1., 1., shape.getScaleDp(null), 0.);
        Assert.assertTrue(shape.isContactFilterEnabled());
    }

    /**
     * Verify defaults common to all newly-created collision spaces.
     *
     * @param space the space to test (not null, unaffected)
     */
    private static void verifyCollisionSpaceDefaults(CollisionSpace space) {
        Assert.assertNotNull(space);
        Assert.assertTrue(space.isEmpty());
        Assert.assertEquals(0, space.countCollisionObjects());
        Assert.assertEquals(
                RayTestFlag.SubSimplexRaytest, space.getRayTestFlags());
        Assert.assertTrue(space.isForceUpdateAllAabbs());
        Assert.assertFalse(space.isUsingDeterministicDispatch());
    }

    /**
     * Verify defaults common to all newly-created physics spaces.
     *
     * @param space the space to test (not null, unaffected)
     */
    private static void verifyPhysicsSpaceDefaults(PhysicsSpace space) {
        verifyCollisionSpaceDefaults(space);

        Assert.assertEquals(0, space.countCollisionListeners());
        Assert.assertEquals(0, space.countJoints());
        Assert.assertEquals(0, space.countManifolds());
        Assert.assertEquals(0, space.countRigidBodies());
        Assert.assertEquals(1 / 60f, space.getAccuracy(), 0f);
        Utils.assertEquals(0f, -9.81f, 0f, space.getGravity(null), 0f);
        Assert.assertFalse(space.isCcdWithStaticOnly());
        Assert.assertFalse(space.isUsingScr());
        Assert.assertEquals(0, space.listManifoldIds().length);
        Assert.assertEquals(4, space.maxSubSteps());
        Assert.assertEquals(0.1f, space.maxTimeStep(), 0f);

        SolverInfo info = space.getSolverInfo();
        Assert.assertNotNull(info);
        Assert.assertNotEquals(0L, info.nativeId());
        Assert.assertEquals(0.2f, info.contactErp(), 0f);
        Assert.assertEquals(0f, info.globalCfm(), 0f);
        Assert.assertEquals(0.2f, info.jointErp(), 0f);
        Assert.assertEquals(128, info.minBatch());

        int expectedMode = (space instanceof MultiBodySpace) ? 0x114 : 0x104;
        Assert.assertEquals(expectedMode, info.mode());

        Assert.assertEquals(10, info.numIterations());
        Assert.assertTrue(info.isSplitImpulseEnabled());
        Assert.assertEquals(0.1, info.splitImpulseErp(), 1e-6f);
        Assert.assertEquals(-0.04, info.splitImpulseThreshold(), 1e-7f);

        if (space instanceof MultiBodySpace) {
            MultiBodySpace mbSpace = (MultiBodySpace) space;
            Assert.assertEquals(0, mbSpace.countMultiBodies());

        } else if (space instanceof PhysicsSoftSpace) {
            PhysicsSoftSpace softSpace = (PhysicsSoftSpace) space;
            Assert.assertEquals(0, softSpace.countSoftBodies());
        }
    }

    /**
     * Verify defaults common to all newly-created simplex shapes.
     *
     * @param simplex the shape to test (not null, unaffected)
     */
    private static void verifySimplexDefaults(SimplexCollisionShape simplex) {
        verifyCollisionShapeDefaults(simplex);
        Assert.assertEquals(0.04f, simplex.getMargin(), 0f);

        Assert.assertFalse(simplex.isConcave());
        Assert.assertTrue(simplex.isConvex());
        Assert.assertFalse(simplex.isInfinite());
        Assert.assertFalse(simplex.isNonMoving());
        Assert.assertTrue(simplex.isPolyhedral());
    }
}
