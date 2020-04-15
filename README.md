<img height="150" src="https://i.imgur.com/YEPFEcx.png">

The [Libbulletjme Project][libbulletjme] adds JNI "glue code"
to portions of [Bullet Physics][bullet]
and [Khaled Mamou's V-HACD Library][vhacd],
enabling 3-D physics simulation from Java applications.

Complete source code (in C++ and Java) is provided under
[a mixed open-source license][license].

The project supports the 3 major desktop operating systems:
Windows, Linux, and macOS.  Both the x86 and x86-64 architectures
are supported for each operating system.
It also supports Linux on the 64-bit ARM architecture (aarch64)
and provides native libraries for the 4 supported Android ABIs
(armeabi-v7a, arm64-v8a, x86, and x86_64)
making a total of 11 platforms.

For each desktop platform, 4 native libraries are distributed:
 + a release build using single-precision arithmetic (the default library)
 + a release build using double-precision arithmetic
 + a debug build using single-precision arithmetic
 + a debug build using double-precision arithmetic

Libbulletjme's native libraries are used in [Minie],
which integrates Libbulletjme into [the jMonkeyEngine game engine][jme].
For applications that don't use jMonkeyEngine,
standalone Maven artifacts are provided.

<a name="toc"/>

## Contents of this document

 + [How to add Libbulletjme to an existing project](#add)
 + [Example applications](#examples)
 + [How to build Libbulletjme from source](#build)
 + [Lexicon of class/enum/struct names](#lexicon)
 + [What's missing](#todo)
 + [External links](#links)
 + [History](#history)
 + [Acknowledgments](#acks)

<a name="add"/>

## How to add Libbulletjme to an existing project

 1. For projects built using Gradle, add the following dependency:

        repositories {
            jcenter()
        }
        dependencies {
            compile 'com.github.stephengold:Libbulletjme:5.6.0
        }

 2. Download appropriate native libraries from [GitHub][latest].
    You probably don't need all 36 native libraries.
    Start with the ReleaseSp library for your development environment
    (for instance, "Linux64ReleaseSp_libbulletjme.so" for Linux on x86_64).

 3. Load the native library:

        import com.jme3.system.NativeLibraryLoader;
        NativeLibraryLoader.loadLibbulletjme(true, downloadDirectory, "Release", "Sp");

<a name="examples"/>

## Example applications

### HelloLibbulletjme: drop a dynamic sphere onto a horizontal surface

```java
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import java.io.File;

public class HelloLibbulletjme {

    public static void main(String[] args) {
        /*
         * Load a native library from ~/Downloads directory.
         */
        String homePath = System.getProperty("user.home");
        File downloadDirectory = new File(homePath, "Downloads");
        NativeLibraryLoader.loadLibbulletjme(true, downloadDirectory, "Release", "Sp");
        /*
         * Create a 20x20x20 PhysicsSpace using DBVT for broadphase.
         */
        Vector3f min = new Vector3f(-10f, -10f, -10f);
        Vector3f max = new Vector3f(10f, 10f, 10f);
        PhysicsSpace.BroadphaseType bPhase = PhysicsSpace.BroadphaseType.DBVT;
        PhysicsSpace space = new PhysicsSpace(min, max, bPhase);
        /*
         * Add a static horizontal plane at y=-1.
         */
        float planeY = -1;
        Plane plane = new Plane(Vector3f.UNIT_Y, planeY);
        CollisionShape planeShape = new PlaneCollisionShape(plane);
        float mass = PhysicsBody.massForStatic;
        PhysicsRigidBody floor = new PhysicsRigidBody(planeShape, mass);
        space.addCollisionObject(floor);
        /*
         * Add a sphere-shaped, dynamic, rigid body at the origin.
         */
        float radius = 0.3f;
        CollisionShape ballShape = new SphereCollisionShape(radius);
        mass = 1f;
        PhysicsRigidBody ball = new PhysicsRigidBody(ballShape, mass);
        space.addCollisionObject(ball);
        /*
         * 50 iterations with a 20-msec timestep
         */
        float timeStep = 0.02f;
        for (int i = 0; i < 50; ++i) {
            space.update(timeStep, 0);
            Vector3f location = ball.getPhysicsLocation();
            System.out.println(location);
        }
    }
}
```

### HelloVehicle: drive a vehicle on a horizontal surface

```java
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.objects.PhysicsVehicle;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import java.io.File;

public class HelloVehicle {

    public static void main(String[] args) {
        /*
         * Load a native library from ~/Downloads directory.
         */
        String homePath = System.getProperty("user.home");
        File downloadDirectory = new File(homePath, "Downloads");
        NativeLibraryLoader.loadLibbulletjme(true, downloadDirectory, "Release", "Sp");
        /*
         * Create a 20x20x200 PhysicsSpace using DBVT for broadphase.
         */
        Vector3f min = new Vector3f(-10f, -10f, -100f);
        Vector3f max = new Vector3f(10f, 10f, 100f);
        PhysicsSpace.BroadphaseType bPhase = PhysicsSpace.BroadphaseType.DBVT;
        PhysicsSpace space = new PhysicsSpace(min, max, bPhase);
        /*
         * Add a static horizontal plane at y=-1.
         */
        float planeY = -1f;
        Plane plane = new Plane(Vector3f.UNIT_Y, planeY);
        CollisionShape planeShape = new PlaneCollisionShape(plane);
        float mass = PhysicsBody.massForStatic;
        PhysicsRigidBody floor = new PhysicsRigidBody(planeShape, mass);
        space.addCollisionObject(floor);
        /*
         * Add a vehicle with a boxy chassis.
         */
        CompoundCollisionShape chassisShape = new CompoundCollisionShape();
        BoxCollisionShape box = new BoxCollisionShape(1.2f, 0.5f, 2.4f);
        chassisShape.addChildShape(box, 0f, 1f, 0f);
        mass = 400f;
        PhysicsVehicle vehicle = new PhysicsVehicle(chassisShape, mass);
        vehicle.setMaxSuspensionForce(9e9f);
        vehicle.setSuspensionCompression(4f);
        vehicle.setSuspensionDamping(6f);
        vehicle.setSuspensionStiffness(50f);
        /*
         * Add 4 wheels, 2 in front (for steering) and 2 in back.
         */
        Vector3f axleDirection = new Vector3f(-1, 0, 0);
        Vector3f suspensionDirection = new Vector3f(0, -1, 0);
        float restLength = 0.3f;
        float radius = 0.5f;
        float xOffset = 1f;
        float yOffset = 0.5f;
        float zOffset = 2f;
        vehicle.addWheel(new Vector3f(-xOffset, yOffset, zOffset),
                suspensionDirection, axleDirection, restLength, radius,
                true);
        vehicle.addWheel(new Vector3f(xOffset, yOffset, zOffset),
                suspensionDirection, axleDirection, restLength, radius,
                true);
        vehicle.addWheel(new Vector3f(-xOffset, yOffset, -zOffset),
                suspensionDirection, axleDirection, restLength, radius,
                false);
        vehicle.addWheel(new Vector3f(xOffset, yOffset, -zOffset),
                suspensionDirection, axleDirection, restLength, radius,
                false);

        space.add(vehicle);
        vehicle.accelerate(500f);
        /*
         * 150 iterations with a 20-msec timestep
         */
        float timeStep = 0.02f;
        for (int i = 0; i < 150; ++i) {
            space.update(timeStep, 0);
            Vector3f location = vehicle.getPhysicsLocation();
            System.out.println(location);
        }
    }
```

<a name="build"/>

## How to build Libbulletjme from source

 1. Install build software:
   + one of the supported C++ compilers:
     + for Linux:  the [GNU Compiler Collection][gcc] or [Clang][llvm]
     + for Windows:  Microsoft Visual Studio
     + for macOS:  Xcode
   + a Java Development Kit, and
   + [Gradle]
 2. Download and extract the source code from GitHub:
   + using Git:
     + `git clone https://github.com/stephengold/Libbulletjme.git`
     + `cd Libbulletjme`
     + `git checkout -b latest 5.6.0`
   + using a web browser:
     + browse to [the latest release](https://github.com/stephengold/Libbulletjme/releases/latest)
     + follow the "Source code (zip)" link
     + save the ZIP file
     + unzip the saved ZIP file
     + `cd` to the extracted directory/folder
 3. Set the `JAVA_HOME` environment variable:
   + using Bash:  `export JAVA_HOME="` *path to your JDK* `"`
   + using Windows Command Prompt:  `set JAVA_HOME="` *path to your JDK* `"`
 4. Run the Gradle wrapper on the desktop build script:
   + using Bash:  `./gradlew build`
   + using Windows Command Prompt:  `.\gradlew build`
 5. Building Android native libraries requires additional software:
   + the Android SDK Tools
   + the Android SDK Patch Applier (patcher)
   + version 21.0.6113669 of the Android Native Development Kit (NDK)
 6. Run the Gradle wrapper on the Android build script:
   + using Bash:  `./gradlew copyToDist --build-file=android.gradle`
   + using Windows Command Prompt:  `.\gradlew copyToDist --build-file=android.gradle`

After a successful build, Maven artifacts and native libraries will be found
in the `dist` directory/folder.

You can also install the Maven artifacts to your local cache:
 + using Bash:  `./gradlew :Libbulletjme:publishToMavenLocal`
 + using Windows Command Prompt:  `.\gradlew :Libbulletjme:publishToMavenLocal`

<a name="lexicon"/>

## Lexicon of class/enum/struct names

<pre>
Bullet v2 C++ type:                     corresponding Java class: com.jme3...
===================                     =====================================
btBox2dShape                            .bullet.collision.shapes.Box2dShape
btBoxShape                              .bullet.collision.shapes.BoxCollisionShape
btBU_Simplex1to4                        .bullet.collision.shapes.SimplexCollisionShape
btBvhTriangleMeshShape                  .bullet.collision.shapes.MeshCollisionShape
btCapsuleShape                          .bullet.collision.shapes.CapsuleCollisionShape
btCollisionObject                       .bullet.collision.PhysicsCollisionObject
btCollisionObject::CollisionFlags       .bullet.collision.CollisionFlag
btCollisionObject::CollisionObjectTypes .bullet.collision.PcoType
btCollisionShape                        .bullet.collision.shapes.CollisionShape
btCollisionWorld                        .bullet.CollisionSpace
btCollisionWorld::LocalConvexResult     .bullet.collision.PhysicsSweepTestResult
btCollisionWorld::LocalRayResult        .bullet.collision.PhysicsRayTestResult
btCompoundShape                         .bullet.collision.shapes.CompoundCollisionShape
btCompoundShapeChild                    .bullet.collision.shapes.info.ChildCollisionShape
btConeShape                             .bullet.collision.shapes.ConeCollisionShape
btConeTwistConstraint                   .bullet.joints.ConeJoint
btConstraintParams                      .bullet.joints.motors.MotorParam
btContactPointFlags                     .bullet.collision.ContactPointFlag
btContactSolverInfo                     .bullet.SolverInfo
btConvex2dShape                         .bullet.collision.shapes.Convex2dShape
btConvexHullShape                       .bullet.collision.shapes.HullCollisionShape
btConvexShape                           .bullet.collision.shapes.ConvexShape
btCylinderShape                         .bullet.collision.shapes.CylinderCollisionShape
btDynamicsWorld                         .bullet.PhysicsSpace
btEmptyShape                            .bullet.collision.shapes.EmptyShape
btGeneric6DofConstraint                 .bullet.joints.SixDofJoint
btGeneric6DofSpring2Constraint          .bullet.joints.New6Dof
btGeneric6DofSpringConstraint           .bullet.joints.SixDofSpringJoint
btGImpactMeshShape                      .bullet.collision.shapes.GImpactCollisionShape
btHeightfieldTerrainShape               .bullet.collision.shapes.HeightfieldCollisionShape
btHingeConstraint                       .bullet.joints.HingeJoint
btIndexedMesh                           .bullet.collision.shapes.infos.IndexedMesh
btKinematicCharacterController          .bullet.objects.PhysicsCharacter
btManifoldPoint                         .bullet.collision.PhysicsCollisionEvent
btMatrix3x3                             .math.Matrix3f
btMultiBody                             .bullet.MultiBody
btMultiBodyCollider                     .bullet.objects.MultiBodyCollider
btMultiBodyLink                         .bullet.MultiBodyLink
btMultiBodyLink::eFeatherstoneJointType .bullet.MultiBodyJointType
btMultiBodyDynamicsWorld                .bullet.MultiBodySpace
btMultiSphereShape                      .bullet.collision.shapes.MultiSphere
btPairCachingGhostObject                .bullet.objects.PhysicsGhostObject
btPoint2PointConstraint                 .bullet.joints.Point2PointJoint
btQuaternion                            .math.Quaternion
btRaycastVehicle                        .bullet.objects.PhysicsVehicle
btRaycastVehicle::btVehicleTuning       .bullet.objects.infos.VehicleTuning
btRigidBody                             .bullet.objects.PhysicsRigidBody
btRotationalLimitMotor                  .bullet.joints.motors.RotationalLimitMotor
btRotationalLimitMotor2                 .bullet.joints.motors.RotationMotor
btSliderConstraint                      .bullet.joints.SliderJoint
btSoftBody                              .bullet.objects.PhysicsSoftBody
btSoftBody::AJoint                      .bullet.joints.SoftAngularJoint
btSoftBody::Anchor                      .bullet.joints.Anchor
btSoftBody::Body                        .bullet.object.PhysicsBody
btSoftBody::Config                      .bullet.objects.infos.SoftBodyConfig
btSoftBody::eAeroModel                  .bullet.objects.infos.Aero
btSoftBody::Joint                       .bullet.joints.SoftPhysicsJoint
btSoftBody::LJoint                      .bullet.joints.SoftLinearJoint
btSoftBodyWorldInfo                     .bullet.SoftBodyWorldInfo
btSoftRigidDynamicsWorld                .bullet.PhysicsSoftSpace
btSolverMode                            .bullet.SolverMode
btSphereShape                           .bullet.collision.shapes.SphereCollisionShape
btStaticPlaneShape                      .bullet.collision.shapes.PlaneCollisionShape
btTransform                             .math.Transform
btTranslationalLimitMotor               .bullet.joints.motors.TranslationalLimitMotor
btTranslationalLimitMotor2              .bullet.joints.motors.TranslationMotor
btTriangleIndexVertexArray              .bullet.collision.shapes.infos.CompoundMesh
btTriangleRaycastCallback::Eflags       .bullet.RayTestFlag
btTypedConstraint                       .bullet.joints.Constraint
btVector3                               .math.Vector3f
btWheelInfo                             .bullet.objects.VehicleWheel
RotateOrder                             .bullet.RotationOrder
</pre>

<pre>
V-HACD C++ type:    corresponding Java class:
================    =========================
IVHACD              vhacd.VHACD
IVHACD::ConvexHull  vhacd.VHACDHull
IVHACD::Parameters  vhacd.VHACDParamters
</pre>

<a name="todo"/>

## What's missing

 + certain constraints:
   + `btFixedConstraint`
   + `btGearConstraint`
   + `btHinge2Constraint`
   + `btUniversalConstraint`
 + certain collision shapes:
   + `btMinkowskiSumShape`
   + `btMultimaterialTriangleMeshShape`
   + `btScaledBvhTriangleMeshShape`
 + inverse dynamics
 + serialization (file loader)
 + profiling
 + extras, examples, and tests
 + execution tracing
 + support for ARM architecture
 + `btAssert()` should perhaps throw a Java exception

<a name="links"/>

## External links

 + The [Bullet Physics SDK Manual](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf)
 + The [Bullet source-code repository](https://github.com/bulletphysics/bullet3) at [GitHub]
 + The [Minie project][minie] at [GitHub]
 + The [V-HACD Library][vhacd] at [GitHub]
 + The [physics section of the jMonkeyEngine Wiki](https://wiki.jmonkeyengine.org/jme3/advanced/physics.html)
 + The [Bullet Forum](https://pybullet.org/Bullet/phpBB3)
 + The [Bullet home page][bullet]
 + [JBullet], a known alternative to Libbulletjme

[Jump to table of contents](#toc)


[appveyor]: https://www.appveyor.com "AppVeyor Continuous Integration"
[bsd3]: https://opensource.org/licenses/BSD-3-Clause "3-Clause BSD License"
[bullet]: https://pybullet.org/wordpress "Bullet Real-Time Physics Simulation"
[chrome]: https://www.google.com/chrome "Chrome"
[findbugs]: http://findbugs.sourceforge.net "FindBugs Project"
[firefox]: https://www.mozilla.org/en-US/firefox "Firefox"
[gcc]: https://gcc.gnu.org "GNU Compiler Collection"
[gdb]: https://www.gnu.org/software/gdb/ "GNU Project Debugger"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gradle]: https://gradle.org "Gradle Project"
[heart]: https://github.com/stephengold/Heart "Heart Project"
[imgur]: https://imgur.com/ "Imgur"
[jbullet]: http://jbullet.advel.cz "JBullet"
[jfrog]: https://www.jfrog.com "JFrog"
[jme]: https://jmonkeyengine.org  "jMonkeyEngine Project"
[latest]: https://github.com/stephengold/Libbulletjme/releases/latest "latest release"
[libbulletjme]: https://github.com/stephengold/Libbulletjme "Libbulletjme Project"
[license]: https://github.com/stephengold/Libbulletjme/blob/master/LICENSE "Libbulletjme license"
[llvm]: https://www.llvm.org "LLVM Compiler"
[log]: https://github.com/stephengold/Libbulletjme/blob/master/release-notes.md "release log"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[meld]: https://meldmerge.org "Meld Project"
[minie]: https://github.com/stephengold/Minie "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[travis]: https://travis-ci.org "Travis CI"
[vhacd]: https://github.com/kmammou/v-hacd "V-HACD Library"
[v-hacd-java-bindings]: https://github.com/riccardobl/v-hacd-java-bindings "Riccardo's V-hacd-java-bindings Project"


<a name="history"/>

## History

The evolution of the project is chronicled in [its release log][log].

The C++ glue code for Bullet was originally copied from `jme3-bullet-native`,
a library of [jMonkeyEngine][jme].
The soft-body portion was added in 2018,
and is based on the work of Jules (aka "dokthar").

The Java code is based partly jMonkeyEngine,
partly on [Riccardo's V-hacd-java-bindings][v-hacd-java-bindings],
and partly on [Minie].
Minie is, in turn, based on `jme3-bullet`, another jMonkeyEngine library.

[Jump to table of contents](#toc)

<a name="acks"/>

## Acknowledgments

The Libbulletjme Project is based on open-source software:

  + the [Bullet] physics simulation kit
  + the [jMonkeyEngine][jme] game engine
  + [Dokthar's fork of jMonkeyEngine](https://github.com/dokthar/jmonkeyengine)
  + [Khaled Mamou's V-HACD Library][vhacd] for approximate convex decomposition
  + Riccardo's [V-hacd-java-bindings]

This project also made use of the following software tools:

  + the [FindBugs] source-code analyzer
  + the [GNU Compiler Collection][gcc] and [Project Debugger][gdb]
  + the [Git] revision-control system and GitK commit viewer
  + the [Firefox] and [Google Chrome][chrome] web browsers
  + the [Gradle] build tool
  + the Java compiler, standard doclet, and runtime environment
  + [jMonkeyEngine][jme] and the jME3 Software Development Kit
  + the [Linux Mint][mint] operating system
  + the [LLVM Compiler Infrastructure][llvm]
  + the [Markdown] document-conversion tool
  + the [Meld] visual merge tool
  + Microsoft Windows and Visual Studio
  + the [NetBeans] integrated development environment

I am grateful to Riccardo Balbo (aka "riccardo") for bringing
V-HACD to my attention.

I am grateful to [Github], [JFrog], [AppVeyor], [Travis], and [Imgur]
for providing free hosting for this project
and many other open-source projects.

I am grateful to Yanis Boudiaf for many helpful suggestions.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know so I can
correct the situation: sgold@sonic.net

[Jump to table of contents](#toc)
