The Libbulletjme Project adds "glue code" to a subset of the Bullet Physics
Library, enabling real-time simulation of 3-D physics in Java applications.

Complete source code (in C++) is provided under
[a mixed license](https://github.com/stephengold/Libbulletjme/blob/master/LICENSE).

The project supports the 3 major desktop operating systems:
Windows, Linux, and OSX.  Both the x86 and x86-64 architectures
are supported for each operating system.
For each (operating system, architecture) combination,
 up to 4 shared libraries are distributed:
 + a release build using single-precision arithmetic (the default library)
 + a release build using double-precision arithmetic
 + a debug build using single-precision arithmetic
 + a debug build using double-precision arithmetic

The project is hosted at
[GitHub](https://github.com/stephengold/Libbulletjme).

Libbulletjme is used in [Minie].

## Contents of this document

 + [History](#history)
 + [How to build Libbulletjme](#build)
 + [Lexicon of class/enum/struct names](#lexicon)
 + [External links](#links)
 + [Acknowledgments](#acks)

<a name="history"/>

## History

The glue code of Libbulletjme was originally copied from `jme3-bullet-native`,
a library of the [jMonkeyEngine][jme] game engine.

The evolution of the project is chronicled in
[its release notes](https://github.com/stephengold/Libbulletjme/blob/master/release-notes.md).

<a name="build"/>

## How to build Libbulletjme

 1. Install the right tools:
   + [Gradle]
   + [Gcc] (for Linux)
   + Microsoft Visual Studio (for Windows)
   + XCode (for MacOSX)
 2. Download and extract the source code from GitHub:
   + using Git:
     + `git clone https://github.com/stephengold/Libbulletjme.git`
     + `cd Libbulletjme`
   + using a web browser:
     + browse to [https://github.com/stephengold/Libbulletjme/releases/latest](https://github.com/stephengold/Libbulletjme/releases/latest)
     + follow the "Source code (zip)" link
     + save the file
     + unzip the saved file
     + `cd` to the extracted directory/folder
 3. Run the Gradle wrapper: `./gradlew build`

After a successful build, dynamic libraries will be found
in the `dist` directory/folder.

<a name="lexicon"/>

## Lexicon of class/enum/struct names

<pre>
Bullet v2 C++ type:               corresponding Java class: com.jme3...
</pre>
<pre>
btBoxShape                        .bullet.collision.shapes.BoxCollisionShape
btBU_Simplex1to4                  .bullet.collision.shapes.SimplexCollisionShape
btBvhTriangleMeshShape            .bullet.collision.shapes.MeshCollisionShape
btCapsuleShape{X//Z}              .bullet.collision.shapes.CapsuleCollisionShape
btCollisionObject                 .bullet.collision.PhysicsCollisionObject
btCollisionShape                  .bullet.collision.shapes.CollisionShape
btCompoundShape                   .bullet.collision.shapes.CompoundCollisionShape
btCompoundShapeChild              .bullet.collision.shapes.info.ChildCollisionShape
btConeShape{X//Z}                 .bullet.collision.shapes.ConeCollisionShape
btConeTwistConstraint             .bullet.joints.ConeJoint
btConvexHullShape                 .bullet.collision.shapes.HullCollisionShape
btCylinderShape{X//Z}             .bullet.collision.shapes.CylinderCollisionShape
btDynamicsWorld                   .bullet.PhysicsSpace
btEmptyShape                      .bullet.collision.shapes.EmptyShape
btGeneric6DofConstraint           .bullet.joints.SixDofJoint
btGeneric6DofSpringConstraint     .bullet.joints.SixDofSpringJoint
btGImpactMeshShape                .bullet.collision.shapes.GImpactCollisionShape
btHeightfieldTerrainShape         .bullet.collision.shapes.HeightfieldCollisionShape
btHingeConstraint                 .bullet.joints.HingeJoint
btIndexedMesh                     .bullet.collision.shapes.infos.IndexedMesh
btKinematicCharacterController    .bullet.objects.PhysicsCharacter
btManifoldPoint                   .bullet.collision.PhysicsCollisionEvent
btMatrix3x3                       .math.Matrix3f
btMultiSphereShape                .bullet.collision.shapes.MultiSphere
btPairCachingGhostObject          .bullet.objects.PhysicsGhostObject
btPoint2PointConstraint           .bullet.joints.Point2PointJoint
btQuaternion                      .math.Quaternion
btRaycastVehicle                  .bullet.objects.PhysicsVehicle
btRigidBody                       .bullet.objects.PhysicsRigidBody
btRotationalLimitMotor            .bullet.joints.motors.RotationalLimitMotor
btSliderConstraint                .bullet.joints.SliderJoint
btSoftBody                        .bullet.objects.PhysicsSoftBody
btSoftBody::Config                .bullet.objects.infos.SoftBodyConfig
btSoftBodyWorldInfo               .bullet.SoftBodyWorldInfo
btSoftRigidDynamicsWorld          .bullet.PhysicsSoftSpace
btSphereShape                     .bullet.collision.shapes.SphereCollisionShape
btStaticPlaneShape                .bullet.collision.shapes.PlaneCollisionShape
btTransform                       .math.Transform
btTranslationalLimitMotor         .bullet.joints.motors.TranslationalLimitMotor
btTriangleIndexVertexArray        .bullet.collision.shapes.infos.CompoundMesh
btTriangleRaycastCallback::Eflags .bullet.RayTestFlag
btTypedConstraint                 .bullet.joints.PhysicsJoint
btVector3                         .math.Vector3f
btWheelInfo                       .bullet.objects.VehicleWheel
</pre>

<a name="links"/>

## External links

  + [The Bullet Physics SDK Manual](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf)
  + [The Physics section of the JME Wiki](https://wiki.jmonkeyengine.org/jme3/advanced/physics.html)

<a name="acks"/>

## Acknowledgments

The Libbulletjme Project is based on open-source software:

  + the [Bullet] physics simulation kit
  + the [jMonkeyEngine][jme] game engine
  + [Dokthar's fork of jMonkeyEngine](https://github.com/dokthar/jmonkeyengine)

This project also made use of the following software:

  + the [Firefox web browser][firefox]
  + the [Gcc] compiler
  + the [Git] revision-control system and GitK commit viewer
  + the [Google Chrome web browser][chrome]
  + the [Gradle] build tool
  + the Java compiler, standard doclet, and runtime environment
  + the jME3 Software Development Kit
  + [Linux Mint][mint]
  + the [Markdown] document-conversion tool
  + Microsoft Windows and Visual Studio
  + the [NetBeans] integrated development environment

I am grateful to [Github], [AppVeyor], and [Travis]
for providing free hosting for this project.

If I've misattributed anything or left anyone out, please let me know so I can
correct the situation: sgold@sonic.net

[appveyor]: https://www.appveyor.com "AppVeyor Continuous Integration"
[bullet]: https://pybullet.org/wordpress "Bullet Real-Time Physics Simulation"
[chrome]: https://www.google.com/chrome "Chrome"
[firefox]: https://www.mozilla.org/en-US/firefox "Firefox"
[gcc]: https://gcc.gnu.org "Gcc Compiler"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gradle]: https://gradle.org "Gradle Project"
[jme]: http://jmonkeyengine.org  "jMonkeyEngine Project"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[minie]: https://github.com/stephengold/Minie "Minie Project"
[mint]: https://linuxmint.com/ "Linux Mint"
[netbeans]: https://netbeans.org "NetBeans Project"
[travis]: https://travis-ci.org "Travis CI"
