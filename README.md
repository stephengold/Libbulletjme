The Libbulletjme Project adds "glue code" to a subset of the Bullet Physics
Library, providing Java projects with access to Bullet Physics functionality.

The project supports the 3 major desktop operating systems:
Windows, Linux, and OSX.  Both x86 and x64 architectures are supported for
each operating system.

The C++ source code is provided under
[a mixed license](https://github.com/stephengold/Libbulletjme/blob/master/LICENSE).

Libbulletjme is used in [Minie].

## Contents of this document

 + [History](#history)
 + [Lexicon of class/enum/struct names](#lexicon)
 + [External links](#links)
 + [Acknowledgments](#acks)

<a name="history"/>

## History

The Libbulletjme Project is hosted at
[GitHub](https://github.com/stephengold/Libbulletjme).

The evolution of the project is chronicled in
[its release notes](https://github.com/stephengold/Libbulletjme/blob/master/release-notes.md).

<a name="lexicon"/>

## Lexicon of class/enum/struct names

<pre>
Bullet v2 C++ type:               corresponding JME3 Java class com.jme3.bullet...
</pre>
<pre>
btBoxShape                        .collision.shapes.BoxCollisionShape
btBU_Simplex1to4                  .collision.shapes.SimplexCollisionShape
btBvhTriangleMeshShape            .collision.shapes.MeshCollisionShape
btCapsuleShape{X//Z}              .collision.shapes.CapsuleCollisionShape
btCollisionObject                 .collision.PhysicsCollisionObject
btCollisionShape                  .collision.shapes.CollisionShape
btCompoundShape                   .collision.shapes.CompoundCollisionShape
btCompoundShapeChild              .collision.shapes.info.ChildCollisionShape
btConeShape{X//Z}                 .collision.shapes.ConeCollisionShape
btConeTwistConstraint             .joints.ConeJoint
btConvexHullShape                 .collision.shapes.HullCollisionShape
btCylinderShape{X//Z}             .collision.shapes.CylinderCollisionShape
btDynamicsWorld                   .PhysicsSpace
btEmptyShape                      .collision.shapes.EmptyShape
btGeneric6DofConstraint           .joints.SixDofJoint
btGeneric6DofSpringConstraint     .joints.SixDofSpringJoint
btGImpactMeshShape                .collision.shapes.GImpactCollisionShape
btHeightfieldTerrainShape         .collision.shapes.HeightfieldCollisionShape
btHingeConstraint                 .joints.HingeJoint
btKinematicCharacterController    .objects.PhysicsCharacter
btManifoldPoint                   .collision.PhysicsCollisionEvent
btMultiSphereShape                .collision.shapes.MultiSphere
btPairCachingGhostObject          .objects.PhysicsGhostObject
btPoint2PointConstraint           .joints.Point2PointJoint
btRaycastVehicle                  .objects.PhysicsVehicle
btRigidBody                       .objects.PhysicsRigidBody
btRotationalLimitMotor            .joints.motors.RotationalLimitMotor
btSliderConstraint                .joints.SliderJoint
btSoftBody                        .objects.PhysicsSoftBody
btSoftBody::Config                .objects.infos.SoftBodyConfig
btSoftBodyWorldInfo               .SoftBodyWorldInfo
btSoftRigidDynamicsWorld          .PhysicsSoftSpace
btSphereShape                     .collision.shapes.SphereCollisionShape
btStaticPlaneShape                .collision.shapes.PlaneCollisionShape
btTriangleRaycastCallback::Eflags .RayTestFlag
btTypedConstraint                 .joints.PhysicsJoint
btTranslationalLimitMotor         .joints.motors.TranslationalLimitMotor
</pre>

<a name="links"/>

## External links

  + [The Bullet Physics SDK Manual](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf)
  + [The Physics section of the JME Wiki](https://wiki.jmonkeyengine.org/jme3/advanced/physics.html)

<a name="acks"/>

## Acknowledgments

The Libbulletjme Project is based on open-source software:

  + the [Bullet][] physics simulation kit
  + [jMonkeyEngine][jme]
  + [Dokthar's fork of jMonkeyEngine](https://github.com/dokthar/jmonkeyengine)

This project also made use of the following software:

  + the [Firefox web browser][firefox]
  + the [Git][] revision-control system and GitK commit viewer
  + the [Google Chrome web browser][chrome]
  + the [Gradle][] build tool
  + the Java compiler, standard doclet, and runtime environment
  + the jME3 Software Development Kit
  + [Linux Mint][mint]
  + the [Markdown][] document-conversion tool
  + Microsoft Windows
  + the [NetBeans][] integrated development environment

I am grateful to [Github][], [AppVeyor][], and [Travis][]
for providing free hosting for this project.

If I've misattributed anything or left anyone out, please let me know so I can
correct the situation: sgold@sonic.net

[appveyor]: https://www.appveyor.com "AppVeyor Continuous Integration"
[bullet]: https://pybullet.org/wordpress "Bullet Real-Time Physics Simulation"
[chrome]: https://www.google.com/chrome "Chrome"
[firefox]: https://www.mozilla.org/en-US/firefox/ "Firefox"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gradle]: https://gradle.org "Gradle Project"
[jme]: http://jmonkeyengine.org  "jMonkeyEngine Project"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[minie]: https://github.com/stephengold/Minie "Minie Project"
[mint]: https://linuxmint.com/ "Linux Mint"
[netbeans]: https://netbeans.org "NetBeans Project"
[travis]: https://travis-ci.org "Travis CI"
