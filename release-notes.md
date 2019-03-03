# release notes for the Libbulletjme Project

## Version 1.0.33 released on 3 March 2019

 + Try riccardobl's fix for JME issue #1029.

## Version 1.0.32 released on 2 March 2019

 + Use a copy of btSphereSphereCollisionAlgorithm.cpp from Bullet v2.82
   in an attempt to work around JME issue #1029.

## Version 1.0.31 released on 2 March 2019

 + Added a `getOrientation()` method to the `PhysicsCollisionObject` class.
 + In `jmePhysicsSpace::contactProcessedCallback()`, check both collision
   objects for NULL user pointers.
 + Upgraded the Bullet sources to match SHA1 id=1bac759a of the bullet3 project.

## Version 1.0.30 released on 30 January 2019

Added 21 methods to `PhysicsCollisionObject`:

 + `{get/has/set}AnisotropicFriction()`
 + `getBasis()`, `getLocation()`, and `setLocationAndBasis()`
 + `getContactDamping()`, `getContactStiffness()`,
    and `setContactStiffnessAndDamping()`
 + `{get/set}ContactProcessingThreshold()`
 + `{get/set}DeactivationTime`
 + `{get/set}Friction()`
 + `{get/set}Restitution()`
 + `{get/set}RollingFriction()`
 + `{get/set}SpinningFriction()`

## Version 1.0.29 released on 25 January 2019

 + Added `getFallSpeed()`, `getJumpSpeed()`, `getUpDirection()`, `reset()`,
   and `setUseGhostSweepTest()` to the `PhysicsCharacter` class.
 + Added `getCcdMotionThreshold()`, `getCcdSweptSphereRadius()`,
   `setCcdMotionThreshold()`, and `setCcdSweptSphereRadius()` to the
   `PhysicsCollisionObject` class.
 + Verify convex shape in `PhysicsCharacter.createCharacterObject()`.

## Version 1.0.28 released on 24 January 2019

Bugfix: `setStartupMessageEnabled()` should default to TRUE.

## Version 1.0.27 released on 23 January 2019

 + Renamed the project from Jme3-bullet-native to Libbulletjme.
 + Added `countSpheres()`, `getSpherePosition()`, and `getSphereRadius()`
   to the `MultiSphere` class.
 + Added `DebugTools.setStartupMessageEnabled()` in order to make the startup
   message optional.

## Version 1.0.26 released on 17 January 2019

Added `createShapeB(ByteBuffer, int)` methods to the `HullCollisionShape`
and `MultiSphere` classes.

## Version 1.0.24 released on 7 January 2019

Upgraded the Bullet sources to match release 2.88 of the bullet3 project.

## Version 1.0.22 released on 30 December 2018

Added diagnostic messages when a contact is ignored
by `jmePhysicsSpace::contactProcessedCallback`.

## Version 1.0.21 released on 8 December 2018

 + Print version number during initialization.
 + Added `EmptyShape` class based on `btEmptyShape`.
 + Added `isActive()` method to the `PhysicsCollisionObject` class.
 + Upgraded the Bullet sources to match SHA1 ID=4a66d6c of the bullet3 project.
 + Removed ancient threading support from `jmePhysicsSoftSpace`.

## Version 1.0.20 released on 4 December 2018

 + Added missing source file for `PhysicsSoftBody`.
 + Removed portions of the Bullet Physics library that are not needed here.

## Version 1.0.19 released on 3 December 2018

 + Added soft-body physics support, mostly copied straight from Dokthar's fork.
 + Added `getCollisionFlags()` and `setCollisionFlags()` to
   the `PhysicsCollisionObject` class.

## Version 1.0.18 released on 27 November 2018

 + Added a method to create a single-ended `SliderJoint`.
 + Added null checks and improved exception messages.

## Version 1.0.17 released on 25 November 2018

Added methods to create single-ended `ConeJoint`, `HingeJoint`,
`SixDofJoint`, and `SixDofSpringJoint`.

## Version 1.0.16 released on 24 November 2018

 + Added `getBreakingImpulseThreshold()`, `setBreakingImpulseThreshold()`,
   `isEnabled()`, and `setEnabled()` to the `PhysicsJoint` class.
 + Added a method to create a single-ended `Point2PointJoint`.

## Version 1.0.15 released on 19 November 2018

 + Added an option to generate debug meshes with 256 vertices for convex shapes,
   instead of the usual 42 vertices.
 + Avoid trashing the shape's user pointer in `DebugShapeFactory`.

## Version 1.0.14 released on 18 November 2018

 + Added a multi-sphere collision shape.
 + Added `isConcave()` to the `CollisionShape` class.
 + Changed the initialization message.

## Version 1.0.13 released on 15 November 2018

Upgraded the Bullet sources to match SHA1 ID=9ad77a2 of the bullet3 project.

## Version 1.0.12 released on 27 October 2018

 + Added `activate()` to the `PhysicsCollisionObject` class.
 + Added `getTargetVelocity()` to the `TranslationalLimitMotor` class.
 + Removed an accidentally committed DLL.
 + Improved argument validation for JNI methods.

## Version 1.0.11 released on 27 October 2018

Upgraded the Bullet sources to match release 2.87 of the bullet3 project.

## Version 1.0.10 released on 18 October 2018

Use the Force in `PhysicsRigidBody.activate()`.

## Version 1.0.9 released on 13 October 2018

 + Added `getAccumulatedImpulse()` and `getCurrentPosition()` to the
   `RotationalLimitMotor` class.
 + Added `getOffset()` and `setTargetVelocity()` to the
   `TranslationalLimitMotor` class.

## Version 1.0.8 released on 12 October 2018

 + Removed the unused `addConstraint()` and `initNativePhysics()` from
   the `PhysicsSpace` class.
 + Added 4 CFM accessors to the `RotationalLimitMotor` class.
 + Added 8 accessors to the `TranslationalLimitMotor` class for CFM, ERP,
   and max motor force.

## Version 1.0.7 released on 8 October 2018

 + Provided access to JME's `Transform.getScale()` method.
 + Added methods to convert a `btQuaternion` or `btTransform` to JME.
 + Added 3 frame accessors to the `SixDofJoint` class.

## Version 1.0.6 released on 8 October 2018

Removed all Bullet3 source files except `Bullet3Common`.

## Version 1.0.5 released on 3 October 2018

Disabled Quickprof to avoid its thread limit.

## Version 1.0.4 released on 2 October 2018

 + Fixed JME issues 918 and 919.
 + Added `getLocalScaling()` method for `CollisionShape`.
 + Removed 2 files not needed for Gradle build.

## Version 1.0.2 released on 30 September 2018

This was the initial baseline release, combining
source code from release 2.86.1 of the bullet3 project
with glue code from the `master` branch of the jmonkeyengine project.