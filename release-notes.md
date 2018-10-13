# release notes for the jme3-bullet-native project

## Version 1.0.8 released on TBD

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

 + Removed all Bullet3 source files except Bullet3Common.

## Version 1.0.5 released on 3 October 2018

 + Disabled Quickprof to avoid its thread limit.

## Version 1.0.4 released on 2 October 2018

 + Fixed JME issues 918 and 919.
 + Added `getLocalScaling()` method for `CollisionShape`.
 + Removed 2 files not needed for Gradle build.

## Version 1.0.2 released on 30 September 2018

This was the initial baseline release, combining
source-code from release 2.86.1 of the bullet3 project
with glue code from the `master` branch of the jmonkeyengine project.