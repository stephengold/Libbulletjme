# release notes for the Libbulletjme Project

## Version 2.0.1 released on 25 July 2019

 + Deleted `NativeMeshUtil` and `PhysicsJoint`.
 + Deleted 51 JNI methods that Minie no longer uses.

## Version 1.0.90 released on 4 July 2019

Fixed 2 bugs:
 + `btAssert()` while creating a rigid body with a `MeshCollisionShape`
 + `ConeJoint` causes rigid bodies to disappear (JME issue 1135)

## Version 1.0.89 released on 27 June 2019

Added accessors for 6 cluster parameters.

## Version 1.0.88 released on 25 June 2019

Re-try after a partial deployment.

## Version 1.0.87 released on 24 June 2019

Solved some compile-time errors reported by GCC.

## Version 1.0.86 released on 24 June 2019

 + Added a `getShapeType()` method to `CollisionShape`.
 + Added `getConstraintType()`, `getOverrideIterations()`, and
   `overrideIterations()` methods to the `Constraint` class.
 + Added assertions to verify the internal type in `PhysicsGhostObject`.

## Version 1.0.85 released on 23 June 2019

 + Added new class `Anchor`.
 + Upgraded the Bullet sources to match SHA1 ID=681a730e of the bullet3 project.

## Version 1.0.84 released on 22 June 2019

Added 2 new classes: `Constraint` and `NativeLibrary`.

## Version 1.0.83 released on 21 June 2019

 + Redesigned the create methods for `SoftAngularJoint` and `SoftLinearJoint`.
 + Removed the `updateBound()` call from `GImpactCollisionShape.createShape()`.
 + Added `enableFeedback()` and `needsFeedback()` methods to the
   `PhysicsJoint` class.
 + Added a `getInternalType()` method to `PhysicsCollisionObject`.
 + Added assertions to validate indices in `PhysicsSoftBody`.

## Version 1.0.82 released on 21 June 2019

Added `appendCluster()` and `finishClusters()` methods to the
`PhysicsSoftBody` class.

## Version 1.0.81 released on 18 June 2019

 + Added `recalcAabb()` methods to 6 shape classes.
 + Deployed PDB files from AppVeyor.

## Version 1.0.80 released on 17 June 2019

 + Bugfix: crash with `HeightfieldCollisionShape` in DP build
   (accessing freed array).
 + Added `createShape2()` and `finalizeNative()` methods to the
   `HeightfieldCollisionShape` class.
 + Deployed debug libraries from AppVeyor.
 + Stopped #undef-ing `_FORTIFY_SOURCE`.

## Version 1.0.79 released on 15 June 2019

 + Added a new mechanism for creating native meshes: uses `CompoundMesh`
   and `IndexedMesh` instead of `NativeMeshUtil`.
 + Bugfix: `btAssert` while creating a rigid body with `EmptyShape`.
 + Implemented `Debug` builds for the `VisualCpp` toolchain.
 + Added a `finalizeBVH()` method to `MeshCollisionShape`.

## Version 1.0.78 released on 12 June 2019

Resolved yet another compile-time error.

## Version 1.0.77 released on 12 June 2019

Resolved more compile-time diagnostic messages.

## Version 1.0.76 released on 12 June 2019

Resolved some compile-time diagnostic messages.

## Version 1.0.75 released on 12 June 2019

 + Renamed the distribution files again.
 + Added double-precision (DP) builds.

## Version 1.0.74 released on 12 June 2019

 + Renamed the distribution files.
 + Added debug builds for the gcc toolchain.
 + Added null checks for buffers passed via JNI.

## Version 1.0.73 released on 6 June 2019

 + Added count arguments to `appendFaces()`, `appendLinks()`, `appendNodes()`,
   and `appendTetras()` methods in the `PhysicsSoftBody` class.
 + Added limit checks for the number of nodes when masses, normals,
   and velocities in the `PhysicsSoftBody` class.

## Version 1.0.72 released on 6 June 2019

Bugfix: scaling ignored in PhysicsSoftBody.applyPhysicsTransform().

## Version 1.0.71 released on 31 May 2019

 + Added an `updateAnchorMesh()` method to the `NativeSoftBodyUtils` class.
 + Changed the default collision margin for soft bodies from 0 to
   `CONVEX_DISTANCE_MARGIN` (which is currently 0.04).

## Version 1.0.70 released on 27 May 2019

Added an explicit cast to enum type.

## Version 1.0.69 released on 27 May 2019

 + Removed `applyAeroToMode()` method from the `PhysicsSoftBody` class.
 + Added `getWindVelocity()` and `setWindVelocity()` methods to the
   `PhysicsSoftBody` class.
 + Added `getAeroModel()` and `setAeroModel()` methods to the
   `SoftBodyConfig` class.

## Version 1.0.68 released on 26 May 2019

Renamed the `PhysicsSoftBody.Config` class to `SoftBodyConfig`.

## Version 1.0.67 released on 26 May 2019

Corrected the names of 2 methods in `SoftBodyWorldInfo`.

## Version 1.0.66 released on 26 May 2019

Corrected the names of 2 methods in `SoftBodyWorldInfo`.

## Version 1.0.65 released on 26 May 2019

 + Moved the `SoftBodyWorldInfo` class from com_jme3_bullet_objects_info
   to com_jme3_bullet.
 + Removed 4 methods from `PhysicsSoftBody`:
   + `getPhysicsRotation`
   + `getPhysicsTransform`
   + `setPhysicsRotation`
   + `setPhysicsTransform`

## Version 1.0.64 released on 25 May 2019

Added an `updateClusterMesh()` method to the `NativeSoftBodyUtil` class.

## Version 1.0.63 released on 25 May 2019

 + Added `countNodesInCluster()` and `listNodesInCluster()`
   to the `PhysicsSoftBody` class.
 + Upgraded the Bullet sources to match SHA1 ID=26486d56 of the bullet3 project.

## Version 1.0.62 released on 21 May 2019

 + Changed "and" to "or" in the `needBroadphaseCollision()` methods.
 + Added a `getClustersMasses()` method to the `PhysicsSoftBody` class.

## Version 1.0.61 released on 13 May 2019

Added `getMargin()` and `setMargin()` methods to the `PhysicsSoftBody` class.

## Version 1.0.60 released on 12 May 2019

 + Initialized the inertia tensor of a `PhysicsRigidBody` even if it's static,
   since the tensor is used in `btSoftBody::PSolve_RContacts()`.
 + Registered some soft-body collision algorithms in `jmePhysicsSoftSpace`.

## Version 1.0.59 released on 9 May 2019

Added 8 methods to the `PhysicsSoftBody` class:
 + `cutLink()`
 + `getAnchorInfluence()`
 + `getAnchorNodeIndex()`
 + `getAnchorPivot()`
 + `getAnchorRigidId()`
 + `isCollisionAllowed()`
 + `setNormals()`
 + `setVelocities()`

## Version 1.0.57 released on 7 May 2019

Added a `getBounds()` method to the `PhysicsSoftBody` class.

## Version 1.0.56 released on 6 May 2019

 + Copied the `needBroadphaseCollision` code from `jmePhysicsSpace.cpp`
   to `jme3PhysicsSoftSpace.cpp`.
 + Added `getClusterCenter()` and `setNodeVelocity()` methods to
   the `PhysicsSoftBody` class.
 + Removed all null checks for `new` operations:  not necessary.
 + Implemented the `bt32BitAxisSweep3` option in `jme3PhysicsSoftSpace`.
 + Registered the `btGImpactCollisionAlgorithm` in `jme3PhysicsSoftSpace`.

## Version 1.0.55 released on 5 May 2019

 + Removed the redundant `getBoundingCenter()` methods from the
   `PhysicsSoftBody` class.
 + Renamed the `setMacDisplacement()` method in the `SoftBodyWorldInfo` class.
 + Upgraded the Bullet sources to match SHA1 ID=f4f5f708 of the bullet3 project.
 + Added 9 methods to the `PhysicsSoftBody` class:
   + `addVelocity()`
   + `getAnchorCount()`
   + `getClustersPositions()`
   + `getNodeLocation()`
   + `getNodeNormal()`
   + `getNodesNormals()`
   + `getNodesVelocities()`
   + `getNodeVelocity()`
   + `setVelocity()`

## Version 1.0.54 released on 30 April 2019

 + Renamed the `getRestLenghtScale()` and `setRestLenghtScale()` methods
   in the `PhysicsSoftBody` class.
 + Added an `isInWorld()` method to the `PhysicsCollisionObject` class.
 + Changed `free()` to `delete` in the `jmeMotionState` destructor.

## Version 1.0.53 released on 29 April 2019

Changed `free()` to `delete` in `MultiSphere`.

## Version 1.0.52 released on 29 April 2019

Rolled back half of the recent change to `MultiSphere`.

## Version 1.0.51 released on 29 April 2019

 + Clarified the buffer-reading loops in `PhysicsSoftBody`.
 + Plugged a memory leak in `MultiSphere`.

## Version 1.0.50 released on 25 April 2019

 + Added getters for frame transforms of cone, hinge, and slider joints.
 + Built using Gradle v5.3.1

## Version 1.0.49 released on 14 April 2019

Bugfix: `NullPointerException` thrown by `NULL_CHECK` in
`PhysicsRigidBody.setAngularDamping()`.

## Version 1.0.48 released on 14 April 2019

Forced recalculation of pivot offset in `SixDofJoint.getPivotOffset()`.

## Version 1.0.47 released on 14 April 2019

 + Forced recalculation of joint angles in `SixDofJoint.getAngles()`.
 + Deployed a `NULL_CHECK` macro.

## Version 1.0.46 released on 10 April 2019

Bugfix: compile-time errors in `SixDofJoint`.

## Version 1.0.45 released on 10 April 2019

 + Added `getAngles()` and `getPivotOffset()` methods
   to the `SixDofJoint` class.
 + Updated the Bullet sources to match SHA1 ID=c6a43e0a5 of the bullet3 project.

## Version 1.0.44 released on 2 April 2019

 + Reverted GCC options for debugging.
 + Final fix for JME issue #1058.

## Version 1.0.43 released on 2 April 2019

Reordered 2 statements in `btSimpleBroadphase::destroyProxy()` while
trying to fix JME issue #1058.

## Version 1.0.42 released on 2 April 2019

GCC options for debugging.

## Version 1.0.41 released on 1 April 2019

 + Added a `getNumConstaints()` method to the `PhysicsSpace` class.
 + Implement `AXIS_SWEEP_3_32` broadphase algorithm; was aliased
   to `btAxisSweep3`.

## Version 1.0.40 released on 23 March 2019

Added `getAabb()` and `getBoundingSphere()` methods to
the `CollisionShape` class.

## Version 1.0.39 released on 11 March 2019

Try a different method of constructing `btVector3`.

## Version 1.0.38 released on 11 March 2019

 + Added a `CollisionShape.setLocalScaling()` using scalars.
 + Added a `PhysicsRigidBody.setGravity()` using scalars.
 + Updated the Bullet sources to match SHA1 id=d56b1361 of the bullet3 project.

## Version 1.0.37 released on 9 March 2019

Optimize the convex hull when creating a `HullCollisionShape`.

## Version 1.0.36 released on 9 March 2019

 + Added `countHullVertices()` and `getHullVertices()` methods to
   the `HullCollisionShape` class.
 + Added a `getInverseInertiaWorld()` method to the `PhysicsRigidBody` class.

## Version 1.0.35 released on 5 March 2019

Added an `applyCentralImpulse()` method to the `PhysicsRigidBody` class.

## Version 1.0.34 released on 3 March 2019

Remove extra qualification from `jmePhysicsSpace.h`.

## Version 1.0.33 released on 3 March 2019

Try riccardobl's fix for JME issue #1029.

## Version 1.0.32 released on 2 March 2019

Use a copy of `btSphereSphereCollisionAlgorithm.cpp` from Bullet v2.82
in an attempt to work around JME issue #1029.

## Version 1.0.31 released on 2 March 2019

 + Added a `getOrientation()` method to the `PhysicsCollisionObject` class.
 + In `jmePhysicsSpace::contactProcessedCallback()`, check both collision
   objects for NULL user pointers.
 + Updated the Bullet sources to match SHA1 id=1bac759a of the bullet3 project.

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