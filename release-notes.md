# Release log for the Libbulletjme project

## Version 16.3.0 released on 25 September 2022

+ Bugfix:  shape's scale isn't copied in `HullCollisionShape.split()`
+ Bugfix:  shape's scale is ignored by `ChildCollisionShape.split()`
+ Bugfix:  `MeshCollisionShape.split()` doesn't preserve scaling
+ Bugfix:  `GImpactCollisionShape.split()` doesn't preserve scaling
+ Added the `Generator` class for randomization.
+ Added a 4-sphere constructor to the `MultiSphere` class.
+ Added 4 more math methods:
  + `MyMath.fromAngles()`
  + `MyMath.isBetween()`
  + `MyMath.max()`
  + `RectangularSolid.halfExtents()`
+ Added the `root2` constant to the `MyMath` class.

## Version 16.2.1 released on 22 September 2022

+ Bugfix:  assertion failure in `VHACDHull` after a failed decomposition
+ Added the `pairTest()` method to the `CollisionSpace` class.
+ Implemented collision-shape splitting:
  + Added `split()` methods to 7 classes:
    + `ChildCollisionShape`
    + `CompoundCollisionShape`
    + `CompoundMesh`
    + `GImpactCollisionShape`
    + `HullCollisionShape`
    + `IndexedMesh`
    + `MeshCollisionShape`
  + Added the `canSplit()` and `toSplittableShape()` methods
    to the `CollisionShape` class.
  + Added some constructors to 5 classes:
    + `CompoundMesh`
    + `GImpactCollisionShape`
    + `HullCollisionShape`
    + `IndexedMesh`
    + `MeshCollisionShape`
  + Added the `toHullShape()` method to the `ConvexShape` class.

+ Implemented center/volume estimation for collision shapes:
  + Added the `aabbCenter()` and `scaledVolume()`
    methods to the `CollisionShape` class.
  + Added the `maxMin()` method to the `CompoundMesh` class.
  + Added `volumeConvex()` methods to the `DebugMeshCallback`
    and `DebugShapeFactory` classes.
  + Added the `copyTriangle()` and `maxMin()` methods
    to the `IndexedMesh` class.

+ Added math classes:
  + `DistinctVectorValues`
  + `MyVolume`
  + `RectangularSolid`
  + `Triangle`
  + `VectorSetUsingBuffer`
+ Added math methods:
  + `MyBuffer.covariance()`
  + `MyBuffer.maxMin()`
  + `MyBuffer.mean()`
  + `MyMath.cube()`
  + `MyMath.mid(float, float, float)`
  + `MyMath.min(float...)`
  + `MyMath.modulo(int, int)`
  + `MyMath.transformInverse()` for a `Triangle`
  + the normal-and-displacement constructor for a `Plane`
  + `Plane.pseudoDistance()`
  + `Vector3f.crossLocal()`
  + `Vector3f.distance()`
  + `Vector3f.mult(float, Vector3f)`
  + `Vector3f.mult(Vector3f)`
+ Added the `FastMath.QUARTER_PI` constant.
+ Added the `VectorSet` interface.

+ Added supplemental V-HACD methods:
  + `VHACDParameters.toMap()`
  + `Vhacd4Parameters.nextFillMode()`
  + `Vhacd4Parameters.toMap()`
+ Updated the Bullet sourcecode to match SHA1 ID=dad061fc1
  of the bullet3 project.
+ Upgraded Node.js to v16.17.0

## Version 16.1.0 released on 12 August 2022

+ Bugfix:  I/O resources not safely closed in `VHACDParameters`.
+ Added V-HACD v4 without removing the classic version:
  + Added the `Vhacd4`, `Vhacd4Hull`, and `Vhacd4Parameters` classes.
  + Added the `FillMode` enum.
  + Added a new constructor for `HullCollisionShape`.
+ Added the `setIgnoreList()` method to the `PhysicsCollisionObject` class.
+ Added a simpler constructor to `PhysicsSoftSpace`.
+ Overrode the `toString()` method of `VHACDParameters`.
+ Publicized the `rebuildRigidBody()` method of the `PhysicsRigidBody` class.
+ Created per-class loggers for `SoftBodyMaterial` and `SoftBodyWorldInfo`.
+ Upgraded the GCC compilers used to build Linux natives:
  + Linux32 and Linux64 non-multithreaded, from v4.7.3 to v7.5.0
  + Linux_ARM32hf and Linux_ARM64, from v5.4.0 to v6.5.0
  + Linux64 multithreaded and Linux_ARM32 with software floating-point,
    from v5.4.0 to v7.5.0

## Version 16.0.0 released on 4 August 2022

+ Qualified 13 utility classes as `final`. (API changes)
+ Protected 2 constructors of `SoftPhysicsJoint`. (API changes)
+ Bugfix:  `PhysicsRigidBody.setInverseInertiaLocal()` and
  `PhysicsRigidBody.updateMassProps()` don't update the world inertia tensor
+ Bugfix:  static rigid body isn't rebuilt when assigned a positive mass
+ Bugfix:  when rebuilding a rigid body, its ignore list is lost
+ De-privatized the `setIgnoreCollisionCheck()` method
  in `PhysicsCollisionObject`.

## Version 15.2.1 released on 6 July 2022

+ Bugfix:  values of 3 `ConfigFlag` constants are out-of-date
+ Bugfix:  Android native libraries are missing an object file
+ Bugfix:  `ConfigFlag.describe()` ignores 3 flags
+ Added the `DeformableSpace`, `MyShape`, and `ReducedDeformableBody` classes.
+ Added constants `SDF_MDF` and `SDF_RDN` to the `ConfigFlag` class.
+ Added the `tan()` method to the `FastMath` class.
+ Added the `isOdd()` method to the `MyMath` class.
+ Added the `removeSuffix()` method to the `MyString` class.
+ Added a warning when the native library version is unexpected.
+ Updated the Bullet sourcecode to match SHA1 ID=a1d96646
  of the bullet3 project.
+ Added the "checkstyle" plugin to the build.

## Version 15.1.0 released on 6 June 2022

+ Added accessors for global deactivation settings:
  + `PhysicsBody.getDeactivationDeadline()`
  + `PhysicsBody.isDeactivationEnabled()`
  + `PhysicsBody.setDeactivationDeadline()`
  + `PhysicsBody.setDeactivationEnabled()`
+ Added public setters for constraint pivots:
  + `New6Dof.setPivotInA()`
  + `New6Dof.setPivotInB()`

## Version 15.0.0 released on 3 June 2022

+ Changed the arguments of `PhysicsSoftBody.appendFaces()`
  and `PhysicsSoftBody.appendLinks()`. (API change)
+ Added the `Mesh` interface and `IndexBuffer` class to simplify the interface
  between soft bodies and the SPORT graphics engine.
+ Added soft-body methods from Minie:
  + `PhysicsSoftBody.appendTetras()`
  + `PhysicsSoftBody.boundingBox()`
  + `NativeSoftBodyUtils.appendFromLineMesh()`
  + `NativeSoftBodyUtils.appendFromTriMesh()`
  + `NativeSoftBodyUtils.appendTetras()`
  + `NativeSoftBodyUtils.mapIndices()`
  + `NativeSoftBodyUtils.updateClusterMesh()`
  + `NativeSoftBodyUtils.updateMesh()`
  + `NativeSoftBodyUtils.updatePinMesh()`
+ Added math methods from Heart:
  + `MyMath.isIdentity(Transform)`
  + `MyBuffer.rotate()`
  + `MyBuffer.translate()`
+ Added the `phi` constant to the `MyMath` class

## Version 14.5.0 released on 29 May 2022

+ Bugfix: `PhysicsCharacter.onGround()` is unreliable (issue #18)
+ Added math methods:
  + `BoundingBox.getCenter()`
  + `MyVector3f.midpoint()`

## Version 14.4.0 released on 20 May 2022

+ When allocating direct buffers, specify *native* byte order.
+ Added math methods:
  + `FastMath.pow(float, float)`
  + `MyMath.modulo(float, float)`
  + `MyMath.standardizeAngle(float)`
  + `MyMath.toDegrees(float)`
  + `MyMath.toRadians(float)`
  + `MyVector3f.accumulateScaled(Vector3f, Vector3f, float)`
  + `Transform.loadIdentity()`
  + `Vector3f.divide(Vector3f)`
  + `Vector3f.mult(float)`
+ Added math constants:
  + `FastMath.TWO_PI`
  + `MyMath.DEG_TO_RAD`
  + `MyMath.RAD_TO_DEG`
  + `MyMath.rootHalf`
  + `MyVector3f.firstAxis`
  + `MyVector3f.lastAxis`
+ Added string methods:
  + `MyString.escape(CharSequence)`
  + `MyString.quote(CharSequence)`
+ Added validation methods:
  + `Validate.axisIndex(int, String)`
  + `Validate.finite(float, String)`
  + `Validate.nonEmpty(String, String)`
+ Log the filename passed to `System.load()` in order to simplify debugging.

## Version 14.3.0 released on 11 April 2022

Added the `GearJoint` class.

## Version 14.2.0 released on 28 March 2022

+ Bugfix:  `EXCEPTION_ACCESS_VIOLATION` on Windows (Minie issue #23)
+ Added `add()`, `negate()`, `negateLocal()`, and `subtractLocal()` methods to
  the `Vector3f` class.

## Version 14.1.0 released on 13 March 2022

+ Added accessors for `m_erp` and `m_erp2` to the `SolverInfo` class.
+ Explicitly required Java v8 or higher.

## Version 14.0.0 released on 2 March 2022

+ Redesigned the `ContactListener` interface for utility and efficiency.
  (API changes)
+ Added the `PersistentManifolds` utility class.
+ Added the `countManifolds()` and `listManifolds()` methods
  to the `PhysicsSpace` class.
+ Added a script to build native libraries for/on Apple Silicon.
+ Handle missing platform subdirectories in `NativeLibraryLoader`.

## Version 13.0.0 released on 26 February 2022

+ Added the `ManifoldPoints` utility class, which provides getters and setters
  for `btManifoldPoint` without the need to instantiate
  a `PhysicsCollisionEvent`.
+ The `PhysicsCollisionEvent.setContactCalcArea3Points()` method was moved
  to the `ManifoldPoints` class. (API change)
+ The 4 setters added to `PhysicsCollisionEvent` in v12.8.0
  were deleted. (API change)
+ The `PhysicsSpace.onContactProcessed()` method (added in v12.8.0)
  was deleted. (API change)
+ Defined the `ContactListener` interface for immediate processing
  of rigid-body contacts.
  Overriding this interface of `PhysicsSpace` is now the recommended way
  to process rigid-body contacts.
+ Added a new `update()` method to `PhysicsSpace` to enable callbacks
  to specific `ContactListener` methods.
+ Deprecated 5 `PhysicsSpace` methods associated with event queueing:
  + `addCollisionListener()`
  + `addOngoingCollisionListener()`
  + `distributeEvents()`
  + `removeCollisionListener()`
  + `removeOngoingCollisionListener()`

## Version 12.8.0 released on 25 February 2022

+ Bugfix:  btAssert from `HingeJoint.setAngularOnly(true)` (Minie issue 20)
+ Added a dynamic collision-filtering hook to the `CollisionSpace` class.
+ Added an immediate ongoing-contact handler to the `PhysicsSpace` class.
+ Added 4 native setters to the `PhysicsCollisionEvent` class.
+ Changed `PrimitiveAllocator` to throw an exception
  on any destruction attempt.
+ Added a `Platform` mechanism to determine the generic name
  of the operating system.
+ Built using Gradle v7.4 .

## Version 12.7.1 released on 24 January 2022

+ Bugfix: continuous collision detection causes memory corruption in a
  multithreaded world (bullet3 issue 4117)
+ Restored support for the MacOSX32 platform.
+ Added `divide(float, float, float)`, `divideLocal(float, float, float)`, and
  `mult(float, float, float)` methods to the `Vector3f` class.
+ Updated the Bullet sourcecode.

## Version 12.6.0 released on 4 December 2021

+ Bugfix: `btTriangleShape::isInside()` relies on the plane normal, which is
  invalid for a degenerate triangle
+ Bugfix: unsafe `normalize()` is used in `btTriangleShape`
+ Bugfix: typo in `btTriangleShape::isInside()` ("distance" for "distance2")
+ Pending resolution of Travis CI ticket #34567:
  + Upgraded MacOS64 build tools from Xcode 9.3 to Xcode 12.4 .
  + Dropped support for the MacOSX32 platform.
+ Added the `isInsideTriangle()` method to the `NativeLibrary` class.
+ Updated the Bullet sourcecode to match SHA1 ID=10f72b9b5
  of the bullet3 project.

## Version 12.5.0 released on 8 November 2021

+ Cached the methods that free native objects, to improve performance.
+ Implemented contact filtering for GImpact collision shapes.
+ Added the `setPivotInB()` method to the `Anchor` class.
+ Overrode the `toString()` method in the `IntPair` class.

## Version 12.4.1 released on 25 October 2021

+ Bugfix: `SphereTriangleDetector` doesn't account for the triangle's margin
+ NativePhysicsObject:  make all NPOs comparable (for use in collections)

## Version 12.3.1 released on 23 October 2021

Bugfix: contact filtering is too aggressive

## Version 12.3.0 released on 22 October 2021

+ Bugfix: invalid contact points for heightfield/mesh shapes (Minie issue #18)
+ Bugfix: unsafe `normalize()` is used in `btRaycastVehicle`
+ Bugfix: logic errors in `btTriangleShape::isInside()`
+ Added a flag to the `CollisionShape` class to disable contact filtering.
+ Identified native libraries built with `DEBUG_PERSISTENCY` defined.
+ Added accessors to the `CollisionSpace` class
  for the "deterministic overlapping pairs" mode bit.

## Version 12.2.2 released on 30 September 2021

Bugfix: vehicle wheels don't rotate in v12.2.1

## Version 12.2.1 released on 30 September 2021

+ Bugfix: the velocities used to calculate `deltaRotation` are inaccurate
+ Bugfix: wheel rotation grows without bound, leading to roundoff errors
+ Bugfix: damping is applied incorrectly to `deltaRotation`
+ Increased customization of the right/forward/up axes of a vehicle chassis.
+ Added double-precision accessors for the gravity vectors of rigid bodies.
+ Added the `Comparable` interface to the `IntPair` class.
+ Upgraded the Bullet sources to match SHA1 ID=ce2627192
  of the bullet3 project.

## Version 12.1.1 released on 25 September 2021

+ Added double-precision getters for the locations and orientations
  of collision objects.
+ Added double-precision accessors for the locations, orientations,
  and velocities of rigid bodies.
+ Added the `Quatd` and `Vec3d` classes from SimMath.
+ Added the `isFinite()` method to the `MyMath` class.
+ Built using Gradle v6.9.1 .

## Version 12.0.0 released on 20 August 2021

+ Removed the `getX()` method from the `Vector3f` class. (API change)
+ Removed the deprecated `getAngularFactor()` method
  from the `PhysicsRigidBody` class. (API change)
+ Renamed the `addContraintTorque()` native method
  in the `MultiBodyLink` class. (API change)
+ Bugfix: "SpQuickprof" not recognized as a valid build flavor
+ Disabled contact callbacks when the space has no listeners. (API change)
+ In Mt builds, allocated 2 worker threads instead of the maximum number.
+ Added profiling points to the `jmePhysicsSpace` class.

## Version 11.2.1 released on 14 August 2021

Optimized Release-type builds that use Microsoft's Visual C++ compiler.

## Version 11.2.0 released on 13 August 2021

+ Bugfix: pure virtual call by btGImpactMeshShape destructor (Minie issue #17)
+ Bugfix: Quickprof reset at the start of every timestep
+ Bugfix: BT_PROFILE() macro never invokes CProfileManager
+ Added access to Quickprof profiling.

## Version 11.1.0 released on 9 August 2021

+ Bugfix: missing the cppCompiler arguments to activate OpenMP extensions
+ Added the copy constructor and `multLocal(x,y,z,w)` method
  to the `Quaternion` class.
+ Added the copy constructor and `multLocal(x,y,z)` method
  to the `Vector3f` class.
+ Deprecated the `Vector3f.getX()` and `PhysicsRigidBody.getAngularFactor()`
  methods.

## Version 11.0.0 released on 8 August 2021

+ Bugfix: contact tests report events with positive separation distance
+ Tweaked the behavior of `PhysicsRigidBody.setKinematic()`.
+ Added OpenMP-based multithreading for Linux64 and Windows64 platforms:
  + Added the `countThreads()` and `isThreadSafe()` methods
    to the `NativeLibrary` class.
  + Added 2 build flavors: "SpMt" and "DpMt".
  + In Mt builds, added a pool of contact-and-constraint solvers
    to every `PhysicsSpace`.
+ Added the `destroy()` method to all physics spaces.
+ Added the `add(float, float, float)` and `subtract(float, float, float)`
  methods to the `Vector3f` class.
+ Added the `MyString` utility class.
+ Built using Gradle v6.9 .

## Version 10.5.0 released on 23 June 2021

+ Bugfix: GImpact contact tests always fail (issue #7)
+ Added `hasClosest()` and `hasContact()` methods
  to the `CollisionSpace` class.
+ Added a public `getShapeType()` method to the `CollisionShape` class.

## Version 10.4.0 released on 9 June 2021

+ Implemented tick listeners for the `PhysicsSpace` class.
+ Distinguished ARM macOS/Windows from other platforms.
+ Eliminated all build dependencies on JCenter.
+ Upgraded the Bullet sources to match SHA1 ID=0e124cb2
  of the bullet3 project.

## Version 10.3.1 released on 21 April 2021

+ Bugfix: `NullPointerException` in `Point2PointJoint.setPivotInB()`
+ Upgraded the Bullet sources to match SHA1 ID=00dcc7788
  of the bullet3 project.

## Version 10.3.0 released on 24 February 2021

+ Bugfix: Java copies of `Point2PointJoint` pivot locations
  not updated by setters
+ Bugfix: overflow/underflow in `Vector3f.length()`
+ Added the `setRotation()` method to the `Transform` class.
+ Added the `set(int, int, float)` method to the `Matrix3f` class.
+ Added `lerp()` methods to the `MyMath` and `MyVector3f` classes.
+ Increased use of double-precision arithmetic in the `MyMath` class.
+ Built using Gradle v6.8.3 .

## Version 10.2.0 released on 20 February 2021

+ Added getters and setters for the pivot locations of a `Point2PointJoint`.
+ Tested using JUnit v4.13.2 .

## Version 10.1.0 released on 10 February 2021

+ Bugfix: suspension lengths of a `PhysicsVehicle` are not initialized.
+ Eliminated all `finalize()` methods by implementing a cleaner thread and
  adding an `NpoTracker` class.
+ Published to MavenCentral instead of JCenter.
+ Built using Gradle v6.8.2 .

## Version 10.0.0 released on 24 January 2021

+ Removed the `angularMomentum()` and `kineticEnergy()` methods
  from the `MultiBody` class. (API change)
+ Upgraded the Bullet sources to match SHA1 ID=537ccb220
  of the bullet3 project.
+ Added a return value to `MyQuaternion.validateUnit()`.
+ Reduced argument validation when assertions are disabled.
+ Added an option to `Validate` to configure the type of exception
  thrown for a `null` argument.
+ Publicized some loggers.
+ Built using Gradle v6.8.1 .

## Version 9.3.2 released on 8 January 2021

+ Bugfix: a vehicle's acceleration depends on its location (Minie issue #13).
+ Cleared the user pointer of each `PhysicsCollisionObject` removed
  during `jmeCollisionSpace` destruction.
+ Reverted the null check added to `btAlignedObjectArray::findLinearSearch()`.
+ Built using Gradle v6.8 .

## Version 9.3.1 released on 6 January 2021

+ Build Linux_ARM32hf libraries at Travis-CI.
+ Added a NULL check to `btAlignedObjectArray::findLinearSearch()`.

## Version 9.3.0 released on 5 January 2021

+ Build Linux_ARM32 libraries at Travis-CI.
+ Publicized 2 methods in `MultiBodySpace`.
+ Bugfix: off-by-one in validation of wheel indices
+ Built using Gradle v6.7.1 .

## Version 9.2.4 released on 14 November 2020

 + Built using NDK v21.3.6528147 and Gradle plugin v4.1.1,
   with minSdkVersion=22.
 + Test using JUnit v4.13.1 .

## Version 9.2.3 released on 13 November 2020

Built using Gradle v6.7 and Android SDK v30.

## Version 9.2.2 released on 31 August 2020

Bugfix: collision-group checks are ineffective due to missing parentheses

## Version 9.2.1 released on 31 August 2020

Bugfix: logic error in NewHinge.setLowerLimit()

## Version 9.2.0 released on 30 August 2020

 + Bugfix: RotationOrder.cpp missing from the Android builds
 + Bugfix: JVM crash while reading collision flags of a static rigid body
 + Added the `NewHinge` class.
 + Added methods to the `New6Dof` class:
   + `calculatedOriginA()`
   + `calculatedOriginB()`
 + Added 8 methods to the com.jme3.math package.
 + Built using Gradle v6.6.1 .

## Version 9.1.1 released on 18 August 2020

 + Added the `matrixToEuler()` method to the `RotationOrder` class.
 + Added the `newInstance()` method to the `New6Dof` class.
 + Added the `validateNonZero(Quaternion)` method to the `Validate` class.
 + Added the `fromAngles()` and `normalizeLocal()` methods
   to the `Quaternion` class.
 + Built using Gradle v6.6 .

## Version 9.0.0 released on 8 August 2020

 + Changed the return type of `rayTestRaw()`
   in the `CollisionSpace` class. (API change)
 + Changed the semantics of the `countJoints()` and `listJoints()` methods
   in the `PhysicsBody` class. (API change)
 + Deleted the `getIndices()` and `getNumInts()` methods
   in the `VHACDHull` class. (API change)
 + Delete the deprecated constructor in the `VHACDHull` class. (API change)
 + Privatized and renamed the `physicsJoints` field
   in the `PhysicsSpace` class. (API change)
 + Privatized the `physicsSpaceTL` field
   in the `CollisionSpace` class. (API change)
 + Bugfix: characters and ghosts ignore their own ignore lists!
 + Added a `clearIgnoreList()` method to the `PhysicsCollisionObject` class.
 + Added `findEnd()` and `findOtherBody()` methods to the `PhysicsJoint` class.
 + Built using Gradle Android tools v4.0.1 .

## Version 8.5.0 released on 28 July 2020

 + Added the `MyQuaternion` class from the Heart library.
 + Added `set()`, `addLocal()`, `mult()`, and `multLocal()` methods
   to the `Quaternion` class.
 + Added a `clamp()` method for doubles to the `MyMath` class.

## Version 8.4.0 released on 8 July 2020

 + Bugfix: `PhysicsCharacter` behaves strangely until its location is set.
 + Added 4 methods to the `NativeLibrary` class:
   + `crash()`
   + `fail()`
   + `reinitialization()`
   + `setReinitializationCallbackEnabled()`

## Version 8.3.0 released on 3 July 2020

 + Added collision listeners for ongoing contacts in a `PhysicsSpace`.
 + Added a setter for `gContactCalcArea3Points`.
 + Built using Gradle v6.5.1 .

## Version 8.2.0 released on 25 June 2020

Added getters for the total force and torque on a rigid body.

## Version 8.1.0 released on 18 June 2020

 + Added accessors for the speculative contact restitution flag
   of a `PhysicsSpace`.
 + Deprecated one of the `VHACDHull` constructors.
 + Tested using JUnit v4.13 .

## Version 8.0.0 released on 10 June 2020

 + Added the `static` qualifier to 816 native methods. (API changes)
 + Finalized 2 methods in the `NativePhysicsObject` class. (API changes)
 + Split off `BoundingValueHierarchy`
   from the `MeshCollisionShape` class. (API change)
 + Split off `CharacterController`
   from the `PhysicsCharacter` class. (API change)
 + Split off `VehicleController` from the `PhysicsVehicle` class. (API change)
 + Privatized the `finalizeNative()` method
   in the `Constraint` class. (API change)
 + Privatized 2 protected fields in the `PhysicsVehicle` class.
 + Removed 4 unused methods from "PhysicsRigidBody.cpp". (API changes)
 + Removed 2 unused methods from "MultiBody.cpp". (API changes)
 + Added a native-code fix for JME issue #1351.
 + Added `postInitialization()` and `versionNumber()` native methods to
   the `NativeLibrary` class.
 + Added `getIndex()` to the `VehicleWheel` class.
 + Built using Gradle v6.5 .

## Version 7.0.1 released on 5 June 2020

 + Replaced inner class `PhysicsSoftBody.Material`
   with `SoftBodyMaterial`. (API changes)
 + Deleted the `PhysicsCollisionObject.getObjectId()` method. (API change)
 + Deleted the `isUseSpaceGravity()` and `setUseSpaceGravity()` methods from
   the `PhysicsRigidBody` class. (API changes)
 + Deleted the `createEmptySoftBody()` native method. (API change)

## Version 6.4.0 released on 26 May 2020

 + Added a `copyClusterVelocities()` method to the `PhysicsSoftBody` class.
 + Added an `updatePinMesh()` native method to the `NativeSoftBodyUtil` class.

## Version 6.3.0 released on 24 May 2020

 + Added a `contactTest()` method to the `CollisionSpace` class.
 + Added a `CompoundCollisionShape` constructor to specify initial capacity.

## Version 6.2.0 released on 21 May 2020

 + Added `countIgnored()`, `findInstance()`, and `ignores()` methods
   to the `PhysicsCollisionObject` class.
 + Added `isWorldInfoProtected()` and `setWorldInfoProtected()` methods
   to the `PhysicsSoftBody` class.
 + Used ignore lists to implement the `collisionBetweenLinkedBodies` feature
   of the `Constraint` class.

## Version 6.1.0 released on 18 May 2020

 + bugfix: crash caused by `ShortBuffer` containing index > 32767
 + bugfix: `CompoundCollisionShape.correctAxes()` gave incorrect results for
   some shapes
 + Plugged native memory leaks found in:
   + `Java_vhacd_VHACD_compute()`
   + `jmeCollisionSpace`
   + `jmePhysicsSpace`
   + `SoftBodyWorldInfo`
   + `VehicleTuning`
 + In the `PhysicsRigidBody` class, replaced `isUseSpaceGravity()` with
   `isGravityProtected()` and `setUseSpaceGravity()` with `setProtectGravity()`.
 + Deprecated the `getObjectId()` method in `PhysicsCollisionObject`.
 + Provided Java access to the ignore list of a `PhysicsCollisionObject`.
   Formerly these lists were used only to implement
   `setCollisionBetweenLinkedBodies()` for constraints.
 + Added a `dumpMemoryLeaks()` method to the `NativeLibrary` class.
   This feature requires a native library built
   with `-DBT_DEBUG_MEMORY_ALLOCATIONS`.
 + Added the `toRotationMatrix()` method to the `Matrix4f` class.
 + Built using Gradle v6.4.1 .

## Version 6.0.0 released on 13 May 2020

 + Removed 18 deprecated methods. (API changes)
 + Added arguments for `proxyGroup` and `proxyMask` to the
  `Java_com_jme3_bullet_PhysicsSpace_addRigidBody()` method. (API change)
 + Changed `PhysicsCollisionObject` and `PhysicsCollisionEvent` to be
   subclasses of `NativePhysicsObject`. (API changes)
 + Added a no-arg constructor to the `VHACD` class. (API change)
 + Changed 7 `PhysicsSpace` methods to return unmodifiable collections instead
   of copies:
   + `getCharacterList()`
   + `getGhostObjectList()`
   + `getJointList()`
   + `getMultiBodyList()`
   + `getRigidBodyList()`
   + `getSoftBodyList()`
   + `getVehicleList()`
 + Added `isUseSpaceGravity()` and `setUseSpaceGravity()` methods to the
   `PhysicsRigidBody` class.
 + Added `rotate()` and `translate()` methods
   to the `CompoundCollisionShape` class.
 + Publicized the `addJoint()` and `removeJoint()` methods
   in the `PhysicsSpace` class.
 + Added 2 `mult()` methods to the `Matrix3f` class.
 + Updated the V-HACD sources to match SHA1 ID=2731201 of the v-hacd project.
 + Built using Gradle v6.4 .

## Version 5.8.0 released on 6 May 2020

 + bugfix: `FINE` logging of collision spaces reports `nativeId=0` in `create()`
 + Implemented collision listeners in `PhysicsSpace`.
 + Added tracking of the `PhysicsSpace` where each `PhysicsJoint` is added.
 + Added an `activateAll()` method to the `PhysicsSpace` class.
 + Added `proxyGroup()` and `proxyMask()` methods to the
   `PhysicsCollisionObject` class.
 + Check the `m_objectType` field of every `btTypedConstraint`.
 + Build for Java 7 compatibility.

## Version 5.7.0 released on 1 May 2020

 + bugfix: native crash while finalizing a non-empty `CollisionSpace`
   (JME issue #1351).
 + Added a `NativePhysicsObject` class and refactored 16 classes to
   become its subclasses.
 + Added a constructor for `MeshCollisionShape` that takes a `Collection`
   of native meshes.
 + Added 3 methods to the `PhysicsCollisionObject` class:
   + `getProxyFilterGroup()`
   + `getProxyFilterMask()`
   + `hasBroadphaseProxy()`
 + Deprecated many redundant methods.

## Version 5.6.0 released on 14 April 2020

 + Added 8 methods:
   + `PhysicsCollisionEvent.getCombinedRollingFriction()`
   + `PhysicsCollisionEvent.getCombinedSpinningFriction()`
   + `SolverInfo.isSplitImpulseEnabled()`
   + `SolverInfo.setSplitImpulseEnabled()`
   + `SolverInfo.setSplitImpulseErp()`
   + `SolverInfo.setSplitImpulseThreshold()`
   + `SolverInfo.splitImpulseErp()`
   + `SolverInfo.splitImpulseThreshold()`

## Version 5.5.7 released on 11 April 2020

bugfix: the `btBvhTriangleMeshShape`, `btGImpactShape`, and `btOptimizedBvh`
classes don't support `PHY_UCHAR` mesh indices.

## Version 5.5.6 released on 9 April 2020

Specify the `c++_static` STL for Android builds.

## Version 5.5.4 released on 5 April 2020

bugfix: specify the Android STL in Application.mk, not Android.mk

## Version 5.5.3 released on 4 April 2020

Build native libraries for Android (Sp flavor only).

## Version 5.5.1 released on 30 March 2020

Use GCC 4.7 for Linux-on-AMD builds, to support Centos 7.

## Version 5.5.0 released on 30 March 2020

 + Bugfix: Minie issue #2 (access violations with Java 9+ on Windows).
 + Added 5 methods needed for soft-body construction:
   + `IndexedMesh.copyIndices()`
   + `IndexedMesh.copyVertexPositions()`
   + `NativeSoftBodyUtil.appendFromNativeMesh()`
   + `PhysicsSoftBody.appendFaces()`
   + `PhysicsSoftBody.appendLinks()`
 + Added `IntPair` class.
 + Built using Gradle v6.3.

## Version 5.4.2 released on 25 March 2020

Bugfix: wrong `indexType` passed to `addIndexedMesh()`.

## Version 5.4.1 released on 24 March 2020

Built on trusty (for AMD) and xenial (for ARM).

## Version 5.4.0 released on 24 March 2020

 + Added `getCollisionSpace()` and `spaceId()` methods
   to the `PhysicsCollisionObject` class.
 + Added `boundingBox()` method to the `PhysicsCollisionObject` class.
 + Added the `PcoType` class.
 + Built on bionic.

## Version 5.3.0 released on 22 March 2020

 + Added the `ContactPointFlag` class and also a `getFlags()` method
   for the `PhysicsCollisionEvent` class.
 + Added a `serializeBvh()` method and BVH-based constructor
   to the `MeshCollisionShape` class.
 + Added a `get()` method to the `Matrix3f` class.
 + Added a `setSuspensionLength()` method to the `VehicleWheel` class.

## Version 5.2.0 released on 20 March 2020

Added 3 methods to the `VehicleWheel` class:
 + `getRotationAngle()`
 + `getSuspensionLength()`
 + `setRotationAngle()`

## Version 5.1.1 released on 19 March 2020

 + Added `boundingBox()` methods to the `CollisionShape` class,
   along with the `BoundingBox` class and supporting methods.
 + Added methods `createByte()` and `createShort()` to the `IndexedMesh` class.
 + Specify ABI version 7 for Linux builds.

## Version 5.0.0 released on 11 March 2020

 + API changes:
   + renamed `MultiBodySolver` to `SolverType`
   + renamed `getVertices2()` to `getTriangles()`
     in the `DebugShapeFactory` class
   + access the number of solver iterations via `SolverInfo`
     instead of `PhysicsSpace`
   + deleted the `configureClonedLink()` method from the `MultiBody` class
   + changed the return types of the `addBaseCollider()` and `listColliders()`
     methods in the `MultiBody` class
   + changed the return type of the `addCollider()` method
     in the `MultiBodyLink` class
 + Bugfix: wrong `JNIEnv` used in multithreaded apps.
 + Implemented the `Comparable` interface in the `MultiBody` class.
 + Added support for MLCP and NNCG solvers to the `PhysicsSpace` class.
 + Added 2 new classes: `SolverInfo` and `SolverMode`.
 + Added a `getVertices()` method to the `DebugShapeFactory` class.

## Version 4.2.0 released on 8 March 2020

 + Bugfix: incorrect filter groups and masks when adding a
   collider to a `MultiBodySpace`
 + Bugfix: collider spaces not set when adding to a `MultiBodySpace`
 + Bugfix: unsatisfied link of `MultiBody.getCanSleep()`
 + Added capability to specify the constraint solver for a `MultiBodySpace`.
 + Added 5 methods to the `MultiBodyCollider` class:
   + `getMultiBody()`
   + `linkIndex()`
   + `mass()`
   + `setPhysicsLocation()`
   + `setPhysicsRotation()`
 + Added 2 native methods to the `PhysicsCollisionObject` class:
   + `getCollideWithGroups()`
   + `getCollisionGroup()`

## Version 4.1.1 released on 5 March 2020

 + Added 10 methods to the `MultiBodyLink` class:
   + `axis()`
   + `orientation()`
   + `parent2Link()`
   + `parent2Pivot()`
   + `pivot2Link()`
   + `getAxisBottom()`
   + `getAxisTop()`
   + `getDVector()`
   + `getEVector()`
   + `getQ0Parent2LinkRotation()`
 + Added a `configureClonedLink()` method to the `MultiBody` class.
 + Built using Gradle v6.2.2

## Version 4.0.0 released on 28 February 2020

 + API changes:
   + Isolated collision detection from dynamics by adding 2 new classes:
     `CollisionSpace` and `jmeCollisionSpace`.
   + Deleted the `com.jme3.bullet.debug` package, including `DebugTools`.
   + Deleted the `isNormalInWorldSpace()` methods from the
     `PhysicsRayTestResult` and `PhysicsSweepTestResult` classes.
   + Privatized the `scale` field in the `CompoundMesh` class.
   + Privatized 2 public fields in the `VHACDHull` class.
   + Changed arguments to `ConvexShape` in `Convex2dShape` constructor,
     `PhysicsCharacter` constructor, and `CollisionSpace.sweepTest()`.
   + Changed argument to `float...` in `HullCollisionShape` constructor.
   + Changed arguments in `jmeCollisionShape.createCollisionSpace()`.
   + Changed arguments in `jmePhysicsSpace.createPhysicsSpace()`.
   + Deleted many fields and methods from the `jme3utilities`,
     `jme3utilities.math`, and `com.jme3.math` packages.
 + Implemented multibody/Featherstone support by adding 5 new classes:
   `MultiBody`, `MultiBodyCollider`, `MultiBodyJointType`, `MultiBodyLink`,
   and `MultiBodySpace`.
 + Added new methods:
   + 2 `PhysicsSpace` constructors
   + `VHACDHull.clonePositions()`
   + `PhysicsSpace.getGlobalCfm()`
   + `PhysicsSpace.setGlobalCfm()`
   + `PhysicsSoftSpace.getNumSoftBodies()`
 + Added more `package-info.java` files.
 + Built using Gradle v6.2.1

## Version 3.0.12 released on 18 February 2020

 + Bugfix: `btAssert()` in `btVector3::normalize()` (Minie issue #3).
 + Added `countCollisionObjects()` and `getWorldType()` methods
   to the `PhysicsSpace` class.
 + Added more automated tests to `TestLibbulletjme`.
 + Added `package-info.java` files.
 + Built using Gradle v6.2

## Version 3.0.8 released on 16 February 2020

 + Added an abstract class `ConvexShape`.
 + Added `getStepHeight()`, `getWalkOffset()`, and `isUsingGhostSweepTest()`
   methods to the `PhysicsCharacter` class.

## Version 3.0.6 released on 12 February 2020

 + Implemented filtering of predictive contacts (JME issue 1283).
 + Added validation of angular limits to the `SixDofJoint` class.
 + Added a `VF_DD` flag to the `ConfigFlag` class.

## Version 3.0.5 released on 11 February 2020

 + Build native libraries for `Linux_ARM64` platforms.
 + Added more automated tests to `TestLibbulletjme`.
 + Added a `getDebugTriangles()` method to the `DebugShapeFactory` class.
 + Added a `generateBasis()` method to the `MyVector3f` class.
 + Added a `hashCode()` method to the `VHACDParameters` class.

## Version 3.0.4 released on 6 February 2020

 + Build a class JAR, a javadoc JAR, a sources JAR, and a POM.
 + Added automated testing with Junit.
 + Moved all source files from "src" to "src/main".

## Version 3.0.2 released on 3 February 2020

 + Added a `getNbPinnedNodes()` method to the `PhysicsSoftBody` class.
 + Built using Gradle v6.1.1

## Version 3.0.1 released on 21 January 2020

 + Provided part and triangle indices from ray tests and sweep tests.
 + Cleaned up the handling of results from ray tests and sweep tests.
 + Built using Gradle v6.1

## Version 3.0.0 released on 14 January 2020

 + Removed 6 JNI methods:
   + `CompoundCollisionShape.createShape()`
   + `PhysicsCharacter.getCcdMotionThreshold()`
   + `PhysicsCharacter.getCcdSquareMotionThreshold()`
   + `PhysicsCharacter.getCcdSweptSphereRadius()`
   + `PhysicsCharacter.setCcdMotionThreshold()`
   + `PhysicsCharacter.setCcdSweptSphereRadius()`
 + Replaced `jobject` argument with `jlong` in `PhysicsVehicle.addWheel()`.
 + Replaced `ByteBuffer` arguments with `FloatBuffer` in `HullCollisionShape`.
 + Removed unused arguments of 2 JNI methods:
   + `PhysicsSpace.createPhysicsSpace()`
   + `PhysicsVehicle.createVehicleRaycaster()`
 + Changed the return-type of 4 JNI methods:
   + `CompoundCollisionShape.addChildShape()`
   + `CompoundCollisionShape.removeChildShape()`
   + `PhysicsRigidBody.updateMassProps()`
   + `SoftBodyWorldInfo.setSoftBodyWorldInfo()`
 + Added a new `VehicleTuning` class.
 + Added a new `PhysicsRigidBody.getMass()` method.

## Version 2.0.23 released on 13 January 2020

 + Updated the V-HACD sources to match SHA1 ID=b07958e1 of the v-hacd project.
   This eliminated the depth and gamma properties from `VHACDParameters`.
 + Began reporting V-HACD progress to a `VHACD.update()` method.
 + Switched from Visual Studio 2017 to Visual Studio 2019 for Windows builds.

## Version 2.0.22 released on 13 January 2020

Solved a compile-time error on platforms where jint != int.

## Version 2.0.21 released on 12 January 2020

Added V-HACD sources from March 2016 (SHA1 ID=ded1fe4) plus new JNI glue code.

## Version 2.0.20 released on 10 January 2020

 + Fixed a `btAssert()` that occurred (in Debug builds) while rescaling
   a rigid body with a `MeshCollisionShape`.
 + Upgraded the Bullet sources to match version 2.89 (SHA1 ID=830f0a956)
   of the bullet3 project.

## Version 2.0.19 released on 29 December 2019

 + Added `getForwardAxisIndex()`, `getRightAxisIndex()`, `getNumWheels()`,
   `getUpAxisIndex()`, and `rayCast()` methods to the `PhysicsVehicle` class.
 + Added `getBrake()`, `getEngineForce()`, `getRadius()`, `getRestLength()`,
   `getRollInfluence()`, `getSteerAngle()`, and `isFront()` methods to the
   `VehicleWheel` class.
 + Added many assertions to the `PhysicsVehicle` class.

## Version 2.0.18 released on 28 December 2019

Added support for `btBox2dShape` and `btConvex2dShape`
(new classes `Box2dShape` and `Convex2dShape`).

## Version 2.0.17 released on 16 December 2019

 + Added `getAngles()`, `getAxis()`, `getFrameOffsetA()`, `getFrameOffsetB()`,
   `getPivotOffset()`, `getRotationOrder()`, and `setRotationOrder()` methods
   to the `New6Dof` class.
 + Added range checks for axis indices in `TranslationMotor`.

## Version 2.0.16 released on 14 December 2019

Added support for `btGeneric6DofSpring2Constrant` (new class `New6Dof`).

## Version 2.0.14 released on 6 December 2019

 + Reverted the Bullet sources to match SHA1 id=1981493a of the bullet3 project.
 + Built using Gradle v6.0.1

## Version 2.0.12 released on 4 November 2019

Updated the Bullet sources to match SHA1 id=aac737017 of the bullet3 project,
to resolve soft-body performance issues at id=837e333.

## Version 2.0.11 released on 14 October 2019

 + Added `getDamping()`, `getEquilibriumPoint()`, `getStiffness()`, and
   `isSpringEnabled()` methods to the `SixDofSpringJoint` class.
 + Updated the Bullet sources to match SHA1 id=837e333 of the bullet3 project.

## Version 2.0.10 released on 10 September 2019

 + Added `calculatePrincipalAxisTransform()`, `countChildren()`,
   `createShape2()`, and `setChildTransform()` methods to the
   `CompoundCollisionShape` class.
 + Disabled NULL_CHECKs except in Debug builds.

## Version 2.0.9 released on 6 September 2019

 + Added `getActivationState()` and `setActivationState()` methods to the
   `PhysicsCollisionObject` class.
 + Added `isConvex()`, `isInfinite()`, `isNonMoving()`, and `isPolyhedral()`
   methods to the `CollisionShape` class.
 + Added more internal checks.

## Version 2.0.8 released on 6 September 2019

 + Added a `getSquaredSpeed()` method to the `PhysicsRigidBody` class.
 + Extensive refactoring and reformatting.

## Version 2.0.7 released on 13 August 2019

 + Added a `getGravity()` method to the `PhysicsSpace` class.
 + Added more internal type checks.

## Version 2.0.5 released on 7 August 2019

 + Added a `setAccumulatedImpulse()` method to the `RotationalLimitMotor` class.
 + Added internal type checks to joint constructors.
 + Updated the Bullet sources to match SHA1 id=1981493a of the bullet3 project.

## Version 2.0.4 released on 3 August 2019

 + Added `isEnabled()` and `setEnabled()` methods to the
   `TranslationalLimitMotor` class.
 + Added assertions to verify the internal type in `HullCollisionShape`,
   `PhysicsRigidBody`, and `PhysicsSoftBody`.

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

 + Added a `getShapeType()` method to the `CollisionShape` class.
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