/*
 * Copyright (c) 2005 Erwin Coumans https://bulletphysics.org
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_RAYCASTVEHICLE_H
#define BT_RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class btDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

//class btVehicleTuning;

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btRaycastVehicle : public btActionInterface
{
	btAlignedObjectArray<btVector3> m_forwardWS;
	btAlignedObjectArray<btVector3> m_axle;
	btAlignedObjectArray<btScalar> m_forwardImpulse;
	btAlignedObjectArray<btScalar> m_sideImpulse;

	///backwards compatibility
	int m_userConstraintType;
	int m_userConstraintId;

public:
	class btVehicleTuning
	{
	public:
		btVehicleTuning()
			: m_suspensionStiffness(btScalar(5.88)),
			  m_suspensionCompression(btScalar(0.83)),
			  m_suspensionDamping(btScalar(0.88)),
			  m_maxSuspensionTravelCm(btScalar(500.)),
			  m_frictionSlip(btScalar(10.5)),
			  m_maxSuspensionForce(btScalar(6000.))
		{
		}
		btScalar m_suspensionStiffness;
		btScalar m_suspensionCompression;
		btScalar m_suspensionDamping;
		btScalar m_maxSuspensionTravelCm;
		btScalar m_frictionSlip;
		btScalar m_maxSuspensionForce;
	};

private:
	btVehicleRaycaster* m_vehicleRaycaster;
	btScalar m_pitchControl;
	btScalar m_steeringValue;
	btScalar m_currentVehicleSpeedKmHour;

	btRigidBody* m_chassisBody;

	int m_indexRightAxis;
	int m_indexUpAxis;
	int m_indexForwardAxis;
	btMatrix3x3 m_rfu2Local; // stephengold added 2021-09-28

	void defaultInit(const btVehicleTuning& tuning);

public:
	//constructor to create a car from an existing rigidbody
	btRaycastVehicle(const btVehicleTuning& tuning, btRigidBody* chassis, btVehicleRaycaster* raycaster);

	virtual ~btRaycastVehicle();

	///btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step)
	{
		(void)collisionWorld;
		updateVehicle(step);
	}

	///btActionInterface interface
	void debugDraw(btIDebugDraw* debugDrawer);

	const btTransform& getChassisWorldTransform() const;

	btScalar rayCast(btWheelInfo& wheel);

	virtual void updateVehicle(btScalar step);

	void resetSuspension();

	btScalar getSteeringValue(int wheel) const;

	void setSteeringValue(btScalar steering, int wheel);

	void applyEngineForce(btScalar force, int wheel);

	const btTransform& getWheelTransformWS(int wheelIndex) const;

	void updateWheelTransform(int wheelIndex, bool interpolatedTransform = true);

	//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);

	btWheelInfo& addWheel(const btVector3& connectionPointCS0, const btVector3& wheelDirectionCS0, const btVector3& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel);

	inline int getNumWheels() const
	{
		return int(m_wheelInfo.size());
	}

	btAlignedObjectArray<btWheelInfo> m_wheelInfo;

	const btWheelInfo& getWheelInfo(int index) const;

	btWheelInfo& getWheelInfo(int index);

	void updateWheelTransformsWS(btWheelInfo& wheel, bool interpolatedTransform = true);

	void setBrake(btScalar brake, int wheelIndex);

	void setPitchControl(btScalar pitch)
	{
		m_pitchControl = pitch;
	}

	void updateSuspension(btScalar deltaTime);

	virtual void updateFriction(btScalar timeStep);

	inline btRigidBody* getRigidBody()
	{
		return m_chassisBody;
	}

	const btRigidBody* getRigidBody() const
	{
		return m_chassisBody;
	}

	inline int getRightAxis() const
	{
		return m_indexRightAxis;
	}
	inline int getUpAxis() const
	{
		return m_indexUpAxis;
	}

	inline int getForwardAxis() const
	{
		return m_indexForwardAxis;
	}

	///Worldspace forward vector
	btVector3 getForwardVector() const
	{
		const btTransform& chassisTrans = getChassisWorldTransform();

        btVector3 forward = m_rfu2Local.getColumn(1);// stephengold changed 2021-09-28
        btVector3 forwardW = chassisTrans.getBasis() * forward;// stephengold changed 2021-09-28

		return forwardW;
	}

	///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
	btScalar getCurrentSpeedKmHour() const
	{
		return m_currentVehicleSpeedKmHour;
	}

	virtual void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex)
	{
        btVector3 right, up, forward;// stephengold added 2021-09-28
        switch (rightIndex) {// stephengold added 2021-09-28
            case 0: right.setValue(1,0,0); break;// stephengold added 2021-09-28
            case 1: right.setValue(0,1,0); break;// stephengold added 2021-09-28
            case 2: right.setValue(0,0,1); break;// stephengold added 2021-09-28
        }// stephengold added 2021-09-28
        switch (upIndex) {// stephengold added 2021-09-28
            case 0: up.setValue(1,0,0); break;// stephengold added 2021-09-28
            case 1: up.setValue(0,1,0); break;// stephengold added 2021-09-28
            case 2: up.setValue(0,0,1); break;// stephengold added 2021-09-28
        }// stephengold added 2021-09-28
        switch (forwardIndex) {// stephengold added 2021-09-28
            case 0: forward.setValue(1,0,0); break;// stephengold added 2021-09-28
            case 1: forward.setValue(0,1,0); break;// stephengold added 2021-09-28
            case 2: forward.setValue(0,0,1); break;// stephengold added 2021-09-28
        }// stephengold added 2021-09-28
        setupCoordinateSystem(right, up, forward);// stephengold added 2021-09-28
		m_indexRightAxis = rightIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}

	virtual void setupCoordinateSystem(btVector3 right, btVector3 up, btVector3 forward) // stephengold added 2021-09-28
    { // stephengold added 2021-09-28
        btMatrix3x3 rfu = btMatrix3x3(right, forward, up); // stephengold added 2021-09-28
        m_rfu2Local = rfu.transpose(); // stephengold added 2021-09-28
		m_indexRightAxis = -1; // stephengold added 2021-09-28
		m_indexUpAxis = -1; // stephengold added 2021-09-28
		m_indexForwardAxis = -1; // stephengold added 2021-09-28
    } // stephengold added 2021-09-28
    
	///backwards compatibility
	int getUserConstraintType() const
	{
		return m_userConstraintType;
	}

	void setUserConstraintType(int userConstraintType)
	{
		m_userConstraintType = userConstraintType;
	};

	void setUserConstraintId(int uid)
	{
		m_userConstraintId = uid;
	}

	int getUserConstraintId() const
	{
		return m_userConstraintId;
	}
};

class btDefaultVehicleRaycaster : public btVehicleRaycaster
{
	btDynamicsWorld* m_dynamicsWorld;

public:
	btDefaultVehicleRaycaster(btDynamicsWorld* world)
		: m_dynamicsWorld(world)
	{
	}

	virtual void* castRay(const btVector3& from, const btVector3& to, btVehicleRaycasterResult& result);
};

#endif  //BT_RAYCASTVEHICLE_H
