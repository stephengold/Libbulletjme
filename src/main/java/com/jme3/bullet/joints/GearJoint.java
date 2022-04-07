package com.jme3.bullet.joints;

import java.util.logging.Logger;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;

import jme3utilities.Validate;

/**
 * A joint that couples the angular velocity for two bodies based on Bullet's
 * btGearConstraint.
 * <p>
 * <i>From the Bullet manual:</i><br>
 * <p>
 * The btGearConstraint will couple the angular velocity for two bodies around given local axis and ratio. 
 * 
 * @author elmfrain
 */
public class GearJoint extends Constraint {
	// *************************************************************************
    // constants and loggers

	/**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(GearJoint.class.getName());
    // *************************************************************************
    // fields
    
    /**
     * copy of the joint axis in A's local coordinates (unit vector)
     */
    final private Vector3f axisA;
    /**
     * copy of the joint axis: in B's local coordinates (unit vector)
     */
    final private Vector3f axisB;
    /**
     * copy of the joint ratio; gear ratio
     */
    final private float ratio;
    // *************************************************************************
    // constructors
    
    public GearJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f axisInA, Vector3f axisInB) {
        super(rigidBodyA, rigidBodyB, Vector3f.ZERO, Vector3f.ZERO);
        
        assert axisInA.isUnitVector() : axisInA;
        assert axisInB.isUnitVector() : axisInB;
        
        axisA = axisInA.clone();
        axisB = axisInB.clone();
        ratio = 1.0f;
        createJoint();
    }
    
    public GearJoint(PhysicsRigidBody rigidBodyA, PhysicsRigidBody rigidBodyB,
            Vector3f axisInA, Vector3f axisInB, float ratio) {
        super(rigidBodyA, rigidBodyB, Vector3f.ZERO, Vector3f.ZERO);
        
        assert axisInA.isUnitVector() : axisInA;
        assert axisInB.isUnitVector() : axisInB;
        
        axisA = axisInA.clone();
        axisB = axisInB.clone();
        this.ratio = ratio;
        createJoint();
    }
    // *************************************************************************
    // new methods exposed
    
    /**
     * Copy the joint's rotation axis in body A.
     * 
     * @param storeResult storage for the result (modified if not null)
     * @return The rotation axis in body A (either storeResult or new vector, if
     * not null)
     */
    public Vector3f getAxisA(Vector3f storeResult) {
        Vector3f result
            = (storeResult == null) ? new Vector3f() : storeResult;
        
        long constraintId = nativeId();
        getAxisA(constraintId, result);
        
        return result;
    }
    
    /**
     * Copy the joint's rotation axis in body B.
     * 
     * @param storeResult storage for the result (modified if not null)
     * @return The rotation axis in body B (either storeResult or new vector, if
     * not null)
     */
    public Vector3f getAxisB(Vector3f storeResult) {
        Vector3f result
        = (storeResult == null) ? new Vector3f() : storeResult;
        
        long constraintId = nativeId();
        getAxisB(constraintId, result);
        
        return result;
    }
    
    /**
     * Get the joint's gear ratio.
     * 
     * @return The gear ratio
     */
    public float getRatio() {
        long constraintId = nativeId();
        return getRatio(constraintId);
    }
    
    /**
     * Alter the joint's rotation axis in body A.
     * 
     * @param axisA the rotation axis in body A (unit vector, not null, unaffected)
     */
    public void setAxisA(Vector3f axisA) {
        Validate.unitVector(axisA, "Axis in body A");
        
        long constraintId = nativeId();
        setAxisA(constraintId, axisA);
    }
    
    /**
     * Alter the joint's rotation axis in body B.
     * 
     * @param axisA the rotation axis in body B (unit vector, not null, unaffected)
     */
    public void setAxisB(Vector3f axisB) {
        Validate.unitVector(axisA, "Axis in body B");
        
        long constraintId = nativeId();
        setAxisB(constraintId, axisB);
    }
    
    /**
     * Alter the joint's gear ratio.
     * 
     * @param ratio the gear ratio
     */
    public void setRatio(float ratio) {
        long constraintId = nativeId();
        setRatio(constraintId, ratio);
    }
    // *************************************************************************
    // Java private methods
    
    /**
     * Create the configured joint in Bullet.
     */
    private void createJoint() {
        PhysicsRigidBody a = getBodyA();
        long aId = a.nativeId();
        assert axisA.isUnitVector();
        
        PhysicsRigidBody b = getBodyB();
        long bId = b.nativeId();
        assert axisB.isUnitVector();
        
        long constraintId;
        /*
         * Create a double-ended joint.
         */
        constraintId = createJoint(aId, bId, axisA, axisB, ratio);
        
        assert getConstraintType(constraintId) == 10;
        setNativeId(constraintId);
    }
    // *************************************************************************
    // native private methods
    
    native private static long createJoint(long objectIdA, long objectIdB,
            Vector3f axisInA, Vector3f axisInB, float ratio);
    
    native private static void getAxisA(long jointId, Vector3f storeResult);
    
    native private static void getAxisB(long jointId, Vector3f storeResult);
    
    native private static float getRatio(long jointId);
    
    native private static void setAxisA(long jointId, Vector3f axisA);
    
    native private static void setAxisB(long jointId, Vector3f axisB);
    
    native private static void setRatio(long jointId, float ratio);
}
