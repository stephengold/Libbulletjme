package com.jme3.bullet.joints;

import java.util.logging.Logger;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;

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
    
    public Vector3f getAxisA(Vector3f storeResult) {
        Vector3f result
            = (storeResult == null) ? new Vector3f() : storeResult;
        
        long constraintId = nativeId();
        getAxisA(constraintId, result);
        
        return result;
    }
    
    public Vector3f getAxisB(Vector3f storeResult) {
        Vector3f result
        = (storeResult == null) ? new Vector3f() : storeResult;
        
        long constraintId = nativeId();
        getAxisB(constraintId, result);
        
        return result;
    }
    
    public float getRatio() {
        long constraintId = nativeId();
        return getRatio(constraintId);
    }
    
    public void setAxisA(Vector3f axisA) {
        long constraintId = nativeId();
        setAxisA(constraintId, axisA);
    }
    
    public void setAxisB(Vector3f axisB) {
        long constraintId = nativeId();
        setAxisB(constraintId, axisB);
    }
    
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
    
    native private static void getAxisA(long jointId, Vector3f axisA);
    
    native private static void getAxisB(long jointId, Vector3f axisB);
    
    native private static float getRatio(long jointId);
    
    native private static void setAxisA(long jointId, Vector3f axisA);
    
    native private static void setAxisB(long jointId, Vector3f axisB);
    
    native private static void setRatio(long jointId, float ratio);
}
