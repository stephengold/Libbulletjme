/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_jme3_bullet_collision_shapes_CompoundCollisionShape */

#ifndef _Included_com_jme3_bullet_collision_shapes_CompoundCollisionShape
#define _Included_com_jme3_bullet_collision_shapes_CompoundCollisionShape
#ifdef __cplusplus
extern "C" {
#endif
#undef com_jme3_bullet_collision_shapes_CompoundCollisionShape_defaultCapacity
#define com_jme3_bullet_collision_shapes_CompoundCollisionShape_defaultCapacity 6L
/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    addChildShape
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_addChildShape
  (JNIEnv *, jobject, jlong, jlong, jobject, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    calculatePrincipalAxisTransform
 * Signature: (JLjava/nio/FloatBuffer;Lcom/jme3/math/Transform;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_calculatePrincipalAxisTransform
  (JNIEnv *, jobject, jlong, jobject, jobject, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    countChildren
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_countChildren
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    createShape2
 * Signature: (ZI)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_createShape2
  (JNIEnv *, jobject, jboolean, jint);

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    recalcAabb
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_recalcAabb
  (JNIEnv *, jobject, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    removeChildShape
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_removeChildShape
  (JNIEnv *, jobject, jlong, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    rotate
 * Signature: (JLcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_rotate
  (JNIEnv *, jobject, jlong, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    setChildTransform
 * Signature: (JJLcom/jme3/math/Vector3f;Lcom/jme3/math/Matrix3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_setChildTransform
  (JNIEnv *, jobject, jlong, jlong, jobject, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_CompoundCollisionShape
 * Method:    translate
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_CompoundCollisionShape_translate
  (JNIEnv *, jobject, jlong, jobject);

#ifdef __cplusplus
}
#endif
#endif
