/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy */

#ifndef _Included_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
#define _Included_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    deSerialize
 * Signature: ([B)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_deSerialize
  (JNIEnv *, jclass, jbyteArray);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    finalizeNative
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_finalizeNative
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getAabb
 * Signature: (JLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getAabb
  (JNIEnv *, jclass, jlong, jobject, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getEscapeIndex
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getEscapeIndex
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getNumLeafNodes
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getNumLeafNodes
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getNumNodes
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getNumNodes
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getNumSubtreeHeaders
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getNumSubtreeHeaders
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getOptimizedBvh
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getOptimizedBvh
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getPartId
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getPartId
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getQuantization
 * Signature: (JLcom/jme3/math/Vector3f;)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getQuantization
  (JNIEnv *, jclass, jlong, jobject);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getTraversalMode
 * Signature: (J)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getTraversalMode
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    getTriangleIndex
 * Signature: (JI)I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_getTriangleIndex
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    isCompressed
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_isCompressed
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    isLeafNode
 * Signature: (JI)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_isLeafNode
  (JNIEnv *, jclass, jlong, jint);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    serialize
 * Signature: (J)[B
 */
JNIEXPORT jbyteArray JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_serialize
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy
 * Method:    setTraversalMode
 * Signature: (JI)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_infos_BoundingValueHierarchy_setTraversalMode
  (JNIEnv *, jclass, jlong, jint);

#ifdef __cplusplus
}
#endif
#endif
