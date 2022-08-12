/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_jme3_bullet_collision_shapes_MeshCollisionShape */

#ifndef _Included_com_jme3_bullet_collision_shapes_MeshCollisionShape
#define _Included_com_jme3_bullet_collision_shapes_MeshCollisionShape
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    createShape
 * Signature: (ZZJ)J
 */
JNIEXPORT jlong JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_createShape
  (JNIEnv *, jclass, jboolean, jboolean, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    recalcAabb
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_recalcAabb
  (JNIEnv *, jclass, jlong);

/*
 * Class:     com_jme3_bullet_collision_shapes_MeshCollisionShape
 * Method:    setOptimizedBvh
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_collision_shapes_MeshCollisionShape_setOptimizedBvh
  (JNIEnv *, jclass, jlong, jlong);

#ifdef __cplusplus
}
#endif
#endif