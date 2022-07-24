/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_jme3_bullet_util_NativeLibrary */

#ifndef _Included_com_jme3_bullet_util_NativeLibrary
#define _Included_com_jme3_bullet_util_NativeLibrary
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    countThreads
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_util_NativeLibrary_countThreads
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    crash
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeLibrary_crash
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    dumpMemoryLeaks
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_util_NativeLibrary_dumpMemoryLeaks
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    dumpQuickprof
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_jme3_bullet_util_NativeLibrary_dumpQuickprof
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    fail
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeLibrary_fail
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    isDebug
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_NativeLibrary_isDebug
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    isDoublePrecision
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_NativeLibrary_isDoublePrecision
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    isInsideTriangle
 * Signature: (Lcom/jme3/math/Vector3f;FLcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;Lcom/jme3/math/Vector3f;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_NativeLibrary_isInsideTriangle
  (JNIEnv *, jclass, jobject, jfloat, jobject, jobject, jobject);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    isQuickprof
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_NativeLibrary_isQuickprof
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    isThreadSafe
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_com_jme3_bullet_util_NativeLibrary_isThreadSafe
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    resetQuickprof
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeLibrary_resetQuickprof
  (JNIEnv *, jclass);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    setReinitializationCallbackEnabled
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeLibrary_setReinitializationCallbackEnabled
  (JNIEnv *, jclass, jboolean);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    setStartupMessageEnabled
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_com_jme3_bullet_util_NativeLibrary_setStartupMessageEnabled
  (JNIEnv *, jclass, jboolean);

/*
 * Class:     com_jme3_bullet_util_NativeLibrary
 * Method:    versionNumber
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_jme3_bullet_util_NativeLibrary_versionNumber
  (JNIEnv *, jclass);

#ifdef __cplusplus
}
#endif
#endif