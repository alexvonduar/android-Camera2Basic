//
// Created by alex on 17-10-17.
//
#include <jni.h>

#include "debug_log.h"
#include "image-quality.hpp"

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_eyes_zero_helper_ImageUtil_YuvQuality(
        JNIEnv *env,
        jclass /*clazz*/,
        jbyteArray data,
        jint width,
        jint height) {

    jbyte * buffer;
    jboolean isCopy;
    buffer = env->GetByteArrayElements(data, &isCopy);
    const unsigned char * pbuffer = (const unsigned char *)buffer;
    jdouble result = yuv_quality(pbuffer, width, height);
    env->ReleaseByteArrayElements(data, buffer, 0);
    return result;
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_com_eyes_zero_helper_ImageUtil_JpegQuality(JNIEnv *env, jobject instance,
                                                jstring fileName_) {
    const char *fileName = env->GetStringUTFChars(fileName_, 0);
    ZEYES_LOG_INFO("file %s", fileName);
    double result = jpeg_quality(fileName);
    return result;
}
