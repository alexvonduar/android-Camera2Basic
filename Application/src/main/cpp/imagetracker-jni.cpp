//
// Created by alex on 18-4-10.
//

#include <jni.h>

#include <android/bitmap.h>

#include <deque>
#include <vector>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <pthread.h>

#include <opencv2/opencv.hpp>

#include "debug_log.h"

#include "turbojpeg.h"

#include "neonorb.hpp"

#define ZEYES_LOG_LEVEL ZEYES_LOG_LEVEL_VERBOSE


#include "ORBextractor.h"
#include "imagetracker.hpp"

//static Tracker *tracker = NULL;
static int running = 0;

static bool save_djpg = false;

bool
tjpeg_file2rgb(const uchar *const &jpeg_buffer, const int &jpeg_size, cv::Mat &rgb) {
    tjhandle handle = NULL;
    int width, height, subsample, colorspace;
    int flags = 0;
    int padding = 1; // 1或4均可，但不能是0
    int ret = 0;

    unsigned char *trans_buffer = NULL;
    unsigned long trans_size = 0;
    tjtransform tf;
    memset(&tf, 0, sizeof(tjtransform));
    tf.op = TJXOP_ROT270;
    tjhandle trans_handle = tjInitTransform();
    tjTransform(trans_handle, jpeg_buffer, jpeg_size, 1, &trans_buffer, &trans_size, &tf, flags);

    handle = tjInitDecompress();
    tjDecompressHeader3(handle, trans_buffer, trans_size, &width, &height, &subsample, &colorspace);

    ZEYES_LOG_DEBUG("w: %d h: %d subsample: %d color: %d\n", width, height, subsample, colorspace);

    //flags |= 0;

    rgb.release();
    rgb = cv::Mat(height, width, CV_8UC3);

    ret = tjDecompress2(handle, trans_buffer, trans_size, rgb.ptr<uchar>(), width, 0, height,
                        TJPF_RGB, TJFLAG_FASTDCT);

    if (save_djpg) {
        cv::imwrite(
                "/storage/emulated/0/Android/data/com.example.android.camera2basic/files/snap_rgb.jpg",
                rgb);
        save_djpg = false;
    }
    tjFree(trans_buffer);
    if (ret < 0) {
        ZEYES_LOG_ERROR("decompress jpeg failed: %s\n", tjGetErrorStr());
        rgb.release();
        return false;
    }

    ZEYES_LOG_DEBUG("Decod finished");
    tjDestroy(handle);

    return true;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_ImageTracker_start(
        JNIEnv *env,
        jclass /*clazz*/) {
    //tracker = new Tracker();
    Tracker::getInstance()->start();
    running = 1;
    return true;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_ImageTracker_stop(
        JNIEnv *env,
        jclass /*clazz*/) {
    running = false;
    //if (tracker != NULL) {
    //    delete tracker;
    //}
    //tracker = NULL;
    Tracker::getInstance()->stop();
    return true;
}

extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_example_android_camera2basic_ImageTracker_getoverlaprect(
        JNIEnv *env,
        jclass /*clazz*/
) {
    jfloatArray result;
    //int size = intersections[rectangle_idx].size() * 2 + 1;
    std::vector<cv::Point2f> hintbox;
    bool valid_overlap;
    bool valid_hint;
    Tracker::getInstance()->get_hintbox(hintbox, valid_overlap, valid_hint);
    int size = hintbox.size() * 2 + 2;

    if (size == 2) {
        //
        valid_overlap = false;
        //valid_hint = true;
        hintbox.resize(4);
        hintbox[0] = cv::Point2f(0, 0);
        hintbox[1] = cv::Point2f(1.0, 0);
        hintbox[2] = cv::Point2f(1.0, 1.0);
        hintbox[3] = cv::Point2f(0, 1.0);
        size = 10;
    }

    result = env->NewFloatArray(size);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    float *tmp = new float[size];
    for (int i = 0; i < hintbox.size(); ++i) {
        tmp[i * 2] = hintbox[i].x;
        tmp[i * 2 + 1] = hintbox[i].y;
    }
    tmp[size - 2] = valid_overlap ? 1.0 : 0.0;
    tmp[size - 1] = valid_hint ? 1.0 : 0.0;
    // move from the temp structure to the java structure
    env->SetFloatArrayRegion(result, 0, size, tmp);
    delete[] tmp;
    return result;
}

static bool save = false;

static inline void
convert_yuv420_nv21_to_bgra(const uchar *buffer, const int &width, const int &height,
                            cv::Mat &rgb) {
    cv::Mat input(height + height / 2, width, CV_8UC1, (void *) buffer);
    rgb = cv::Mat(height, width, CV_8UC3);
    cv::cvtColor(input, rgb, CV_YUV2RGB_NV21);
    if (save) {
        cv::imwrite(
                "/storage/emulated/0/Android/data/com.example.android.camera2basic/files/preview_rgb.jpg",
                rgb);
        save = false;
    }
}

static inline void
convert_yuv420_nv21_to_bgra(const uchar *y, const uchar *uv, const int &width, const int &y_stride,
                            const int &uv_stride, const int &height, cv::Mat &rgb) {
    assert(y_stride >= width);
    assert(uv_stride == y_stride);
    int y_stride_size = y_stride * height;
    if (y + y_stride_size != uv) {
        // need copy
        assert(0);
    } else {
        cv::Mat input(height + height / 2, y_stride, CV_8UC1, (void *) y);
        cv::Mat full_rgb = cv::Mat(height, y_stride, CV_8UC3);
        cv::cvtColor(input, full_rgb, CV_YUV2RGB_NV21);
        cv::Rect rect(0, 0, width, height);
        rgb = full_rgb(rect);
    }
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_ImageTracker_track(
        JNIEnv *env,
        jclass /*clazz*/,
        jbyteArray data,
        jint width,
        jint height,
        jint rotation,
        jboolean is_nv21,
        jboolean is_keyframe) {

    if (running == false) {
        ZEYES_LOG_DEBUG("DEADBEAF not started\n");
        return true;
    }
    //assert(tracker != NULL);

    jbyte *buffer;
    jboolean isCopy;
    buffer = env->GetByteArrayElements(data, &isCopy);
    const uchar *pbuffer = (const uchar *) buffer;

    cv::Mat rgb;
    if (is_nv21) {
        convert_yuv420_nv21_to_bgra(pbuffer, width, height, rgb);
    }
    bool result = Tracker::getInstance()->track(rgb, rotation, is_keyframe);

    env->ReleaseByteArrayElements(data, buffer, 0);

    return result;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_ImageTracker_track2(
        JNIEnv *env,
        jclass /*clazz*/,
        jbyteArray y,
        jbyteArray uv,
        jint width,
        jint height,
        jint y_stride,
        jint uv_stride,
        jint rotation) {

    if (running == false) {
        ZEYES_LOG_DEBUG("DEADBEAF not started\n");
        return true;
    }
    //assert(tracker != NULL);

    jbyte *y_buffer;
    jbyte *uv_buffer;
    jboolean isCopy;
    y_buffer = env->GetByteArrayElements(y, &isCopy);
    uv_buffer = env->GetByteArrayElements(uv, &isCopy);
    const uchar *py_buffer = (const uchar *) y_buffer;
    const uchar *puv_buffer = (const uchar *) uv_buffer;

    cv::Mat rgb;
    convert_yuv420_nv21_to_bgra(py_buffer, puv_buffer, width, y_stride, uv_stride, height, rgb);
    Tracker::getInstance()->track(rgb, rotation, false);

    env->ReleaseByteArrayElements(y, y_buffer, 0);

    return true;
}

#if 0
extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_ImageTracker_snap(
        JNIEnv *env,
        jclass /*clazz*/,
        jbyteArray data,
        //jint width,
        //jint height,
        //jint stride,
        jint jpeg_size) {

    //ZEYES_LOG_INFO("DEADBEAF native %d %d", width, height);

    if (running == false) {
        return false;
    }

    //ZEYES_LOG_DEBUG("DEADBEAF native 3 %d %d", width, height);
    jbyte *buffer;
    jboolean isCopy;
    buffer = env->GetByteArrayElements(data, &isCopy);
    const uchar *pbuffer = (const uchar *) buffer;

    ZEYES_LOG_DEBUG("DEADBEAF %d\n", __LINE__);
    cv::Mat original;
    bool ret = tjpeg_file2rgb(pbuffer, jpeg_size, original);
    if (ret) {
        tracker->track(original, true);
    }

    env->ReleaseByteArrayElements(data, buffer, 0);

    return true;
}
#endif

extern "C"
JNIEXPORT void JNICALL
Java_com_example_android_camera2basic_ImageTracker_renderPanorama(JNIEnv *env, jobject obj,
                                                                  jobject bitmap) {
    AndroidBitmapInfo info;
    void *pixels;
    int ret;

    if ((ret = AndroidBitmap_getInfo(env, bitmap, &info)) < 0) {
        ZEYES_LOG_ERROR("AndroidBitmap_getInfo() failed ! error=%d", ret);
        return;
    }

    if (info.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
        ZEYES_LOG_ERROR("Bitmap format is not RGBA_8888 !");
        return;
    }

    if ((ret = AndroidBitmap_lockPixels(env, bitmap, &pixels)) < 0) {
        ZEYES_LOG_ERROR("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return;
    }

    /* Now fill the values with a nice little plasma */
    //if (tracker) {
    int width = info.width;
    int height = info.height;
    int stride = info.stride;
    //tracker->render(&info, pixels);
    Tracker::getInstance()->render_panorama(pixels, width, height, stride);
    //}

    AndroidBitmap_unlockPixels(env, bitmap);
}
