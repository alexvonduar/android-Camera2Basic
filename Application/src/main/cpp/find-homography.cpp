//
// Created by alex on 18-4-10.
//

#include <jni.h>

#include <deque>
#include <vector>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <pthread.h>

#include <opencv2/opencv.hpp>

#include "debug_log.h"

#include "jpeglib.h"
#include "turbojpeg.h"

#include "neonorb.hpp"
#define ZEYES_LOG_LEVEL ZEYES_LOG_LEVEL_VERBOSE

typedef struct _Image {
    int buffer_size;
    unsigned char *p_jpg;
    cv::Mat data;
    cv::Mat desc;
    //std::vector<uint32_t> fast_desc;
    std::vector<cv::KeyPoint> keypoints;
    int orig_width;
    int orig_height;
    //FAST_NEON_ORB_640x480_8 pyramid;
} Image;

static inline void initImage(Image &img/*, const int &width, const int &height*/) {
    //if (img.data.cols != width || img.data.rows != height) {
    //    img.data.release();
    //}
    img.data = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);

    //img.data = cv::Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, img.pyramid.img);
    img.desc.release();
    //img.fast_desc.clear();
    img.keypoints.clear();
    img.buffer_size = 0;
    img.p_jpg = NULL;
}

static inline void deinitImage(Image &img) {
    img.data.release();
    img.desc.release();
    //img.fast_desc.clear();
    img.keypoints.clear();
    if (img.p_jpg != NULL) {
        delete[] img.p_jpg;
    }
    img.p_jpg = NULL;
    img.buffer_size = 0;
}

//static int row_size = 0;
static int width = 0;
static int height = 0;
static bool running = false;
static bool first = true;
static cv::Ptr<cv::ORB> orb;
static cv::Ptr<cv::BFMatcher> matcher;
static cv::Ptr<cv::FlannBasedMatcher> flann;
static pthread_t t_feature;
static pthread_t t_matching;
static pthread_t t_decoder;
static const int NUM_REF_RECT = 2;
//static const int RECT_SIZE = 9;
static int rectangle_idx = 0;
//static int rectangle[NUM_REF_RECT][RECT_SIZE];
static std::vector<std::vector<cv::Point2f> > intersections(NUM_REF_RECT);
static bool valid = false;

static inline int next_rect_idx(const int &idx) {
    return (idx + 1 < NUM_REF_RECT) ? idx + 1 : 0;
}

static inline void inc_rect_idx(int &idx) {
    idx = (idx + 1 < NUM_REF_RECT) ? idx + 1 : 0;
}

static const int NUM_EXTRA_BUFF = 4;

static std::vector<Image> buffers;

static std::deque<int> empty_list;
static pthread_mutex_t empty_mtx = PTHREAD_MUTEX_INITIALIZER;

static std::deque<int> feature_list;
static pthread_mutex_t feature_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t feature_cv = PTHREAD_COND_INITIALIZER;

static std::deque<int> matching_list;
static pthread_mutex_t matching_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t matching_cv = PTHREAD_COND_INITIALIZER;

static pthread_mutex_t reference_mtx = PTHREAD_MUTEX_INITIALIZER;

//static std::deque<int> decode_list;
static pthread_mutex_t decode_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t decode_cv = PTHREAD_COND_INITIALIZER;

void *feature(void *id) {
    ZEYES_LOG_DEBUG("DEADBEAF feature started\n");
    while (running) {
        {
            //std::unique_lock<std::mutex> lock(feature_mtx);
            pthread_mutex_lock(&feature_mtx);
            //feature_cv.wait(lock);
            pthread_cond_wait(&feature_cv, &feature_mtx);
            pthread_mutex_unlock(&feature_mtx);
        }
        ZEYES_LOG_DEBUG("DEADBEAF feature extraction wakeup\n");

        while (feature_list.size()) {
            int idx = -1;
            {
                pthread_mutex_lock(&feature_mtx);
                if (feature_list.size()) {
                    idx = feature_list[0];
                    feature_list.pop_front();
                }
                pthread_mutex_unlock(&feature_mtx);
            }

            if (idx != -1) {
                ZEYES_LOG_INFO("DEADBEAF start feature extraction\n");
                if (idx == 0) {
                    pthread_mutex_lock(&reference_mtx);
                }
                orb->detectAndCompute(buffers[idx].data, cv::Mat(), buffers[idx].keypoints,
                                      buffers[idx].desc);
                //fast_orb_640x480_downscale(buffers[idx].pyramid, 8);
                //fast_orb_640x480(buffers[idx].pyramid, 8, buffers[idx].keypoints, buffers[idx].fast_desc);
                ///*buffers[idx].desc = */ cv::Mat desc(buffers[idx].keypoints.size(), 32, CV_8UC1, buffers[idx].fast_desc.data());
                if(buffers[idx].desc.type()!=CV_32F) {
                    buffers[idx].desc.convertTo(buffers[idx].desc, CV_32F);
                }
                if (idx == 0) {
                    pthread_mutex_unlock(&reference_mtx);
                }
                ZEYES_LOG_INFO("DEADBEAF extracted %d features\n", buffers[idx].keypoints.size());

                if (idx == 0) { // reference images
                    ZEYES_LOG_INFO("DEADBEAF got one reference\n");
                    //cv::imwrite("/storage/emulated/0/Android/data/com.example.android.camera2basic/files/ref.jpg", buffers[idx].data);
                    //cv::imwrite("/storage/emulated/0/Android/data/com.example.android.camera2basic/files/data.jpg", buffers[3].data);
                    first = false;
                } else {
                    ZEYES_LOG_INFO("DEADBEAF info matching\n");
                    //std::unique_lock<std::mutex> lock(matching_mtx);
                    pthread_mutex_lock(&matching_mtx);
                    matching_list.push_back(idx);
                    //matching_cv.notify_one();
                    pthread_cond_signal(&matching_cv);
                    pthread_mutex_unlock(&matching_mtx);
                }
            }
        }
    }
    return id;
}

/*** sRANSAC ***/
static const double GLOBAL_TRUE_PROBABILITY = 0.225;
static const double LOCAL_TRUE_PROBABILITY = 0.2;
static const double OPENCV_DEFAULT_CONFIDENCE = 0.995;

cv::Mat getFeaturePairsBySequentialRanSAC(const std::vector<cv::KeyPoint> kp1,
                                          const std::vector<cv::KeyPoint> kp2,
                                          const std::vector<cv::DMatch> &matchings,
                                          std::vector<char> &final_mask) {
    const int HOMOGRAPHY_MODEL_MIN_POINTS = 4;
    const int GLOBAL_MAX_ITERATION = log(1 - OPENCV_DEFAULT_CONFIDENCE) / log(1 -
                                                                              pow(GLOBAL_TRUE_PROBABILITY,
                                                                                  HOMOGRAPHY_MODEL_MIN_POINTS));

    std::vector<cv::Point2f> X;
    std::vector<cv::Point2f> Y;

    for (int i = 0; i < matchings.size(); ++i) {
        X.push_back(
                kp1[matchings[i].queryIdx].pt); //Point2(matchings[i].first.x, matchings[i].first.y));
        Y.push_back(
                kp2[matchings[i].trainIdx].pt); //(Point2(matchings[i].second.x, matchings[i].second.y));
    }

    //std::vector<char> final_mask(matchings.size(), 0);
    cv::Mat h_result = cv::findHomography(X, Y, CV_RANSAC, 3, final_mask, GLOBAL_MAX_ITERATION);

#if 0
    std::vector<Point2> tmp_X = _X;
    std::vector<Point2> tmp_Y = _Y;

    std::vector<int> mask_indices(matchings.size(), 0);
    for (int i = 0; i < mask_indices.size(); ++i)
    {
        mask_indices[i] = i;
    }

    cv::Mat h_local = h_result;
    while (tmp_X.size() >= HOMOGRAPHY_MODEL_MIN_POINTS)
    {
        const int LOCAL_MAX_ITERATION = log(1 - OPENCV_DEFAULT_CONFIDENCE) / log(1 - pow(LOCAL_TRUE_PROBABILITY, HOMOGRAPHY_MODEL_MIN_POINTS));
        std::vector<Point2> next_X, next_Y;
        std::vector<char> mask(tmp_X.size(), 0);
        cv::Mat h_local = cv::findHomography(tmp_X, tmp_Y, CV_RANSAC, LOCAL_HOMOGRAPHY_MAX_INLIERS_DIST, mask, LOCAL_MAX_ITERATION);

        int inliers_count = 0;
        for (int i = 0; i < mask.size(); ++i)
        {
            if (mask[i])
            {
                ++inliers_count;
            }
        }
        if (inliers_count < LOCAL_HOMOGRAPHY_MIN_FEATURES_COUNT)
        {
            break;
        }
        for (int i = 0, shift = -1; i < mask.size(); ++i)
        {
            if (mask[i])
            {
                final_mask[mask_indices[i]] = 1;
            }
            else
            {
                next_X.emplace_back(tmp_X[i]);
                next_Y.emplace_back(tmp_Y[i]);
                mask_indices[++shift] = mask_indices[i];
            }
        }
#ifndef NDEBUG
        std::cout << "Local true Probabiltiy = " << next_X.size() / (float)tmp_X.size() << std::endl;
#endif
        tmp_X = next_X;
        tmp_Y = next_Y;
        h_result = h_local;
    }
#endif

    return h_result;
}

static bool save = true;

void *matching(void *id) {
    ZEYES_LOG_DEBUG("DEADBEAF matching started\n");
    while (running) {
        {
            //std::unique_lock<std::mutex> lock(matching_mtx);
            pthread_mutex_lock(&matching_mtx);
            //matching_cv.wait(lock);
            pthread_cond_wait(&matching_cv, &matching_mtx);
            pthread_mutex_unlock(&matching_mtx);
        }
        ZEYES_LOG_DEBUG("DEADBEAF matching wakeup\n");

        while (matching_list.size()) {
            int idx = -1;
            {
                //std::unique_lock<std::mutex> lock(matching_mtx);
                pthread_mutex_lock(&matching_mtx);
                //matching_cv.wait(lock);
                if (matching_list.size()) {
                    idx = matching_list[0];
                    matching_list.pop_front();
                }
                pthread_mutex_unlock(&matching_mtx);
            }
            if (idx != -1) {
                if (buffers[idx].keypoints.size() == 0) {
                    ZEYES_LOG_ERROR("DEADBEAF no key points in current image %d\n", idx);
                    continue;
                }
                std::vector<cv::DMatch> matchings;
                //cv::BFMatcher matcher(cv::NORM_HAMMING);
                cv::Mat H;

                //std::unique_lock<std::mutex> lock(reference_mtxs[row]);
                pthread_mutex_lock(&reference_mtx);
                //matcher->match(buffers[0].desc, buffers[idx].desc, matchings);
                flann->match(buffers[0].desc, buffers[idx].desc, matchings);
                //matcher.match(buffers[row].desc, buffers[idx].desc, matchings);
                //pthread_mutex_unlock(&reference_mtx);
                //}
#define DRAW_MATCHING
#if 0//!defined(DRAW_MATCHING)
                {
                    //std::unique_lock<std::mutex> lock(empty_mtx);
                    pthread_mutex_lock(&empty_mtx);
                    empty_list.push_back(idx);
                    ZEYES_LOG_DEBUG("DEADBEAF put %d image into empty size %d\n", idx,
                                    empty_list.size());
                    pthread_mutex_unlock(&empty_mtx);
                }
#endif

                std::vector<char> final_mask(matchings.size(), 0);
                H = getFeaturePairsBySequentialRanSAC(buffers[0].keypoints,
                                                      buffers[idx].keypoints, matchings,
                                                      final_mask);
                pthread_mutex_unlock(&reference_mtx);


#if defined(DRAW_MATCHING)

                if (save) {
                    cv::Mat img1;
                    cv::Mat img2;
                    cv::cvtColor(buffers[0].data, img1, CV_GRAY2BGR);
                    cv::cvtColor(buffers[idx].data, img2, CV_GRAY2BGR);
                    cv::Mat output;
                    cv::drawMatches(img1, buffers[0].keypoints, img2,
                                    buffers[idx].keypoints, matchings, output,
                                    cv::Scalar::all(-1), cv::Scalar::all(-1), final_mask);

                    cv::imwrite(
                            "/storage/sdcard0/DCIM/Camera/matching.jpg",
                            output);
                    save = false;
                }

#endif
#if 1
                {
                    //std::unique_lock<std::mutex> lock(empty_mtx);
                    pthread_mutex_lock(&empty_mtx);
                    empty_list.push_back(idx);
                    ZEYES_LOG_DEBUG("DEADBEAF put %d image into empty size %d\n", idx,
                                    empty_list.size());
                    pthread_mutex_unlock(&empty_mtx);
                }
#endif
                std::vector<cv::Point2f> corner_src(4);
                std::vector<cv::Point2f> corner_dst(4);
                corner_src[0] = cv::Point2f(0, 0);
                corner_src[1] = cv::Point2f(buffers[idx].orig_width, 0);
                corner_src[2] = cv::Point2f(buffers[idx].orig_width, buffers[idx].orig_height);
                corner_src[3] = cv::Point2f(0, buffers[idx].orig_height);
                cv::perspectiveTransform(corner_src, corner_dst, H);
                //
                ZEYES_LOG_INFO("DEADBEAF transformed corner [%f, %f] [%f, %f] [%f, %f] [%f, %f]\n",
                               corner_dst[0].x, corner_dst[0].y,
                               corner_dst[1].x, corner_dst[1].y,
                               corner_dst[2].x, corner_dst[2].y,
                               corner_dst[3].x, corner_dst[3].y);
                //std::vector<cv::Point2f> intersections;
                int i = next_rect_idx(rectangle_idx);
                intersections[i].clear();
                cv::intersectConvexConvex(corner_src, corner_dst, intersections[i]);
                ZEYES_LOG_INFO(
                        "DEADBEAF intersections corner %d [%f, %f] [%f, %f] [%f, %f] [%f, %f]\n",
                        intersections[i].size(),
                        intersections[i][0].x, intersections[i][0].y,
                        intersections[i][1].x, intersections[i][1].y,
                        intersections[i][2].x, intersections[i][2].y,
                        intersections[i][3].x, intersections[i][3].y);


                //rectangle[i][0] = (int)intersections[0].x;
                //rectangle[i][1] = (int)intersections[0].y;
                //rectangle[i][2] = (int)intersections[1].x;
                //rectangle[i][3] = (int)intersections[1].y;
                //rectangle[i][4] = (int)intersections[2].x;
                //rectangle[i][5] = (int)intersections[2].y;
                //rectangle[i][6] = (int)intersections[3].x;
                //rectangle[i][7] = (int)intersections[3].y;
                //rectangle[i][8] = 1;
                valid = true;
                inc_rect_idx(rectangle_idx);
            }
        }
    }
    return id;
}

unsigned char *
tjpeg_file2yuv(const unsigned char *const &jpeg_buffer, const int &jpeg_size, cv::Mat &y) {
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
    //tjDecompressHeader3(handle, jpeg_buffer, jpeg_size, &width, &height, &subsample, &colorspace);
    tjDecompressHeader3(handle, trans_buffer, trans_size, &width, &height, &subsample, &colorspace);

    ZEYES_LOG_DEBUG("w: %d h: %d subsample: %d color: %d\n", width, height, subsample, colorspace);

    //if (subsample != TJSAMP_420 || colorspace != TJCS_YCbCr) {
    //    ZEYES_LOG_ERROR("can't process this jpeg image");
    //    return NULL;
    //}

    flags |= 0;

    unsigned long yuv_size = tjBufSizeYUV2(width, padding, height, subsample);
    //unsigned long y_size = width * height;
    ZEYES_LOG_DEBUG("allocate %d bytes yuv buffer", yuv_size);
    unsigned char *yuv_buffer = new unsigned char[yuv_size]; //(unsigned char *)malloc(yuv_size);
    //unsigned char * yuv_buffer = new unsigned char[y_size]; //(unsigned char *)malloc(yuv_size);
    if (yuv_buffer == NULL) {
        ZEYES_LOG_ERROR("malloc buffer for rgb failed.\n");
        return NULL;
    }

    //ret = tjDecompressToYUV2(handle, jpeg_buffer, jpeg_size, yuv_buffer, width,
    //                         padding, height, flags);
    ret = tjDecompressToYUV2(handle, trans_buffer, trans_size, yuv_buffer, width,
                             padding, height, flags);
    //ret = tjDecompress2(handle, jpeg_buffer, jpeg_size, yuv_buffer, width, width, height, TJCS_GRAY, flags);
    //ret = tjDecompress2(handle, trans, trans_size, yuv_buffer, width, width, height, TJCS_GRAY, flags);
    tjFree(trans_buffer);
    if (ret < 0) {
        ZEYES_LOG_ERROR("decompress jpeg failed: %s\n", tjGetErrorStr());
        free(yuv_buffer);
        return NULL;
    }

    ZEYES_LOG_DEBUG("Decod finished");

    //const unsigned char * pbuffer = yuv_buffer;
    //double result = yuv_quality(pbuffer, width, height);
    if (width != y.cols || height != y.rows) {
        cv::Mat src = cv::Mat(height, width, CV_8UC1, yuv_buffer);
        cv::resize(src, y, y.size());
        ZEYES_LOG_DEBUG("DEADBEAF resize decoded jpeg from [%d, %d]->[%d, %d]\n", width, height,
                        y.cols, y.rows);
    } else {
        y.release();
        y = cv::Mat(height, width, CV_8UC1, yuv_buffer);
    }

    //free(yuv_buffer);
    tjDestroy(handle);

    return yuv_buffer;
}

void *decode(void *id) {
    ZEYES_LOG_DEBUG("DEADBEAF decode started\n");
    while (running) {
        {
            //std::unique_lock<std::mutex> lock(feature_mtx);
            pthread_mutex_lock(&decode_mtx);
            //feature_cv.wait(lock);
            pthread_cond_wait(&decode_cv, &decode_mtx);
            //pthread_mutex_unlock(&decode_mtx);
        }
        ZEYES_LOG_DEBUG("DEADBEAF jpeg decoder wakeup\n");

        if (running == false) {
            pthread_mutex_unlock(&decode_mtx);
            break;
        }

        /*while (decode_list.size())*/ {
            //ZEYES_LOG_DEBUG("DEADBEAF list size %d\n", decode_list.size());
            //int idx = -1;
            //{
            //    pthread_mutex_lock(&decode_mtx);
            //    if (decode_list.size()) {
            //        ZEYES_LOG_DEBUG("DEADBEAF list size %d\n", decode_list.size());
            //        idx = decode_list[0];
            //        decode_list.pop_front();
            //        ZEYES_LOG_DEBUG("DEADBEAF list size %d\n", decode_list.size());
            //    }
            //    pthread_mutex_unlock(&decode_mtx);
            //}

            int idx = 0;
            /*if (idx != -1)*/ {
                // TODO: decode jpeg here
                //buffers[idx].data.release();
                {
                    //pthread_mutex_lock(&decode_mtx);
                    ZEYES_LOG_DEBUG("DEADBEAF start decode %p\n", buffers[idx].p_jpg);
                    unsigned char *buffer = tjpeg_file2yuv(buffers[idx].p_jpg,
                                                           buffers[idx].buffer_size,
                                                           buffers[idx].data);
                    delete[] buffers[idx].p_jpg;
                    buffers[idx].p_jpg = buffer;
                    ZEYES_LOG_DEBUG("DEADBEAF decoded %p\n", buffers[idx].p_jpg);
                    buffers[idx].buffer_size = buffers[idx].data.cols * buffers[idx].data.rows;
                    pthread_mutex_unlock(&decode_mtx);
                }

                //if (idx < row_size) {
                ZEYES_LOG_DEBUG("DEADBEAF decode one reference\n");

                ZEYES_LOG_DEBUG("DEADBEAF info feature\n");
                //std::unique_lock<std::mutex> lock(matching_mtx);
                pthread_mutex_lock(&feature_mtx);
                feature_list.push_back(idx);
                //matching_cv.notify_one();
                pthread_cond_signal(&feature_cv);
                pthread_mutex_unlock(&feature_mtx);
                //}
            }
        }
    }
    return id;
}

static inline void start_theads() {
    ZEYES_LOG_DEBUG("DEADBEAF start threads---\n");
    running = true;
    first = true;
    //t_feature = std::thread(feature, 0);
    if (pthread_create(&t_feature, NULL, feature, NULL)) {
        ZEYES_LOG_ERROR("DEADBEAF create feature thread failed\n");
    } else {
        ZEYES_LOG_DEBUG("DEADBEAF feature thread created\n");
    }
    //t_matching = std::thread(matching, 0);
    if (pthread_create(&t_matching, NULL, matching, NULL)) {
        ZEYES_LOG_ERROR("DEADBEAF create matching thread failed\n");
    } else {
        ZEYES_LOG_DEBUG("DEADBEAF matching thread created\n");
    }
    if (pthread_create(&t_decoder, NULL, decode, NULL)) {
        ZEYES_LOG_ERROR("DEADBEAF create decode thread failed\n");
    } else {
        ZEYES_LOG_DEBUG("DEADBEAF decode thread created\n");
    }
}

static inline void stop_threads() {
    running = false;
    first = true;
    //feature_cv.notify_one();
    pthread_cond_signal(&feature_cv);
    //matching_cv.notify_one();
    pthread_cond_signal(&matching_cv);
    pthread_cond_signal(&decode_cv);
    //t_feature.join();
    void *r;
    pthread_join(t_feature, &r);
    //t_matching.join();
    pthread_join(t_matching, &r);
    pthread_join(t_decoder, &r);
}


static inline void deinit() {
    for (int i = 0; i < buffers.size(); ++i) {
        deinitImage(buffers[i]);
    }

    buffers.clear();
    empty_list.clear();
    feature_list.clear();
    matching_list.clear();
    //decode_list.clear();
    //reference_lists.clear();
    //if (!orb)
    //    orb->clear();
    //if (!matcher)
    //    matcher->clear();
}

static inline void init_start_rect(const int &width, const int &height) {
    rectangle_idx = 0;
    valid = false;
    //rectangle[0][0] = 0;
    //rectangle[0][1] = 0;
    //rectangle[0][2] = width;
    //rectangle[0][3] = 0;
    //rectangle[0][4] = width;
    //rectangle[0][5] = height;
    //rectangle[0][6] = 0;
    //rectangle[0][7] = height;
    //rectangle[0][8] = 0;
    //rectangle[1][0] = 0;
    //rectangle[1][1] = 0;
    //rectangle[1][2] = width;
    //rectangle[1][3] = 0;
    //rectangle[1][4] = width;
    //rectangle[1][5] = height;
    //rectangle[1][6] = 0;
    //rectangle[1][7] = height;
    //rectangle[1][8] = 0;
    intersections.resize(NUM_REF_RECT);
    for (int i = 0; i < NUM_REF_RECT; ++i) {
        intersections[i].resize(4);
        intersections[i][0] = cv::Point2f(0, 0);
        intersections[i][1] = cv::Point2f(width, 0);
        intersections[i][2] = cv::Point2f(width, height);
        intersections[i][3] = cv::Point2f(0, height);
    }
}

static inline void init(/*const int &width, const int &height*/) {
    deinit();
    int total = 1 + NUM_EXTRA_BUFF;

    buffers.resize(total);
    empty_list.resize(NUM_EXTRA_BUFF);

    for (int i = 0; i < total; ++i) {
        initImage(buffers[i]/*, width, height*/);
        if (i > 0) {
            empty_list[i - 1] = i;
        }
    }

    feature_list.clear();
    matching_list.clear();
    //decode_list.clear();
    orb = cv::ORB::create(1000);
    //matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
    flann = cv::FlannBasedMatcher::create();
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_FindHomography_start(
        JNIEnv *env,
        jclass /*clazz*/,
        jint width,
        jint height) {
    const int scale_factor = 2;
    ZEYES_LOG_INFO("DEADBEAF start [%d,%d]->[%d,%d]\n", width, height, 640, 480);
    init_start_rect(width, height);
    init(/*640, 480*/);
    ZEYES_LOG_INFO("DEADBEAF start threads\n");
    start_theads();
    ZEYES_LOG_INFO("DEADBEAF start threads end\n");
    return true;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_FindHomography_stop(
        JNIEnv *env,
        jclass /*clazz*/) {
    ZEYES_LOG_DEBUG("DEADBEAF stop\n");
    stop_threads();
    deinit();
    return true;
}

extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_example_android_camera2basic_FindHomography_getoverlaprect(
        JNIEnv *env,
        jclass /*clazz*/
) {
    jfloatArray result;
    int size = intersections[rectangle_idx].size() * 2 + 1;
    result = env->NewFloatArray(size);
    if (result == NULL) {
        return NULL; /* out of memory error thrown */
    }

    float *tmp = new float[size];
    for (int i = 0; i < intersections[rectangle_idx].size(); ++i) {
        tmp[i * 2] = intersections[rectangle_idx][i].x;
        tmp[i * 2 + 1] = intersections[rectangle_idx][i].y;
    }
    tmp[size - 1] = valid ? 1.0 : 0.0;
    // move from the temp structure to the java structure
    env->SetFloatArrayRegion(result, 0, size, tmp);
    delete[] tmp;
    return result;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_FindHomography_gethomography(
        JNIEnv *env,
        jclass /*clazz*/,
        jbyteArray data,
        jint width,
        jint height,
        jint stride) {

    if (running == false) {
        ZEYES_LOG_DEBUG("DEADBEAF not started\n");
        return true;
    }

    if (first) {
        ZEYES_LOG_DEBUG("DEADBEAF need at least one image\n");
        return true;
    }

    if (empty_list.size()) {
        int curr = -1;
        {
            //std::unique_lock<std::mutex> lock(empty_mtx);
            pthread_mutex_lock(&empty_mtx);
            if (empty_list.size()) {
                curr = empty_list[0];
                empty_list.pop_front();
            }
            ZEYES_LOG_DEBUG("DEADBEAF got empty image %d empty size %d\n", curr, empty_list.size());
            pthread_mutex_unlock(&empty_mtx);
        }

        if (curr != -1) {
            jbyte *buffer;
            jboolean isCopy;
            buffer = env->GetByteArrayElements(data, &isCopy);
            const unsigned char *pbuffer = (const unsigned char *) buffer;
            //pthread_mutex_lock(&reference_mtxs[row_id]);
            buffers[curr].orig_width = width;
            buffers[curr].orig_height = height;
            if (width != buffers[curr].data.cols || height != buffers[curr].data.rows) {
                ZEYES_LOG_DEBUG("DEADBEAF scale [%d,%d]->[%d,%d]", width, height,
                                buffers[curr].data.cols, buffers[curr].data.rows);
                cv::Mat orig(height, stride, CV_8UC1, buffer);
                cv::Rect rect(0, 0, width, height);
                cv::Mat src = orig(rect);
                cv::resize(src, buffers[curr].data, buffers[curr].data.size(), cv::INTER_CUBIC);
            } else {
                unsigned char *dst = buffers[curr].data.ptr<unsigned char>();
                for (int i = 0; i < height; ++i) {
                    memcpy(dst, pbuffer, width);
                    dst += width;
                    pbuffer += stride;
                }
            }
            //pthread_mutex_unlock(&reference_mtxs[row_id]);

            env->ReleaseByteArrayElements(data, buffer, 0);

            {
                //std::unique_lock<std::mutex> lock(feature_mtx);
                pthread_mutex_lock(&feature_mtx);
                feature_list.push_back(curr);
                //feature_cv.notify_one();
                pthread_cond_signal(&feature_cv);
                pthread_mutex_unlock(&feature_mtx);
            }
        }
    } else {
        ZEYES_LOG_INFO("DEADBEAF no enough buffer, skip\n");
        pthread_cond_signal(&feature_cv);
        pthread_cond_signal(&matching_cv);
        //pthread_cond_signal(&decode_cv);
    }
    return true;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_example_android_camera2basic_FindHomography_setreference(
        JNIEnv *env,
        jclass /*clazz*/,
        jbyteArray data,
        jint width,
        jint height,
        jint stride,
        jint jpeg_size) {

    ZEYES_LOG_INFO("DEADBEAF native %d %d", width, height);

    if (running == false) {
        init(/*640, 480*/);
        ZEYES_LOG_INFO("DEADBEAF start threads\n");
        start_theads();
        ZEYES_LOG_INFO("DEADBEAF start threads end\n");
    }
    std::vector<int> tmp;
    {
        //std::unique_lock<std::mutex> lock(matching_mtx);
        pthread_mutex_lock(&matching_mtx);
        tmp.assign(matching_list.begin(), matching_list.end());
        matching_list.clear();
        pthread_mutex_unlock(&matching_mtx);
    }

    ZEYES_LOG_DEBUG("DEADBEAF native 2 %d %d", width, height);
    {
        //std::unique_lock<std::mutex> lock(feature_mtx);
        pthread_mutex_lock(&feature_mtx);
        for (std::deque<int>::iterator it = feature_list.begin();
             it != feature_list.end();) {
            if (*it != 0) {
                tmp.push_back(*it);
            }
            it = feature_list.erase(it);
        }
        pthread_mutex_unlock(&feature_mtx);
    }


    ZEYES_LOG_DEBUG("DEADBEAF native 3 %d %d", width, height);
    jbyte *buffer;
    jboolean isCopy;
    buffer = env->GetByteArrayElements(data, &isCopy);
    const unsigned char *pbuffer = (const unsigned char *) buffer;

    {
        ZEYES_LOG_DEBUG("DEADBEAF %d\n", __LINE__);
        //std::unique_lock<std::mutex> lock(reference_mtxs[row_id]);
        pthread_mutex_lock(&reference_mtx);
        buffers[0].orig_width = width;
        buffers[0].orig_height = height;
        ZEYES_LOG_DEBUG("DEADBEAF %d\n", __LINE__);
        if (jpeg_size > 0) {
            pthread_mutex_lock(&decode_mtx);
            // copy and send to jpeg decode
            if (buffers[0].p_jpg) {
                delete[] buffers[0].p_jpg;
            }
            //buffers[row_id].data.release();
            buffers[0].p_jpg = new unsigned char[jpeg_size];
            buffers[0].buffer_size = jpeg_size;
            memcpy(buffers[0].p_jpg, pbuffer, jpeg_size);
            ZEYES_LOG_DEBUG("DEADBEAF input jpg %p\n", buffers[0].p_jpg);
            pthread_mutex_unlock(&decode_mtx);
        } else {
            if (width != buffers[0].data.cols || height != buffers[0].data.rows) {
                ZEYES_LOG_DEBUG("DEADBEAF resize [%d(%d), %d]->[%d, %d]\n", width, stride, height,
                                buffers[0].data.cols, buffers[0].data.rows);
                cv::Mat orig(height, stride, CV_8UC1, buffer);
                cv::Rect rect(0, 0, width, height);
                cv::Mat src = orig(rect);
                cv::resize(src, buffers[0].data, buffers[0].data.size(), 0, 0,
                           cv::INTER_LINEAR);
            } else {
                ZEYES_LOG_DEBUG("DEADBEAF copy reference buffer\n");
                unsigned char *dst = buffers[0].data.ptr<unsigned char>();
                for (int i = 0; i < height; ++i) {
                    memcpy(dst, pbuffer, width);
                    dst += width;
                    pbuffer += stride;
                }
            }
        }
        pthread_mutex_unlock(&reference_mtx);
    }
    env->ReleaseByteArrayElements(data, buffer, 0);

    if (jpeg_size) {
        // info decoder
        ZEYES_LOG_DEBUG("DEADBEAF inform decode list\n");
        {
            //std::unique_lock<std::mutex> lock(feature_mtx);
            pthread_mutex_lock(&decode_mtx);
            //decode_list.push_back(0);
            //feature_cv.notify_one();
            pthread_cond_signal(&decode_cv);
            pthread_mutex_unlock(&decode_mtx);
        }
    } else {
        ZEYES_LOG_DEBUG("DEADBEAF feature list\n");
        {
            //std::unique_lock<std::mutex> lock(feature_mtx);
            pthread_mutex_lock(&feature_mtx);
            feature_list.push_back(0);
            //feature_cv.notify_one();
            pthread_cond_signal(&feature_cv);
            pthread_mutex_unlock(&feature_mtx);
        }
    }

    ZEYES_LOG_DEBUG("DEADBEAF empty\n");
    {
        //std::unique_lock<std::mutex> lock(empty_mtx);
        pthread_mutex_lock(&empty_mtx);
        for (int i = 0; i < tmp.size(); ++i) {
            empty_list.push_back(tmp[i]);
        }
        pthread_mutex_unlock(&empty_mtx);
    }
    ZEYES_LOG_DEBUG("DEADBEAF setref return\n");

    return true;
}
