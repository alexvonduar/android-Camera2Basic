#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "ORBextractor.h"
#include "helper.hpp"

using namespace std;

#include "gms_matcher.h"

#include <bitplanes/core/bitplanes_tracker_pyramid.h>
#include <bitplanes/core/homography.h>
#include <bitplanes/core/viz.h>

#include <bitplanes/utils/timer.h>

typedef struct _Frame {
    cv::Mat original;
    cv::Mat rgb;
    cv::Mat gray;
    cv::Mat homo;
    cv::Mat relative_homo;
    cv::Mat pano_homo;
    std::vector<cv::Point2f> corners;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    int rotation;
    int index;
} Frame;

bool isFeasible(cv::Mat &homo, const bool &a3) {
    if (homo.empty()) {
        //v13 = 0;
        return false;
    } else {
        float sx = homo.at<float>(0, 0);
        float sy = homo.at<float>(1, 1);
        float cxy = homo.at<float>(0, 1);
        float min;
        float max;
        if (sx >= 0 && sy >= 0) {
            if (a3 == true) {
                if (sx >= sy) {
                    min = sy;
                } else {
                    min = sx;
                }
                if (min < 0.85) {
                    return false;
                }
                max = sx <= sy ? sy : sx;
                if (max > 1.15) {
                    return false;
                }
                if (cxy > 0.15 || cxy < -0.15) {
                    return false;
                }
            } else {
                if (sx >= sy) {
                    min = sy;
                } else {
                    min = sx;
                }
                if (min < 0.55) {
                    return false;
                }
                max = sx < sy ? sy : sx;
                if (max > 2.0) {
                    return false;
                }
                if (cxy > 0.5 || cxy < -0.5) {
                    return false;
                }
            }
            return true;
        } else {
            return false;
        }
    }
}

static inline void
create_corners4(const int &width, const int &height, std::vector<cv::Point2f> &corners) {
    corners.resize(4);
    corners[0].x = 0;
    corners[0].y = 0;
    corners[1].x = width;
    corners[1].y = 0;
    corners[2].x = width;
    corners[2].y = height;
    corners[3].x = 0;
    corners[3].y = height;
}

void perspective_transform(const int &width, const int &height, const cv::Mat &homo,
                           std::vector<cv::Point2f> &warped_corners) {
    std::vector<cv::Point2f> corners;
    create_corners4(width, height, corners);

    cv::perspectiveTransform(corners, warped_corners, homo);
}

int calculate_angle(const cv::Point2f &p0,
                    const cv::Point2f &p1,
                    const cv::Point2f &p2,
                    const cv::Point2f &p3) {

    float cross = (p1.y - p0.y) * (p3.y - p2.y) + (p1.x - p0.x) * (p3.x - p2.x);
    float dx10_square = (p1.x - p0.x) * (p1.x - p0.x);
    float dy10_square = (p1.y - p0.y) * (p1.y - p0.y);
    float dx32_square = (p3.x - p2.x) * (p3.x - p2.x);
    float dy32_square = (p3.y - p2.y) * (p3.y - p2.y);
    float len = sqrt((dx10_square + dy10_square) * (dx32_square + dy32_square));
    return (int) (acos(cross / len) * 180.0 / M_PI);
}

bool isLinesCrossed(const cv::Point2f &p0,
                    const cv::Point2f &p1,
                    const cv::Point2f &p2,
                    const cv::Point2f &p3) {

    bool v19 = false;
    float dx10 = p1.x - p0.x;
    float dx23 = p2.x - p3.x;
    float dy10 = p1.y - p0.y;
    float dy23 = p2.y - p3.y;
    float dx20 = p2.x - p0.x;
    float dy20 = p2.y - p0.y;
    float v12 = dx10 * dy23 - dx23 * dy10;
    double v9 = v12 > 0 ? (double) v12 : -(double) v12;
    //if (v9 >= (double)sub_54E46(0) * 0.00001 + 0.00000001)
    if (v9 >= 0.00001) {
        float v11 = (dx10 * dy20 - dx20 * dy10) / v12;
        float v10 = (dx20 * dy23 - dx23 * dy20) / v12;
        v19 = v10 >= 0.0 && v10 <= 1.0 && v11 >= 0.0 && v11 <= 1.0;
    } else {
        v19 = false;
    }
    return v19;
}

bool isStrangPolygon(const cv::Mat &h, const cv::Mat &curr_img, const cv::Mat &last_img) {
    bool result = false;

    //if (cv::Mat::empty(a2) || cv::Mat::empty((cv::Mat *)v56) || cv::Mat::empty((cv::Mat *)v55) == 1)
    if (h.empty() || curr_img.empty() || last_img.empty()) {
        ZEYES_LOG_ERROR("DEADBEAF homo empyt");
        result = true;
    } else {
        if (h.at<float>(0, 0) <= 0 || h.at<float>(1, 1) <= 0) {
            ZEYES_LOG_ERROR("DEADBEAF diagnal zero");
            return true;
        }

        cv::Rect rect(0, 0, 2, 2);
        cv::Mat h_s = h(rect);
        if (cv::determinant(h_s) <= 0) {
            //
            ZEYES_LOG_ERROR("DEADBEAF determinant error");
            return true;
        }

        int width = curr_img.cols;
        int height = curr_img.rows;
        std::vector<cv::Point2f> warped_corners(4);
        perspective_transform(width, height, h, warped_corners);
        int v52 = 0;
        int max_angle;
        int min_angle;
        for (int j = 0; j < 4; ++j) {
            int last;
            if (j) {
                last = j - 1;
            } else {
                last = 3;
            }
            int next;
            if (j == 3) {
                next = 0;
            } else {
                next = j + 1;
            }
            int angle = calculate_angle(warped_corners[j],
                                        warped_corners[last],
                                        warped_corners[j],
                                        warped_corners[next]);

            if (angle <= 10 && ++v52 >= 2) {

                ZEYES_LOG_ERROR("DEADBEAF homo large angle");
                return true;
            }
            if (j) {
                min_angle = min_angle < angle ? min_angle : angle;
                max_angle = max_angle > angle ? max_angle : angle;
            } else {
                min_angle = angle;
                max_angle = angle;
            }
        }
        if (max_angle - min_angle < 120) {
            if (isLinesCrossed(warped_corners[1],
                               warped_corners[2],
                               warped_corners[0],
                               warped_corners[3])) {
                ZEYES_LOG_ERROR("DEADBEAF homo cross line");
                result = true;
            } else {
                warped_corners[1].x += last_img.cols;
                warped_corners[2].x += last_img.cols;
                warped_corners[2].y += last_img.rows;
                warped_corners[3].y += last_img.rows;
                float min_x = warped_corners[0].x;
                float max_x = warped_corners[0].x;
                float min_y = warped_corners[0].y;
                float max_y = warped_corners[0].y;
                for (int k = 1; k < 4; ++k) {
                    float x = warped_corners[k].x;
                    min_x = min_x < x ? min_x : x;
                    max_x = max_x > x ? max_x : x;
                    float y = warped_corners[k].y;
                    min_y = min_y < y ? min_y : y;
                    max_y = max_y > y ? max_y : y;
                }
                int min_h = height < last_img.rows ? height : last_img.rows;
                int min_w = width < last_img.cols ? width : last_img.cols;
                result = ((max_y - min_y) > ((double) min_h * 3.8)) ||
                         ((max_x - min_x) > ((double) min_w * 3.5));
                if (result) {
                    ZEYES_LOG_ERROR("DEADBEAF too large");
                }
            }
        } else {
            ZEYES_LOG_ERROR("DEADBEAF homo too large angle");
            result = true;
        }
    }
    return result;
}

void tracking_frames_optflow(Frame *last, Frame *curr) {
    //
    std::vector<cv::Point2f> corners_next;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(last->gray, curr->gray, last->corners, corners_next, status, err,
                             cv::Size(10, 10));

    int num_status = status.size();
    std::vector<cv::Point2f> good_features_last;
    std::vector<cv::Point2f> good_features_next;

    for (int i = 0; i < num_status; ++i) {
        if (status[i]) {
            good_features_last.push_back(last->corners[i]);
            good_features_next.push_back(corners_next[i]);
        }
    }

    if (good_features_last.empty()) {
        curr->homo.release();
        //return false;
        return;
    }

    cv::Mat H = cv::findHomography(good_features_next, good_features_last, CV_LMEDS, 3.0,
                                   cv::noArray(), 2000, 0.9999);
    H.convertTo(H, CV_32F);
    if (isFeasible(H, true)) {
        curr->relative_homo = H;
        curr->homo = last->homo * H;
    } else {
        curr->relative_homo.release();
        curr->homo.release();
    }
}

static bool save_matching = true;

void pano_frames_orb(ORB_SLAM2::ORBextractor *orb_extractor, cv::Ptr<cv::BFMatcher> &matcher,
                     Frame *last, Frame *curr, cv::Mat &homo) {
    //
#if 0
    cv::Mat mask;
    int * point_pairs;
    int num_point_pairs;
    cv::Mat homo = IMS::FMY::CvSift::getInstance()->project(curr->gray, last->gray, false, mask, &point_pairs, &num_point_pairs);
    if (save_matching) {
        cv::Mat matchings;
        IMS::FMY::CvSift::getInstance()->drawMatches(curr->gray, last->gray, mask, point_pairs, num_point_pairs, matchings);
        cv::imwrite(
                "/storage/emulated/0/Android/data/com.example.android.camera2basic/files/matching.jpg",
                matchings);
        save_matching = false;
        std::vector<cv::Point2f> corners;
        create_corners4(curr->gray.cols, curr->gray.rows, corners);
        std::vector<cv::Point2f> warped_corners;
        cv::perspectiveTransform(corners, warped_corners, homo.inv());
        corners.insert(corners.end(), warped_corners.begin(), warped_corners.end());
        cv::Rect rect = cv::boundingRect(corners);
        cv::Mat shift = cv::Mat::eye(3, 3, CV_32F);
        shift.at<float>(0, 2) = -rect.x;
        shift.at<float>(1, 2) = -rect.y;
        cv::Mat tmp = cv::Mat::zeros(rect.height, rect.width, CV_8U);
        cv::Size s(rect.width, rect.height);
        cv::warpPerspective(curr->gray, tmp, shift, s, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
        shift = shift * homo.inv();
        cv::warpPerspective(last->gray, tmp, shift, s, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
        cv::imwrite(
                "/storage/emulated/0/Android/data/com.example.android.camera2basic/files/warp.jpg",
                tmp);
    }
    if (homo.empty()) {
        curr->pano_homo.release();
    } else {
        homo.convertTo(curr->pano_homo, CV_32F);
    }
#else
    //cv::Mat gray;
    if (last->keypoints.size() == 0 || last->descriptors.empty()) {
        //cv::cvtColor(last->original, gray, CV_RGB2GRAY);
        (*orb_extractor)(last->gray, cv::noArray(), last->keypoints, last->descriptors);
    }

    if (curr->keypoints.size() == 0 || curr->descriptors.empty()) {
        //cv::cvtColor(curr->original, gray, CV_RGB2GRAY);
        (*orb_extractor)(curr->gray, cv::noArray(), curr->keypoints, curr->descriptors);
    }

    std::vector<std::vector<cv::DMatch>> vv_dmatch;
    matcher->knnMatch(last->descriptors, curr->descriptors, vv_dmatch, 2, cv::noArray());
    std::vector<cv::DMatch> v_dmatch;

    for (int j = 0; j < vv_dmatch.size(); ++j) {
        if (vv_dmatch[j].size() == 1) {
            v_dmatch.push_back(vv_dmatch[j][0]);
        } else if (vv_dmatch[j].size() > 1) {
            if (vv_dmatch[j][0].distance <= (vv_dmatch[j][1].distance * 0.75)) {
                v_dmatch.push_back(vv_dmatch[j][0]);
            }
        }
    }

    if (v_dmatch.size() < 4) {
        homo.release();
    } else {
        std::vector<cv::KeyPoint> valid_last_kp;
        std::vector<cv::KeyPoint> valid_curr_kp;
        cv::Mat mask;

#if USE_GMS
        // GMS filter
        int num_inliers = 0;
        std::vector<bool> vbInliers;
        std::vector<cv::DMatch> gms_match;
        gms_matcher gms(last->keypoints, last->gray.size(), curr->keypoints, curr->gray.size(),
                        v_dmatch);
        num_inliers = gms.GetInlierMask(vbInliers, false, false);

        for (size_t k = 0; k < vbInliers.size(); ++k) {
            if (vbInliers[k] == true) {
                gms_match.push_back(v_dmatch[k]);
            }
        }

        if (num_inliers < 4) {
            curr->pano_homo.release();
        } else {
            for (int k = 0; k < gms_match.size(); ++k) {
                valid_last_kp.push_back(last->keypoints[gms_match[k].queryIdx]);
                valid_curr_kp.push_back(curr->keypoints[gms_match[k].trainIdx]);
            }
#else
        {
            for (int k = 0; k < v_dmatch.size(); ++k) {
                valid_last_kp.push_back(last->keypoints[v_dmatch[k].queryIdx]);
                valid_curr_kp.push_back(curr->keypoints[v_dmatch[k].trainIdx]);
            }
#endif
            std::vector<cv::Point2f> valid_last_points;
            cv::KeyPoint::convert(valid_last_kp, valid_last_points);
            std::vector<cv::Point2f> valid_curr_points;
            cv::KeyPoint::convert(valid_curr_kp, valid_curr_points);
            cv::Mat homo_tmp = cv::findHomography(valid_curr_points, valid_last_points, CV_RANSAC,
                                                  2.0,
                                                  mask, 500, 0.999);
            assert(mask.rows <= valid_curr_points.size());
            assert(valid_last_points.size() == valid_curr_points.size());
            if (!homo_tmp.empty()) {
                homo_tmp.convertTo(homo, CV_32F);
            } else {
                homo.release();
            }
        }

        if (save_matching) {
            cv::Mat matchings;
#if USE_GMS
            cv::drawMatches(last->rgb, last->keypoints, curr->rgb, curr->keypoints,
                            gms_match, matchings);
#else
            std::vector<cv::DMatch> homo_match;
            for (int k = 0; k < mask.rows; ++k) {
                if (mask.at<uchar>(k, 0)) {
                    homo_match.push_back(v_dmatch[k]);
                }
            }
            cv::drawMatches(last->rgb, last->keypoints, curr->rgb, curr->keypoints,
                            homo_match, matchings);
#endif
            cv::imwrite(
                    "/storage/emulated/0/Android/data/com.example.android.camera2basic/files/matching.jpg",
                    matchings);
            //save_matching = false;
        }

    }
#endif
}

typedef struct _ECC_PARAMES
{
    Frame * last;
    Frame * curr;
    cv::Mat * homo;
} ECC_PARAMS;

void *ecc_thread_func(void * _p) {
    ZEYES_LOG_ERROR("DEADBEAF ecc++++");
    double * r = new double;
    ECC_PARAMS * p =  (ECC_PARAMS *)_p;
    *r = cv::findTransformECC(p->curr->gray, p->last->gray,
                                    *(p->homo), cv::MOTION_HOMOGRAPHY, cv::TermCriteria(
                    cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001));
    ZEYES_LOG_ERROR("DEADBAF ecc---- %f", *r);
    return (void *)r;
}

typedef struct _ORB_PARAMS
{
    ORB_SLAM2::ORBextractor * orb_extractor;
    cv::Ptr<cv::BFMatcher> matcher;
    Frame * last;
    Frame * curr;
    cv::Mat * homo;
} ORB_PARAMS;

void * orb_thread_func(void * _p)
{
    ZEYES_LOG_ERROR("DEADBEAF orb++++");
    ORB_PARAMS * p = (ORB_PARAMS *)_p;
    pano_frames_orb(p->orb_extractor, p->matcher,
                         p->last, p->curr, *(p->homo));
    ZEYES_LOG_ERROR("DEADBEAF orb----");
    return NULL;
}

static inline bool is_aspect_ratio(const int &a, const int &b, const int &asp_l, const int &asp_s) {
    if (a > b) {
        return (a * asp_s) == (b * asp_l);
    } else if (a < b) {
        return (a * asp_l) == (b * asp_s);
    } else {
        return asp_l == asp_s;
    }
}

static const int SCALED_WIDTH = 320;
static const int SCALED_HEIGHT_4_3 = 240;
static const int SCALED_HEIGHT_16_9 = 180;
static const int PANO_SCALE_FACTOR = 2;
static const int PANO_MAX_WIDTH = 640;
static const int PANO_MAX_HEIGHT_4_3 = 480;
static const int PANO_MAX_HEIGHT_16_9 = 360;

static bool save_rotate = false;

static inline void
init_frame(Frame *frame, const cv::Mat &image, const int &rotation, const bool &is_keyframe) {

    frame->rotation = rotation;

    bool portrait = image.cols < image.rows;
    int max_side = portrait ? image.rows : image.cols;
    int short_side = portrait ? image.cols : image.rows;
    cv::Size scaled_size;
    cv::Size pano_size;
    bool need_resize = max_side > SCALED_WIDTH;
    bool need_resize_pano = max_side > PANO_MAX_WIDTH;
    if (is_aspect_ratio(image.cols, image.rows, 16, 9)) {
        if (need_resize) {
            scaled_size.width = portrait ? SCALED_HEIGHT_16_9 : SCALED_WIDTH;
            scaled_size.height = portrait ? SCALED_WIDTH : SCALED_HEIGHT_16_9;
        }
        if (need_resize_pano) {
            pano_size.width = portrait ? PANO_MAX_HEIGHT_16_9 : PANO_MAX_WIDTH;
            pano_size.height = portrait ? PANO_MAX_WIDTH : PANO_MAX_HEIGHT_16_9;
        }
    } else if (is_aspect_ratio(image.cols, image.rows, 4, 3)) {
        if (need_resize) {
            scaled_size.width = portrait ? SCALED_HEIGHT_4_3 : SCALED_WIDTH;
            scaled_size.height = portrait ? SCALED_WIDTH : SCALED_HEIGHT_4_3;
        }
        if (need_resize_pano) {
            pano_size.width = portrait ? PANO_MAX_HEIGHT_4_3 : PANO_MAX_WIDTH;
            pano_size.height = portrait ? PANO_MAX_WIDTH : PANO_MAX_HEIGHT_4_3;
        }
    } else {
        short_side = SCALED_WIDTH * short_side / max_side;
        int pano_short = short_side * PANO_SCALE_FACTOR;
        if (need_resize) {
            scaled_size.width = portrait ? SCALED_HEIGHT_4_3 : SCALED_WIDTH;
            scaled_size.height = portrait ? SCALED_WIDTH : SCALED_HEIGHT_4_3;
        }
        if (need_resize_pano) {
            pano_size.width = portrait ? pano_short : PANO_MAX_WIDTH;
            pano_size.height = portrait ? PANO_MAX_WIDTH : pano_short;
        }
    }

    //if (rotation == 0) {
    if (need_resize) {
        cv::resize(image, frame->rgb, scaled_size);
    } else {
        frame->rgb = image;
    }

    if (0 && is_keyframe) {
        if (need_resize_pano) {
            cv::resize(image, frame->original, pano_size);
        } else {
            frame->original = image;
        }
    }

    //} else if (rotation == 90) {
    //    cv::Mat tmp;
    //    cv::resize(image, tmp, scaled_size);
    //    cv::rotate(tmp, frame->rgb, cv::ROTATE_90_CLOCKWISE);
    //}
    cv::cvtColor(frame->rgb, frame->gray, CV_RGB2GRAY);
    if (1 || is_keyframe) {
        int maxCorners = 1000;
        double qualityLevel = 0.05;
        double minDistance = 3.;
        cv::Mat mask;
        int blockSize = 3;
        bool useHarrisDetector = false;
        double k = 0.04;
        cv::goodFeaturesToTrack(frame->gray, frame->corners, maxCorners, qualityLevel, minDistance,
                                mask, blockSize, useHarrisDetector, k);
        frame->homo = cv::Mat::eye(3, 3, CV_32F);
        frame->relative_homo = cv::Mat::eye(3, 3, CV_32F);
        frame->pano_homo = cv::Mat::eye(3, 3, CV_32F);
    }
    if (save_rotate) {
        cv::imwrite(
                "/storage/emulated/0/Android/data/com.example.android.camera2basic/files/rotate_rgb.jpg",
                frame->rgb);
        save_rotate = false;
    }
}

static const int MAX_TRACKING_FRAMES = 10;
static pthread_mutex_t hintbox_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t global_lock = PTHREAD_MUTEX_INITIALIZER;
static cv::Mat ROTATE_90 = (cv::Mat_<float>(3, 3) << 0., -1., 0., 1., 0., 0., 0., 0., 1.);

static inline
void recalcHomography(cv::Mat &homo, const int &w, const int &h, const float &scale) {
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point2f> warped_corners;
    create_corners4(w, h, corners);
    cv::perspectiveTransform(corners, warped_corners, homo);
    for (int i = 0; i < 4; ++i) {
        corners[i].x *= scale;
        corners[i].y *= scale;
        warped_corners[i].x *= scale;
        warped_corners[i].y *= scale;
    }
    homo = cv::findHomography(corners, warped_corners, CV_RANSAC, 3.0, cv::noArray(), 2000,
                              0.999);
    homo.convertTo(homo, CV_32F);
}

static bool save_pano = false;

static inline bp::AlgorithmParameters GetDefaultParams() {
    bp::AlgorithmParameters params;
    params.num_levels = 2;
    params.max_iterations = 100;
    params.parameter_tolerance = 5e-5;
    params.function_tolerance = 1e-4;
    params.verbose = false;
    params.sigma = 2.0;
    params.subsampling = 1;
    return params;
}

class Tracker {
    ORB_SLAM2::ORBextractor *orb_extractor; //(1000, 1.2, 8, 20, 7);
    cv::Ptr<cv::BFMatcher> matcher;
    std::vector<Frame *> all_frames;
    std::vector<cv::Point2f> m_hintboxf;
    std::vector<cv::Point2i> m_hintboxi;
    //cv::Mat pano_image;
    bool m_valid_hintbox;
    bool m_valid_overlap;
    //int m_pano_center;
    //cv::Rect m_pano_rect;
    //cv::Mat m_pano_global_homo;
    int m_num_pano_frames;
    bp::BitPlanesTrackerPyramid<bp::Homography> m_bp_tracker; //(GetDefaultParams());
    cv::Rect m_bp_box;
    bp::Result m_bp_result;//(bp::Matrix33f::Identity());

    void get_bounding_box(const int &index, cv::Rect &rect) {
        cv::Mat inv_homo = all_frames[keyframe_list[index]]->pano_homo.inv();
        ZEYES_LOG_ERROR("DEADBEAF bounding %d", index);
        int width = all_frames[keyframe_list[index]]->rgb.cols;
        int height = all_frames[keyframe_list[index]]->rgb.rows;
        std::vector<cv::Point2f> corners(4);
        create_corners4(width, height, corners);
        std::vector<cv::Point2f> warped_corners = corners;
        for (int i = 0; i < keyframe_list.size(); ++i) {
            if (i != index) {
                int frame_index = keyframe_list[i];
                cv::Mat homo = inv_homo * all_frames[frame_index]->pano_homo;
                //homo = inv_homo * homo;
                std::vector<cv::Point2f> warped_corners_tmp;
                cv::perspectiveTransform(corners, warped_corners_tmp, homo);
                warped_corners.insert(warped_corners.end(), warped_corners_tmp.begin(),
                                      warped_corners_tmp.end());
            }
        }
        rect = cv::boundingRect(warped_corners);
    }

    int find_best_pano(cv::Rect &best_box, cv::Mat &homo) {
        int index = 0;
        //cv::Rect2f bbox;
        get_bounding_box(index, best_box);

#if 1
        for (int i = 1; i < keyframe_list.size(); ++i) {
            cv::Rect box;
            get_bounding_box(i, box);
            if (best_box.area() > box.area()) {
                best_box = box;
                index = i;
            }
        }
#endif

        homo = all_frames[keyframe_list[index]]->pano_homo.inv();
        return index;
    }

#if 0
    void update_panorama() {
        if (keyframe_list.size()) {
            if (m_num_pano_frames == keyframe_list.size()) {
                return;
            }
            //
            if (keyframe_list.size() == 1) {
                pano_image = all_frames[keyframe_list[0]]->rgb;
                m_pano_center = 0;
            } else {
                m_pano_center = find_best_pano(m_pano_rect);
                int offset_x = m_pano_rect.x;
                int offset_y = m_pano_rect.y;
                int width = m_pano_rect.width;
                int height = m_pano_rect.height;
                cv::Mat mshift = cv::Mat::eye(3, 3, CV_32F);
                mshift.at<float>(0, 2) = -offset_x;
                mshift.at<float>(1, 2) = -offset_y;
                pano_image = cv::Mat::zeros(height, width, CV_8UC3);
                mshift = mshift * m_pano_global_homo;
                cv::Size dst_size(width, height);
                //std::vector<cv::Point2f> corners;
                //create_corners4(all_frames[m_pano_center]->rgb.cols,
                //                all_frames[m_pano_center]->rgb.rows, corners);
                for (int i = 0; i < keyframe_list.size(); ++i) {
                    cv::Mat homo = mshift * all_frames[keyframe_list[i]]->pano_homo;
                    cv::warpPerspective(all_frames[keyframe_list[i]]->rgb, pano_image, homo,
                                        dst_size, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
                    //std::vector<cv::Point2f> warped_corners;
                    //cv::perspectiveTransform(corners, warped_corners, homo);
                }
            }
            //if (save_pano) {
            //    cv::imwrite(
            //            "/storage/emulated/0/Android/data/com.eyes.zero/files/pano_rgb.jpg",
            //            pano_image);
            //    //save_rotate = false;
            //}
            m_num_pano_frames = keyframe_list.size();
        } else {
            pano_image.release();
        }
    }
#endif

    Frame *get_one_frame(const int &index) {
        if (index >= 0 && all_frames.size() > index) {
            return all_frames[index];
        } else {
            return NULL;
        }
    }

    int set_one_frame(Frame *frame) {
        if (frame->index == -1) {
            int frame_idx = all_frames.size();
            frame->index = frame_idx;
            all_frames.push_back(frame);
        }
        return frame->index;
    }

    std::vector<int> free_list;

    Frame *get_one_free() {
        Frame *new_frame = NULL;
        if (tracking_list.size() >= MAX_TRACKING_FRAMES) {
            squeeze_tracking_list();
        }
        if (free_list.size()) {
            int idx = free_list.back();
            free_list.pop_back();
            new_frame = all_frames[idx];
        } else {
            new_frame = new Frame;
            new_frame->index = -1;
        }
        return new_frame;
    }

    int store_one_free(Frame *frame) {
        free_list.push_back(set_one_frame(frame));
        return frame->index;
    }

    std::vector<int> tracking_list;

    int store_one_tracking(Frame *frame) {
        tracking_list.push_back(set_one_frame(frame));
        frame->rgb.release();
        ZEYES_LOG_ERROR("DEADBEAF add tracking: tracking %d free %d", tracking_list.size(),
                        free_list.size());
        return frame->index;
    }

    //int get_one_tracking()
    //{
    //    int index = -1;
    //    if (tracking_list.size()) {
    //        index = tracking_list[0];
    //        tracking_list.pop();
    //    }
    //    return index;
    //}
    void squeeze_tracking_list() {
        //int last_index = tracking_list.back();
        //for (int i = 0; i < (tracking_list.size() - 1); ++i) {
        //    free_list.push_back(tracking_list[i]);
        //}
        //tracking_list.clear();
        //tracking_list.push_back(last_index);
        free_list.insert(free_list.end(), tracking_list.begin(), tracking_list.end() - 1);
        tracking_list.erase(tracking_list.begin(), tracking_list.end() - 1);
        ZEYES_LOG_ERROR("DEADBEAF squeeze tracking: tracking %d free %d", tracking_list.size(),
                        free_list.size());
    }

    void clear_tracking_list() {
        free_list.insert(free_list.end(), tracking_list.begin(), tracking_list.end());
        tracking_list.clear();
        ZEYES_LOG_ERROR("DEADBEAF clear tracking: tracking %d free %d", tracking_list.size(),
                        free_list.size());
    }

    std::vector<int> keyframe_list;

    int store_keyframe(Frame *frame) {
        //if (keyframe_list.size()) {
        //    int index = keyframe_list.back();
        //    all_frames[index]->original.release();
        //}
        keyframe_list.push_back(set_one_frame(frame));
        return frame->index;
    }

    bool map(Frame *frame) {
        bool result = false;
        Frame *last_frame = NULL;
        if (tracking_list.size()) {
            // map with tracking list last frame
            last_frame = all_frames[tracking_list.back()];
        } else {
            // map with last key frame
            last_frame = all_frames[keyframe_list.back()];
        }

        tracking_frames_optflow(last_frame, frame);

        if (frame->homo.empty()) {
            return false;
        } else {
            return true;
        }
    }

    bool update_hintbox() {
        pthread_mutex_lock(&hintbox_mutex);
        if (tracking_list.size()) {
            //
            int index = tracking_list.back();
            Frame *p_frame = all_frames[index];
            std::vector<cv::Point2f> corners;
            int width = p_frame->gray.cols;
            int height = p_frame->gray.rows;
            create_corners4(width, height, corners);
            std::vector<cv::Point2f> warped_corners;
            cv::Mat homo = p_frame->homo.inv();
            cv::Mat rotate = ROTATE_90;
            if (p_frame->rotation == 90) {
                //rotate = ROTATE_90;
                rotate.at<float>(0, 2) = height;
                //float a = rotate.at<float>(0, 0);
                //a = rotate.at<float>(0, 1);
                //a = rotate.at<float>(0, 2);
                //a = rotate.at<float>(1, 0);
                //a = rotate.at<float>(1, 1);
                //a = rotate.at<float>(1, 2);
                //a = rotate.at<float>(2, 0);
                //a = rotate.at<float>(2, 1);
                //a = rotate.at<float>(2, 2);
                homo = rotate * homo;
                //int tmp = width;
                //width = height;
                //height = width;
            } else if (p_frame->rotation == 0) {
                //
            } else {
                assert(0);
            }
            //perspective_transform(p_frame->gray.cols, p_frame->gray.rows, homo, warped_corners);
            cv::perspectiveTransform(corners, warped_corners, homo);
            if (p_frame->rotation == 90) {
                cv::perspectiveTransform(corners, corners, rotate);
                int temp = width;
                width = height;
                height = temp;
            }
            float overlap = cv::intersectConvexConvex(warped_corners, corners, m_hintboxf);
            for (int i = 0; i < m_hintboxf.size(); ++i) {
                m_hintboxf[i].x /= width;
                m_hintboxf[i].x =
                        m_hintboxf[i].x < 0 ? 0.0 : (m_hintboxf[i].x > 1.0 ? 1.0 : m_hintboxf[i].x);
                m_hintboxf[i].y /= height;
                m_hintboxf[i].y =
                        m_hintboxf[i].y < 0 ? 0.0 : (m_hintboxf[i].y > 1.0 ? 1.0 : m_hintboxf[i].y);
            }
            if (overlap > width * height * 0.4) {
                m_valid_overlap = true;
                m_valid_hintbox = true;
            } else {
                m_valid_overlap = false;
                m_valid_hintbox = true;
            }
        } else {
            m_hintboxi.clear();
            m_hintboxf.clear();
            m_valid_overlap = false;
            m_valid_hintbox = false;
        }
        pthread_mutex_unlock(&hintbox_mutex);
        return m_valid_overlap;
    }

    bool update_hintbox(cv::Rect &rect, const float *H) {
        pthread_mutex_lock(&hintbox_mutex);
        if (tracking_list.size()) {
            //
            int index = tracking_list.back();
            Frame *p_frame = all_frames[index];
            //std::vector<cv::Point2f> corners;
            int width = p_frame->gray.cols;
            int height = p_frame->gray.rows;
            //create_corners4(width, height, corners);
            //std::vector<cv::Point2f> warped_corners;

            std::vector<cv::Point2f> corners;
            RectToPoints(rect, H, corners);
            //std::array<cv::Point2f, 4> warped_corners;
            //cv::Mat homo = p_frame->homo.inv();
            cv::Mat rotate = ROTATE_90;
            if (p_frame->rotation == 90) {
                rotate.at<float>(0, 2) = height;
                //homo = rotate * homo;
            } else if (p_frame->rotation == 0) {
                //
            } else {
                assert(0);
            }
            //perspective_transform(p_frame->gray.cols, p_frame->gray.rows, homo, warped_corners);
            cv::perspectiveTransform(corners, m_hintboxf, rotate);
            if (p_frame->rotation == 90) {
                //    cv::perspectiveTransform(corners, corners, rotate);
                int temp = width;
                width = height;
                height = temp;
            }
            //float overlap = cv::intersectConvexConvex(warped_corners, corners, m_hintboxf);
            for (int i = 0; i < m_hintboxf.size(); ++i) {
                m_hintboxf[i].x /= width;
                m_hintboxf[i].y /= height;
            }
            m_valid_overlap = true;
            m_valid_hintbox = true;
        } else {
            m_hintboxi.clear();
            m_hintboxf.clear();
            m_valid_overlap = false;
            m_valid_hintbox = false;
        }
        pthread_mutex_unlock(&hintbox_mutex);
        return m_valid_overlap;
    }

    Tracker() {
        orb_extractor = new ORB_SLAM2::ORBextractor(2000, 1.2, 4, 30, 1);
        matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
        m_bp_tracker = bp::BitPlanesTrackerPyramid<bp::Homography>(GetDefaultParams());
        clear();
    }

    static Tracker *instance;

    void clear() {

        m_hintboxf.clear();
        m_hintboxi.clear();
        //pano_image.release();
        m_valid_hintbox = false;
        m_valid_overlap = false;
        //m_pano_center = 0;
        //m_pano_rect;
        //m_pano_global_homo = cv::Mat::eye(3, 3, CV_32F);
        m_num_pano_frames = 0;
        free_list.insert(free_list.end(), keyframe_list.begin(), keyframe_list.end());
        free_list.insert(free_list.end(), tracking_list.begin(), tracking_list.end());
        keyframe_list.clear();
        tracking_list.clear();

        ZEYES_LOG_ERROR(
                "DEADBEAF ---- clear num keys: %d num tracking: %d num free: %d all: %d",
                keyframe_list.size(), tracking_list.size(),
                free_list.size(), all_frames.size());

    }

    void match_keyframe(Frame *last, Frame *curr, cv::Mat &homo) {
        pthread_t ecc_t;
        pthread_t orb_t;
        ECC_PARAMS ecc_p;
        ORB_PARAMS orb_p;
        cv::Mat ecc_homo;
        cv::Mat orb_homo;
        ecc_p.curr = curr;
        ecc_p.last = last;
        ecc_p.homo = &ecc_homo;
        orb_p.orb_extractor = orb_extractor;
        orb_p.matcher = matcher;
        orb_p.curr = curr;
        orb_p.last = last;
        orb_p.homo = &orb_homo;

#if 0
#if 1
        double r = cv::findTransformECC(curr->gray, last->gray,
                                        homo, cv::MOTION_HOMOGRAPHY, cv::TermCriteria(
                        cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001));
        if (r < 0.86) {
#endif
            pano_frames_orb(orb_extractor, matcher, last, curr, homo);
#if 1
        }
#endif
#endif
        ZEYES_LOG_ERROR("DEADBEAF start matching ++++");
        pthread_create(&ecc_t, NULL, ecc_thread_func, &ecc_p);
        pthread_create(&orb_t, NULL, orb_thread_func, &orb_p);
        void * ecc_ret;
        pthread_join(ecc_t, &ecc_ret);
        void * orb_ret;
        pthread_join(orb_t, &orb_ret);
        double ecc = (*(double *)ecc_ret);
        delete ecc_ret;
        ZEYES_LOG_ERROR("DEADBEAF stop matching ++++ ecc %f", ecc);
        if (ecc < 0.86) {
            homo = orb_homo;
        } else {
            homo = ecc_homo;
        }
    }

public:
    static Tracker *getInstance() {
        if (!instance) {
            instance = new Tracker();
        }
        return instance;
    }

    ~Tracker() {
        stop();
        if (orb_extractor) {
            delete orb_extractor;
        }
        orb_extractor = NULL;
        //if (matcher) {
        //    delete matcher;
        //}
        //matcher = NULL;
        matcher.release();
    }

    void start() {
        //
    }


    void stop() {
        ZEYES_LOG_ERROR("DEADBEAF stop++--");
        pthread_mutex_lock(&global_lock);
        ZEYES_LOG_ERROR("DEADBEAF stop++--");
        clear();

        free_list.clear();

        for (int i = 0; i < all_frames.size(); ++i) {
            delete all_frames[i];
        }
        all_frames.clear();
        pthread_mutex_unlock(&global_lock);
        ZEYES_LOG_ERROR("DEADBEAF stop----");
    }

    void get_hintbox(std::vector<cv::Point2f> &hintbox, bool &valid_overlap, bool &valid_hintbox) {
        ZEYES_LOG_ERROR("DEADBEAF hint box++--");
        pthread_mutex_lock(&hintbox_mutex);
        ZEYES_LOG_ERROR("DEADBEAF hint box++++");
        if (m_valid_overlap) {
            //hintbox.emplace(hintbox.begin(), m_hintboxi.begin(), m_hintboxi.end());
            hintbox = m_hintboxf;
            //return m_valid_overlap;
        } else {
            hintbox.clear();
            //return m_valid_overlap;
        }
        valid_overlap = m_valid_overlap;
        valid_hintbox = true; //m_valid_hintbox;
        if (keyframe_list.size() == 0) {
            valid_hintbox = false;
        }
        pthread_mutex_unlock(&hintbox_mutex);
        ZEYES_LOG_ERROR("DEADBEAF render panorama----");
        //return m_valid_overlap;
    }

    void render_panorama(void *pixel, const int &width, const int &height, const int &stride) {
        ZEYES_LOG_ERROR("DEADBEAF render panorama++--");
        pthread_mutex_lock(&global_lock);
        ZEYES_LOG_ERROR("DEADBEAF render panorama++++");
        if (keyframe_list.size()) {
            if (m_num_pano_frames == keyframe_list.size()) {
                pthread_mutex_unlock(&global_lock);
                return;
            }

            int offset_x = 0;
            int offset_y = 0;
            int pano_width = all_frames[keyframe_list[0]]->rgb.cols;
            int pano_height = all_frames[keyframe_list[0]]->rgb.rows;
            int pano_center = 0;
            cv::Rect pano_rect;
            cv::Mat mshift = cv::Mat::eye(3, 3, CV_32F);
            cv::Mat pano_homo = cv::Mat::eye(3, 3, CV_32F);
            if (keyframe_list.size() == 1) {
                //pano_image = all_frames[keyframe_list[0]]->rgb;
                pano_center = 0;
            } else {
                pano_center = find_best_pano(pano_rect, pano_homo);
                offset_x = pano_rect.x;
                offset_y = pano_rect.y;
                pano_width = pano_rect.width;
                pano_height = pano_rect.height;
                mshift.at<float>(0, 2) = -offset_x;
                mshift.at<float>(1, 2) = -offset_y;
            }

            int rotation = all_frames[keyframe_list[0]]->rotation;
            //int scale_width = width;
            //int scale_height = height;
            float scale_factor = 1.0;
            cv::Mat r = cv::Mat::eye(3, 3, CV_32F);
            cv::Mat s = cv::Mat::eye(3, 3, CV_32F);
            float valid_width = width;// = pano_width;
            float valid_height = height;// = pano_height;
            if (rotation == 90) {
                //scale_height = width;
                //scale_width = height;
                r = ROTATE_90;
                r.at<float>(0, 2) = pano_height;
                float sf0 = (float) pano_width / height;
                float sf1 = (float) pano_height / width;
                //scale_width = height;
                //scale_height = width;
                scale_factor = sf0 > sf1 ? sf0 : sf1;
                valid_width = pano_height / scale_factor;
                valid_width = valid_width > height ? height : valid_width;
                valid_height = pano_width / scale_factor;
                valid_height = valid_height > width ? width : valid_height;
            } else if (rotation == 0) {
                //
                float sf0 = (float) pano_width / width;
                float sf1 = (float) pano_height / height;
                scale_factor = sf0 > sf1 ? sf0 : sf1;
                valid_height = (float) pano_height / scale_factor;
                valid_height = valid_height > height ? height : valid_height;
                valid_width = (float) pano_width / scale_factor;
                valid_width = valid_width > width ? width : valid_width;
            } else {
                assert(0);
            }

            s.at<float>(2, 2) = scale_factor;
            ZEYES_LOG_ERROR("DEADBEAF %d,%d->%d,%d, valid %f %f", pano_width, pano_height, width,
                            height, valid_width, valid_height);


            //pano_image = cv::Mat::zeros(height, width, CV_8UC3);;
            //mshift.at<float>(2, 2) = scale_factor;
            mshift = mshift * pano_homo;
            mshift = r * mshift;
            mshift = s * mshift;
            //cv::Size dst_size(width, height);

            cv::Mat output(height, width, CV_8UC4, pixel);
            cv::Mat pano_image = cv::Mat::zeros(valid_height, valid_width, CV_8UC3);
            cv::Rect rect(0, 0, valid_width, valid_height);
            cv::Mat roi = output(rect);
            //std::vector<cv::Point2f> corners(4);
            //create_corners4(pano_width, pano_height, corners);
            //std::vector<cv::Point2f> warped_corners;
            for (int i = 0; i < keyframe_list.size(); ++i) {
                cv::Mat homo = mshift * all_frames[keyframe_list[i]]->pano_homo;
                //homo = mshift * homo;
                //homo = r * homo;
                //homo = s * homo;
                cv::warpPerspective(all_frames[keyframe_list[i]]->rgb, pano_image, homo,
                                    pano_image.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
                //ZEYES_LOG_ERROR("DEADBEAF warp homo %f, %f, %f, %f, %f, %f, %f, %f, %f",
                //                homo.at<float>(0, 0), homo.at<float>(0, 1), homo.at<float>(0, 2),
                //                homo.at<float>(1, 0), homo.at<float>(1, 1), homo.at<float>(1, 2),
                //                homo.at<float>(2, 0), homo.at<float>(2, 1), homo.at<float>(2, 2));
                //cv::perspectiveTransform(corners, warped_corners, homo);
                //for (int i = 0; i < 4; ++i) {
                //    ZEYES_LOG_ERROR("DEADBEAF warp %f,%f -> %f,%f", corners[i].x, corners[i].y,
                //                    warped_corners[i].x, warped_corners[i].y);
                //}
            }

            m_num_pano_frames = keyframe_list.size();

            //cv::Mat output(height, width, CV_8UC4, pixel);
            //if (rotation == 90) {
            //    //cv::rotate(pano_image, pano_image,)
            //    cv::rotate(pano_image, pano_image, cv::ROTATE_90_CLOCKWISE);
            //}
            cv::cvtColor(pano_image, roi, CV_RGB2RGBA);
        }
        pthread_mutex_unlock(&global_lock);
        ZEYES_LOG_ERROR("DEADBEAF render panorama----");
    }

    bool track(const cv::Mat &img, const int &rotation, bool is_keyframe) {
        ZEYES_LOG_ERROR("DEADBEAF track++--");
        pthread_mutex_lock(&global_lock);
        ZEYES_LOG_ERROR("DEADBEAF track++++");
        bool result = true;
        Frame *new_frame = get_one_free();
        ZEYES_LOG_ERROR(
                "DEADBEAF ---- track is_key %d frame index %d  num keys: %d num tracking: %d num free: %d all: %d",
                is_keyframe, new_frame->index, keyframe_list.size(), tracking_list.size(),
                free_list.size(), all_frames.size());
        if (is_keyframe) {
            //
            init_frame(new_frame, img, rotation, is_keyframe);
            //
            if (keyframe_list.size()) {
                // map with last keyframe
                Frame *last = all_frames[keyframe_list.back()];
                // TODO:
                cv::Mat homo;
                match_keyframe(last, new_frame, homo);

                if (homo.empty()) {
                    ZEYES_LOG_ERROR("DEADBEAF empty homography");
                    store_one_free(new_frame);
                    result = false;
                } else {
                    if (isStrangPolygon(homo, new_frame->rgb, last->rgb)) {
                        //clear_tracking_list();
                        ZEYES_LOG_ERROR("DEADBEAF strange polygon");
                        store_one_free(new_frame);
                        result = false;
                    } else {
                        new_frame->pano_homo = last->pano_homo * homo;
                        //recalcHomography(new_frame->pano_homo, new_frame->gray.cols,
                        //                 new_frame->gray.rows, PANO_SCALE_FACTOR);
                        store_keyframe(new_frame);

                        //update_panorama();
                        //new_frame->homo =
                    }
                }
            } else {
                ZEYES_LOG_ERROR("DEADBEAF store first key");
                store_keyframe(new_frame);
                m_num_pano_frames = 0;
                //update_panorama();
            }

            if (result == true) {
                int w = new_frame->gray.cols;
                int h = new_frame->gray.rows;
                //int bw = w / 4;
                //int bh = h / 4;
                m_bp_box.x = w * 3 / 8;
                m_bp_box.y = h * 3 / 8;
                m_bp_box.width = w / 4;
                m_bp_box.height = h / 4;
                m_bp_tracker.setTemplate(new_frame->gray, m_bp_box);
                m_bp_result = bp::Result(bp::Matrix33f::Identity());
            }
            clear_tracking_list();
            //update_panorama();
        } else {
            // preview tracking
            if (keyframe_list.size() == 0) {
                // no image taken
                //result = true;
                store_one_free(new_frame);
                result = true;
            } else {
                //
                init_frame(new_frame, img, rotation, is_keyframe);
                result = map(new_frame);
                if (result) {
                    store_one_tracking(new_frame);
                } else {
                    clear_tracking_list();
                    store_one_free(new_frame);
                }
#if 0 // USE_BITPLANES
                {
                    ZEYES_LOG_ERROR("DEADBEAF bp start ----");
                    m_bp_result = m_bp_tracker.track(new_frame->gray, m_bp_result.T);
                    ZEYES_LOG_ERROR("DEADBEAF bp stop -----");
                    result = update_hintbox(m_bp_box, m_bp_result.T.data());
                }
#else
                result = update_hintbox();
#endif
            }
        }
        pthread_mutex_unlock(&global_lock);
        ZEYES_LOG_ERROR("DEADBEAF render panorama----");
        return result;
    }
};

Tracker *Tracker::instance = NULL;
