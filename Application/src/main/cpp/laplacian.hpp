//
// Created by alex on 17-10-17.
//

#ifndef ZEROEYES_LAPLACIAN_HPP
#define ZEROEYES_LAPLACIAN_HPP

#include "debug_log.h"
#include "perf.h"
#include "cpu_arch.h"
#include "util.hpp"

#include <cstring>

static const int KER_S_LOG = 5;

#if defined(DEBUG)
#undef DEBUG
#define DEBUG 1
#endif

// Coefficients of LoG
// Convolution of Gaussian kernel and Laplacian kernel
// Gaussian
// 1 1 1
// 1 8 1
// 1 1 1
// Laplacian
// 0  1 0
// 1 -4 1
// 0  1 0
//
static const int COEFF_LOG[KER_S_LOG][KER_S_LOG] = {
        {0, 1, 2, 1, 0},
        {1, 0,-2, 0, 1},
        {2,-2,-8,-2, 2},
        {1, 0,-2, 0, 1},
        {0, 1, 2, 1, 0},
};

static inline void swap_u8(unsigned char&a, unsigned char&b)
{
    unsigned char tmp = b;
    b = a;
    a = tmp;
}

static inline void bubble3(unsigned char &a, unsigned char &b, unsigned char &c)
{
    if (b > a) {
        swap_u8(a, b);
    }
    if (c > b) {
        swap_u8(b, c);
    }
    if (b > a) {
        swap_u8(a, b);
    }
}

static inline unsigned char select_median3(unsigned char a, unsigned char b, unsigned char c)
{
    if (a < b) {
        swap_u8(a, b);
    }

    if (c > a) return a;
    if (c < b) return b;
    return c;
}

static inline unsigned char median9(unsigned char a,
                                    unsigned char b,
                                    unsigned char c,
                                    unsigned char d,
                                    unsigned char e,
                                    unsigned char f,
                                    unsigned char g,
                                    unsigned char h,
                                    unsigned char i)
{
    bubble3(a, b, c);
    bubble3(d, e, f);
    bubble3(g, h, i);
    bubble3(a, d, g);
    bubble3(b, e, h);
    bubble3(c, f, i);
    //bubble3(c, e, g);

    return select_median3(c, e, g);
}

static inline void median_3x3(unsigned char * dst, const unsigned char * const& src, const int& width, const int& height)
{
    memcpy(dst, src, width);

    unsigned char * _dst = dst + width;
    const unsigned char * _src = src + width;

    for (int i = 0; i < height - 2; ++i) {
        _dst[0] = _src[0];
        int j = 0;
        for (j = 1; j < width - 1; ++j) {
            _dst[j] = median9(_src[j - width - 1], _src[j - width], _src[j - width + 1],
                              _src[j - 1], _src[j], _src[j + 1],
                              _src[j + width - 1], _src[j + width], _src[j + width + 1]);
        }
        _dst[j] = _src[j];
        _dst += width;
        _src += width;
    }
    memcpy(_dst, _src, width);
}

static inline int kernel_LoG(const unsigned char * const& buffer, const int& width)
{
    int sum = 0;
    for (int i = 0; i < KER_S_LOG; ++i) {
        for (int j = 0; j < KER_S_LOG; ++j) {
            sum += buffer[width * i + j] * COEFF_LOG[i][j];
        }
    }
    return sum / 16;
}

#define USE_MEDIAN 1
#define DEBUG_FILE 0
#if defined(DEBUG_FILE) && DEBUG_FILE
#define DEBUG_FILE_ONCE 1
static bool save = true;
#endif

static inline double LoG_c(const unsigned char * const& pbuffer, int width, int height)
{
    int padded_w = width - KER_S_LOG + 1;
    int padded_h = height - KER_S_LOG + 1;

#if defined(DEBUG) && DEBUG
    int64_t start = now_ns();
#endif

    const unsigned char * buffer = pbuffer;

#if defined(USE_MEDIAN) && USE_MEDIAN
    unsigned char * dst = (unsigned char *)malloc(sizeof(unsigned char) * width * height);

    median_3x3(dst, pbuffer, width, height);
    buffer = dst;
#endif

#if defined(DEBUG_FILE) && DEBUG_FILE
    FILE * fp = NULL;
    int * scale = NULL;
    if (save) {
        fp = fopen("/storage/self/primary/DCIM/Camera/1.yuv", "w");
        scale = (int *)malloc(sizeof(int) * padded_w * padded_h);
    }
    int max = 0;
    int min = 0;
#endif

    int64_t m = 0;
    double m2 = 0;
    for (int i = 0; i < padded_h; ++i) {
        for (int j = 0; j < padded_w; ++j) {
            const unsigned char * p = buffer + i * width + j;
            int v = kernel_LoG(p, width);
#if defined(DEBUG_FILE) && DEBUG_FILE
            if (save && fp != NULL) {
                scale[i * padded_w + j] = v;
                max = scale[i * padded_w + j] > max ? scale[i * padded_w + j] : max;
                min = scale[i * padded_w + j] < min ? scale[i * padded_w + j] : min;
            }
#endif
            m += v;
            m2 += v * v;
        }
    }
    double avg = (double)m / (padded_w * padded_h);
    //printf("avg = %f SS = %f\n", avg, m2 / (padded_h * padded_w));

    m2 = m2 / (padded_h * padded_w) - avg * avg;


#if defined(DEBUG) && DEBUG
    int64_t stop = now_ns();

    ZEYES_LOG_DEBUG("---- calculated %lld ns", (int64_t)(stop - start));
    //print_cpu_arc();
#endif

#if defined(DEBUG_FILE) && DEBUG_FILE
    if (save) {
        if (fp != NULL) {
            max = max > -min ? max : -min;
            // y
            for (int i = 0; i < padded_h; ++i) {
                for (int j = 0; j < padded_w; ++j) {
                    int s = scale[i * padded_w + j];
                    s = s < 0 ? -s : s;
                    double s1 = 255.0 * (double) s / max;
                    unsigned char c = s1 > 255 ? 255 : (unsigned char) s1;

                    fwrite(&c, 1, 1, fp);
                }
            }
            // u
            for (int i = 0; i < padded_h; ++i) {
                for (int j = 0; j < padded_w; ++j) {
                    int s = scale[i * padded_w + j];
                    unsigned char c = s > 0 ? 0 : 128;

                    fwrite(&c, 1, 1, fp);
                }
            }
            // v
            for (int i = 0; i < padded_h; ++i) {
                for (int j = 0; j < padded_w; ++j) {
                    int s = scale[i * padded_w + j];
                    unsigned char c = s < 0 ? 0 : 128;

                    fwrite(&c, 1, 1, fp);
                }
            }

            fclose(fp);
            fp = NULL;
        }

        free(scale);

#if defined(DEBUG_FILE_ONCE) && DEBUG_FILE_ONCE
        save = false;
#endif
    }

#endif

#if defined(USE_MEDIAN) && USE_MEDIAN
    free(dst);
#endif

    return m2;
}

static const int KER_S = 3;

static inline int laplacian_kernel(const unsigned char *&buffer, int width)
{
    int result = buffer[1];
    result += buffer[width];
    result -= 4 * buffer[width + 1];
    result += buffer[width + 2];
    result += buffer[2 * width + 1];
    return result;
}

static inline double laplacian_c(const unsigned char * &buffer, int width, int height)
{
    int padded_w = width - KER_S + 1;
    int padded_h = height - KER_S + 1;

#if defined(DEBUG) && DEBUG
    int64_t start = now_ns();
#endif

#if defined(DEBUG_FILE) && DEBUG_FILE
    FILE * fp = NULL;
    unsigned int * scale;
    if (save) {
        fp = fopen("/storage/self/primary/DCIM/Camera/1.yuv", "w");
        scale = new unsigned int[padded_w * padded_h];
    }
    int max = 0;
#endif

    int m = 0;
    double m2 = 0;
    for (int i = 0; i < padded_h; ++i) {
        for (int j = 0; j < padded_w; ++j) {
            const unsigned char * p = buffer + i * width + j;
            int v = laplacian_kernel(p, width);
#if defined(DEBUG_FILE) && DEBUG_FILE
            if (save && fp != NULL) {
                scale[i * padded_w + j] = v > 0 ? v : -v;
                max = scale[i * padded_w + j] > max ? scale[i * padded_w + j] : max;
                //fwrite(p, 1, 1, fp);
            }
#endif
            m += v;
            m2 += v * v;
        }
    }
    double avg = (double)m / (padded_w * padded_h);
    printf("avg = %f SS = %f\n", avg, m2 / (padded_h * padded_w));

    m2 = m2 / (padded_h * padded_w) - avg * avg;


#if defined(DEBUG) && DEBUG
    int64_t stop = now_ns();
    ZEYES_LOG_DEBUG("---- calculated %lld ns", (int64_t)(stop - start));
    //print_cpu_arc();
#endif

#if defined(DEBUG_FILE) && DEBUG_FILE
    if (save && fp != NULL) {
        for (int i = 0; i < padded_h; ++i) {
            for (int j = 0; j < padded_w; ++j) {
                unsigned int s = scale[i * padded_w + j];
                double s1 = 255.0 * (double)s / max;
                unsigned char c = s1 > 255 ? 255 : (unsigned char)s1;

                fwrite(&c, 1, 1, fp);
            }
        }
    }

    if (save) {
        delete [] scale;
        if (fp != NULL) {
            fclose(fp);
        }
#if defined(DEBUG_FILE_ONCE) && DEBUG_FILE_ONCE
        save = false;
#endif
    }
#endif

    return m2;
}

static inline void LAP_KER4(const unsigned int&a, const unsigned int& b, const unsigned int& c, int& m, double& m2) \
    { \
        int var1 = (((a) >> 8) & 0xff); \
        var1 += ((b) & 0xff); \
        var1 += (((b) >> 16) & 0xff); \
        var1 += (((c) >> 8) & 0xff); \
        var1 -= (((b) >> 8) & 0xff) * 4; \
        int var2 = (((a) >> 16) & 0xff); \
        var2 += (((b) >> 8) & 0xff); \
        var2 += (((b) >> 24) & 0xff); \
        var2 += (((c) >> 16) & 0xff); \
        var2 -= (((b) >> 16) & 0xff) * 4; \
        (m) += var1 + var2; \
        (m2) += var1 * var1; \
        (m2) += var2 * var2; \
    }

static inline void LAP_KER4_MED(const unsigned int& a1, const unsigned int& b1, const unsigned int& c1,
        const unsigned int& a2, const unsigned int& b2, const unsigned int& c2,
        int& m, double& m2) \
    { \
        int var1 = (((a1) >> 24) & 0xff); \
        var1 += (((b1) >> 16) & 0xff); \
        var1 += ((b2) & 0xff); \
        var1 += (((c1) >> 24) & 0xff); \
        var1 -= (((b1) >> 24) & 0xff) * 4; \
        int var2 = ((a2) & 0xff); \
        var2 += (((b1) >> 24) & 0xff); \
        var2 += (((b2) >> 8) & 0xff); \
        var2 += ((c2) & 0xff); \
        var2 -= ((b2) & 0xff) * 4; \
        (m) += var1 + var2; \
        (m2) += var1 * var1; \
        (m2) += var2 * var2; \
    }

static inline double laplacian_aligned4(const unsigned char *& buffer, int width, int height)
{
    int padded_w = width - KER_S + 1;
    int padded_h = height - KER_S + 1;

#if defined(DEBUG) && DEBUG
    int64_t start = now_ns();
#endif

    int m = 0;
    double m2 = 0;
    const unsigned char * p = buffer;
    for (int i = 0; i < padded_h; ++i) {
        unsigned int * p_top = (unsigned int *)(buffer + i * width);
        unsigned int * p_med = (unsigned int *)(buffer + (i + 1) * width);
        unsigned int * p_bot = (unsigned int *)(buffer + (i + 2) * width);

        int top = p_top[0];
        int med = p_med[0];
        int bot = p_bot[0];
        LAP_KER4(top, med, bot, m, m2);
        for (int j = 1; j < width / 4; ++j) {
            int top1 = p_top[j];
            int med1 = p_med[j];
            int bot1 = p_bot[j];
            LAP_KER4_MED(top, med, bot, top1, med1, bot1, m, m2);
            LAP_KER4(top1, med1, bot1, m, m2);
            top = top1;
            med = med1;
            bot = bot1;
        }
    }
    double avg = (double)m / (padded_w * padded_h);

    m2 = m2 / (padded_h * padded_w) - avg * avg;

#if defined(DEBUG) && DEBUG
    int64_t stop = now_ns();

    ZEYES_LOG_DEBUG("---- calculated %lld ns", (int64_t)(stop - start));
    //print_cpu_arc();
#endif

    return m2;
}

static inline double laplacian_blurry_detector(const unsigned char * const& buffer, const int& width, const int& height)
{
    if (CHECK_ALIGN(buffer, 4) && CHECK_ALIGN(width, 4)) {
        // TODO:
        //ZEYES_LOG_DEBUG("aligned pointer and width");
        //return laplacian_c(buffer, width, height);
        return LoG_c(buffer, width, height);
    } else {
        //return laplacian_c(buffer, width, height);
        return LoG_c(buffer, width, height);
    }
}

#endif //ZEROEYES_LAPLACIAN_HPP
