//
// Created by alex on 17-10-25.
//

#ifndef ZEROEYES_IMGAGE_QUALITY_HPP
#define ZEROEYES_IMGAGE_QUALITY_HPP

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>

#include "cstdio"
#include "cstdlib"
#include "csetjmp"

#include "laplacian.hpp"

#include "jpeglib.h"
#include "turbojpeg.h"

static inline double yuv_quality(const unsigned char * const& buffer, const int& width, const int& height)
{
    return laplacian_blurry_detector(buffer, width, height);
}


struct my_error_mgr {
    struct jpeg_error_mgr pub;
    /* "public" fields */

    jmp_buf setjmp_buffer;    /* for return to caller */
};

typedef struct my_error_mgr *my_error_ptr;

void my_error_exit(j_common_ptr cinfo) {
    my_error_ptr myerr = (my_error_ptr) cinfo->err;

    (*cinfo->err->output_message)(cinfo);

    longjmp(myerr->setjmp_buffer, 1);
}

double tjpeg_file2yuv(const unsigned char * const& jpeg_buffer, const int& jpeg_size)
{
    tjhandle handle = NULL;
    int width, height, subsample, colorspace;
    int flags = 0;
    int padding = 1; // 1或4均可，但不能是0
    int ret = 0;

    handle = tjInitDecompress();
    tjDecompressHeader3(handle, jpeg_buffer, jpeg_size, &width, &height, &subsample, &colorspace);

    ZEYES_LOG_DEBUG("w: %d h: %d subsample: %d color: %d\n", width, height, subsample, colorspace);

    if (subsample != TJSAMP_420 || colorspace != TJCS_YCbCr) {
        ZEYES_LOG_ERROR("can't process this jpeg image");
        return 0.0;
    }

    flags |= 0;

    unsigned long yuv_size = tjBufSizeYUV2(width, padding, height, subsample);
    ZEYES_LOG_DEBUG("allocate %d bytes yuv buffer", yuv_size);
    unsigned char * yuv_buffer =(unsigned char *)malloc(yuv_size);
    if (yuv_buffer == NULL)
    {
        ZEYES_LOG_ERROR("malloc buffer for rgb failed.\n");
        return 0.0;
    }

    ret = tjDecompressToYUV2(handle, jpeg_buffer, jpeg_size, yuv_buffer, width,
                             padding, height, flags);
    if (ret < 0)
    {
        ZEYES_LOG_ERROR("decompress jpeg failed: %s\n", tjGetErrorStr());
    }

    ZEYES_LOG_DEBUG("Decod finished");

    const unsigned char * pbuffer = yuv_buffer;
    double result = yuv_quality(pbuffer, width, height);

    free(yuv_buffer);
    tjDestroy(handle);

    return result;
}

double jpeg_quality(const char * const& jpeg_file)
{
    struct stat st;
    stat(jpeg_file, &st);
    off_t len = st.st_size;
    int fd = open(jpeg_file, O_RDONLY);
    if (fd == -1) {
        ZEYES_LOG_DEBUG("open file %s failed", jpeg_file);
        return 0.0;
    }

    ZEYES_LOG_DEBUG("open file %s size %d", jpeg_file, len);
    /* Create the memory mapping. */

    void * file_mmap = mmap(0, len, PROT_READ, MAP_SHARED, fd, 0);

    close (fd);

    /* Write a random integer to memory-mapped area. */

    double result = tjpeg_file2yuv((const unsigned char *)file_mmap, len);

    /* Release the memory (unnecessary because the program exits). */

    munmap(file_mmap, len);
    return result;
}

#endif //ZEROEYES_IMGAGE_QUALITY_HPP
