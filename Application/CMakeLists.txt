# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

cmake_minimum_required(VERSION 3.4.1)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG=1")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")


# configure import libs
set(fastcv_DIR ${CMAKE_SOURCE_DIR}/../fastcv)
add_library( fastcv STATIC IMPORTED )
set_target_properties( fastcv PROPERTIES IMPORTED_LOCATION ${fastcv_DIR}/libs/${ANDROID_ABI}/libfastcv.a )

set(opencv_DIR ${CMAKE_SOURCE_DIR}/../opencv)

set(opencv_INC ${opencv_DIR}/${ANDROID_ABI}/sdk/native/jni/include)

add_library( cvvideo STATIC IMPORTED )
set_target_properties( cvvideo PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_video.a )

add_library( cvimgproc STATIC IMPORTED )
set_target_properties( cvimgproc PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_imgproc.a )

add_library( cvimgcodecs STATIC IMPORTED )
set_target_properties( cvimgcodecs PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_imgcodecs.a )

add_library( cvphoto STATIC IMPORTED )
set_target_properties( cvphoto PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_photo.a )

add_library( cvhighgui STATIC IMPORTED )
set_target_properties( cvhighgui PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_highgui.a )

add_library( cvcore STATIC IMPORTED )
set_target_properties( cvcore PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_core.a )

add_library( cvfeatures2d STATIC IMPORTED )
set_target_properties( cvfeatures2d PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_features2d.a )

add_library( cvcalib3d STATIC IMPORTED )
set_target_properties( cvcalib3d PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_calib3d.a )

add_library( cvflann STATIC IMPORTED )
set_target_properties( cvflann PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/staticlibs/libopencv_flann.a )

add_library( cvhal STATIC IMPORTED )
set_target_properties( cvhal PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/3rdparty/libs/libtegra_hal.a )

add_library( cvimf STATIC IMPORTED )
set_target_properties( cvimf PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/3rdparty/libs/libIlmImf.a )

add_library( cvjpeg STATIC IMPORTED )
set_target_properties( cvjpeg PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/3rdparty/libs/liblibjpeg.a )

add_library( cvjasper STATIC IMPORTED )
set_target_properties( cvjasper PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/3rdparty/libs/liblibjasper.a )

add_library( cvwebp STATIC IMPORTED )
set_target_properties( cvwebp PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/3rdparty/libs/liblibwebp.a )

add_library( cvpng STATIC IMPORTED )
set_target_properties( cvpng PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/3rdparty/libs/liblibpng.a )

add_library( cvtiff STATIC IMPORTED )
set_target_properties( cvtiff PROPERTIES IMPORTED_LOCATION ${opencv_DIR}/${ANDROID_ABI}/sdk/native/3rdparty/libs/liblibtiff.a )

set(jpeg_DIR ${CMAKE_SOURCE_DIR}/../jpeg)

set(jpeg_INC ${jpeg_DIR}/include)

add_library( libturbojpeg SHARED IMPORTED )
set_target_properties( libturbojpeg PROPERTIES IMPORTED_LOCATION ${jpeg_DIR}/libs/${ANDROID_ABI}/libturbojpeg.so )

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

find_library( z-lib z )

add_library(cpufeatures STATIC
            ${ANDROID_NDK}/sources/android/cpufeatures/cpu-features.c)

add_library( # Sets the name of the library.
             find-homography

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             # Associated headers in the same location as their source
             # file are automatically included.
             src/main/cpp/find-homography.cpp
             src/main/cpp/ORBextractor.cc )

target_link_libraries( # Specifies the target library.
                       find-homography

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} ${z-lib} cvvideo cvfeatures2d cvcalib3d cvimgproc cvimgcodecs
                       cvhighgui cvphoto cvcore  cvflann cvhal cvimf cvjpeg cvjasper cvwebp cvpng cvtiff cpufeatures
                       libturbojpeg fastcv )


target_include_directories(find-homography PRIVATE ${opencv_INC} ${jpeg_INC})


add_library( # Sets the name of the library.
             imagetracker

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             # Associated headers in the same location as their source
             # file are automatically included.
             src/main/cpp/imagetracker-jni.cpp
             src/main/cpp/ORBextractor.cc )

target_link_libraries( # Specifies the target library.
                       imagetracker

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} ${z-lib} android jnigraphics cvvideo cvfeatures2d cvcalib3d cvimgproc cvimgcodecs
                       cvhighgui cvphoto cvcore  cvflann cvhal cvimf cvjpeg cvjasper cvwebp cvpng cvtiff cpufeatures
                       libturbojpeg fastcv )


target_include_directories(imagetracker PRIVATE ${opencv_INC} ${jpeg_INC} ${CMAKE_SOURCE_DIR}/src/main/cpp/gms)