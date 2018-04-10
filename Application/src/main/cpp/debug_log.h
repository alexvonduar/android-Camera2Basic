//
// Created by alex on 17-10-17.
//

#ifndef ZEROEYES_DEBUG_LOG_H
#define ZEROEYES_DEBUG_LOG_H

/**
* Basic logging framework for NDK.
*
* @author Onur Cinar
*/
#include <android/log.h>
#define ZEYES_LOG_LEVEL_VERBOSE 1
#define ZEYES_LOG_LEVEL_DEBUG 2
#define ZEYES_LOG_LEVEL_INFO 3
#define ZEYES_LOG_LEVEL_WARNING 4
#define ZEYES_LOG_LEVEL_ERROR 5
#define ZEYES_LOG_LEVEL_FATAL 6
#define ZEYES_LOG_LEVEL_SILENT 7

#ifndef ZEYES_LOG_TAG
#define ZEYES_LOG_TAG __FILE__
#endif

#ifndef ZEYES_LOG_LEVEL
#define ZEYES_LOG_LEVEL ZEYES_LOG_LEVEL_VERBOSE
#endif

#define ZEYES_LOG_NOOP (void) 0

#ifndef SIMPLE_DEBUG
#define SIMPLE_DEBUG 1
#endif

#if defined(SIMPLE_DEBUG) && SIMPLE_DEBUG
#define ZEYES_LOG_PRINT(level,fmt,...) \
    __android_log_print(level, "", fmt, ##__VA_ARGS__)
#else
#define ZEYES_LOG_PRINT(level,fmt,...) \
    __android_log_print(level, ZEYES_LOG_TAG, "(%s:%u) %s: " fmt, \
        __FILE__, __LINE__, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#endif

#if ZEYES_LOG_LEVEL_VERBOSE >= ZEYES_LOG_LEVEL
#define ZEYES_LOG_VERBOSE(fmt,...) \
    ZEYES_LOG_PRINT(ANDROID_LOG_VERBOSE, fmt, ##__VA_ARGS__)
#else
#define ZEYES_LOG_VERBOSE(...) ZEYES_LOG_NOOP
#endif

#if ZEYES_LOG_LEVEL_DEBUG >= ZEYES_LOG_LEVEL
#define ZEYES_LOG_DEBUG(fmt,...) \
    ZEYES_LOG_PRINT(ANDROID_LOG_DEBUG, fmt, ##__VA_ARGS__)
#else
#define ZEYES_LOG_DEBUG(...) ZEYES_LOG_NOOP
#endif

#if ZEYES_LOG_LEVEL_INFO >= ZEYES_LOG_LEVEL
#define ZEYES_LOG_INFO(fmt,...) \
    ZEYES_LOG_PRINT(ANDROID_LOG_INFO, fmt, ##__VA_ARGS__)
#else
#define ZEYES_LOG_INFO(...) ZEYES_LOG_NOOP
#endif

#if ZEYES_LOG_LEVEL_WARNING >= ZEYES_LOG_LEVEL
#define ZEYES_LOG_WARNING(fmt,...) \
    ZEYES_LOG_PRINT(ANDROID_LOG_WARN, fmt, ##__VA_ARGS__)
#else
#define ZEYES_LOG_WARNING(...) ZEYES_LOG_NOOP
#endif

#if ZEYES_LOG_LEVEL_ERROR >= ZEYES_LOG_LEVEL
#define ZEYES_LOG_ERROR(fmt,...) \
    ZEYES_LOG_PRINT(ANDROID_LOG_ERROR, fmt, ##__VA_ARGS__)
#else
#define ZEYES_LOG_ERROR(...) ZEYES_LOG_NOOP
#endif

#if ZEYES_LOG_LEVEL_FATAL >= ZEYES_LOG_LEVEL
#define ZEYES_LOG_FATAL(fmt,...) \
    ZEYES_LOG_PRINT(ANDROID_LOG_FATAL, fmt, ##__VA_ARGS__)
#else
#define ZEYES_LOG_FATAL(...) ZEYES_LOG_NOOP
#endif

#if ZEYES_LOG_LEVEL_FATAL >= ZEYES_LOG_LEVEL
#define ZEYES_LOG_ASSERT(expression, fmt, ...) \
    if (!(expression)) \
    { \
        __android_log_assert(#expression, ZEYES_LOG_TAG, \
            fmt, ##__VA_ARGS__); \
    }
#else
#define ZEYES_LOG_ASSERT(...) ZEYES_LOG_NOOP
#endif

#endif //ZEROEYES_DEBUG_LOG_H
