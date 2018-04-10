//
// Created by alex on 17-10-17.
//

#ifndef ZEROEYES_PERF_H
#define ZEROEYES_PERF_H

#include <time.h>
#include <stdio.h>
#include <stdlib.h>

/* return current time in micro seconds */
static inline int64_t now_ns(void)
{
    struct timespec res;
    clock_gettime(CLOCK_REALTIME, &res);
    return (int64_t)res.tv_sec * 1000000000 + res.tv_nsec;
}

#endif //ZEROEYES_PERF_H
