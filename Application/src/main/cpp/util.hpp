//
// Created by alex on 17-10-17.
//

#ifndef ZEROEYES_UTIL_HPP
#define ZEROEYES_UTIL_HPP

#include <stdint.h>

static inline bool CHECK_ALIGN(const void *p, int align)
{
    return (reinterpret_cast<intptr_t >(p) % align) == 0;
}

static inline bool CHECK_ALIGN(int v, int align)
{
    return (v % align) == 0;
}

#endif //ZEROEYES_UTIL_HPP
