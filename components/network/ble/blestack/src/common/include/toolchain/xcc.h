/*
 * Copyright (c) 2020 Bouffalolab.
 *
 * This file is part of
 *     *** Bouffalolab Software Dev Kit ***
 *      (see www.bouffalolab.com).
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Bouffalo Lab nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ZEPHYR_INCLUDE_TOOLCHAIN_XCC_H_
#define ZEPHYR_INCLUDE_TOOLCHAIN_XCC_H_

/* toolchain/gcc.h errors out if __BYTE_ORDER__ cannot be determined
 * there. However, __BYTE_ORDER__ is actually being defined later in
 * this file. So define __BYTE_ORDER__ to skip the check in gcc.h
 * and undefine after including gcc.h.
 */
#define __BYTE_ORDER__
#include <toolchain/gcc.h>
#undef __BYTE_ORDER__

#include <stdbool.h>

/* XCC doesn't support __COUNTER__ but this should be good enough */
#define __COUNTER__ __LINE__

#undef __in_section_unique
#define __in_section_unique(seg) \
	__attribute__((section("." STRINGIFY(seg) "." STRINGIFY(__COUNTER__))))

#ifndef __GCC_LINKER_CMD__
#include <xtensa/config/core.h>

/*
 * XCC does not define the following macros with the expected names, but the
 * HAL defines similar ones. Thus we include it and define the missing macros
 * ourselves.
 */
#if XCHAL_MEMORY_ORDER == XTHAL_BIGENDIAN
#define __BYTE_ORDER__		__ORDER_BIG_ENDIAN__
#elif XCHAL_MEMORY_ORDER == XTHAL_LITTLEENDIAN
#define __BYTE_ORDER__		__ORDER_LITTLE_ENDIAN__
#else
#error "Cannot determine __BYTE_ORDER__"
#endif

#endif /* __GCC_LINKER_CMD__ */

#define __builtin_unreachable() do { __ASSERT(false, "Unreachable code"); } \
	while (true)

#endif
