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
/**
 * @file stack.h
 * Stack usage analysis helpers
 */

/*
 * Copyright (c) 2015 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MISC_STACK_H_
#define _MISC_STACK_H_

#include <misc/printk.h>

#if defined(CONFIG_INIT_STACKS)
static inline size_t stack_unused_space_get(const char *stack, size_t size)
{
	size_t unused = 0;
	int i;

#ifdef CONFIG_STACK_SENTINEL
	/* First 4 bytes of the stack buffer reserved for the sentinel
	 * value, it won't be 0xAAAAAAAA for thread stacks.
	 */
	stack += 4;
#endif

	/* TODO Currently all supported platforms have stack growth down and
	 * there is no Kconfig option to configure it so this always build
	 * "else" branch.  When support for platform with stack direction up
	 * (or configurable direction) is added this check should be confirmed
	 * that correct Kconfig option is used.
	 */
#if defined(STACK_GROWS_UP)
	for (i = size - 1; i >= 0; i--) {
		if ((unsigned char)stack[i] == 0xaa) {
			unused++;
		} else {
			break;
		}
	}
#else
	for (i = 0; i < size; i++) {
		if ((unsigned char)stack[i] == 0xaa) {
			unused++;
		} else {
			break;
		}
	}
#endif
	return unused;
}
#else
static inline size_t stack_unused_space_get(const char *stack, size_t size)
{
	return 0;
}
#endif

#if defined(CONFIG_INIT_STACKS) && defined(CONFIG_PRINTK)
static inline void stack_analyze(const char *name, const char *stack,
				 unsigned int size)
{
	unsigned int pcnt, unused = 0;

	unused = stack_unused_space_get(stack, size);

	/* Calculate the real size reserved for the stack */
	pcnt = ((size - unused) * 100) / size;

	printk("%s (real size %u):\tunused %u\tusage %u / %u (%u %%)\n", name,
	       size, unused, size - unused, size, pcnt);
}
#else
static inline void stack_analyze(const char *name, const char *stack,
				 unsigned int size)
{
}
#endif

#define STACK_ANALYZE(name, sym) \
	stack_analyze(name, K_THREAD_STACK_BUFFER(sym), \
		      K_THREAD_STACK_SIZEOF(sym))

#endif /* _MISC_STACK_H_ */
