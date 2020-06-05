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
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

void co_skip_delimits(const char **ptr, char delimits)
{
    while(**ptr == delimits)(*ptr)++;
}

void co_get_bytearray_from_string(char** params, uint8_t *result, int array_size)
{
    int i = 0;
    char rand[3];

    co_skip_delimits((const char **)params, ' ');
    for(i=0; i < array_size; i++){
        strncpy(rand, (const char*)*params, 2);
        rand[2]='\0';
        result[i] = strtol(rand, NULL, 16);
        *params = *params + 2;
    }
}

void co_get_uint16_from_string(char** params, uint16_t *result)
{
    uint8_t ret_array[2];
    co_get_bytearray_from_string(params, ret_array, 2);
    *result = (ret_array[0]<<8)|ret_array[1];
}

void co_get_uint32_from_string(char** params, uint32_t *result)
{
    uint8_t ret_array[4];
    co_get_bytearray_from_string(params, ret_array, 4);
    *result = (ret_array[0]<<24)|(ret_array[1]<<16)|(ret_array[2]<<8)|ret_array[3];
}

void co_reverse_bytearray(uint8_t *src, uint8_t *result, int array_size)
{
    for(int i=0; i < array_size;i++){
        result[array_size - i -1] = src[i];
    }
}

unsigned int find_msb_set(uint32_t data)
{
    uint32_t count = 0;
    uint32_t mask = 0x80000000;

    if (!data) {
        return 0;
    }
    while ((data & mask) == 0) {
        count += 1u;
        mask = mask >> 1u;
    }
    return (32 - count);
}

unsigned int find_lsb_set(uint32_t data)
{
    uint32_t count = 0;
    uint32_t mask = 0x00000001;

    if (!data) {
        return 0;
    }
    while ((data & mask) == 0) {
        count += 1u;
        mask = mask << 1u;
    }
    return (1 + count);
}
