/*

    */
		
#include "utils.h"
#include "buffer.h"
#include <math.h>
#include <stdbool.h>

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

/*
 * See my question:
 * http://stackoverflow.com/questions/40416682/portable-way-to-serialize-float-as-32-bit-integer
 *
 * Regarding the float32_auto functions:
 *
 * Noticed that frexp and ldexp fit the format of the IEEE float representation, so
 * they should be quite fast. They are (more or less) equivalent with the following:
 *
 * float frexp_slow(float f, int *e) {
 *     if (f == 0.0f) {
 *         *e = 0;
 *         return 0.0f;
 *     }
 *
 *     *e = ceilf(log2f(gabsf(f)));
 *     float res = f / powf(2.0f, (float)*e);
 *
 *     if (res >= 1.0f) {
 *         res -= 0.5f;
 *         *e += 1;
 *     }
 *
 *     if (res <= -1.0f) {
 *         res += 0.5f;
 *         *e += 1;
 *     }
 *
 *     return res;
 * }
 *
 * float ldexp_slow(float f, int e) {
 *     return f * powf(2.0f, (float)e);
 * }
 *
 * 8388608.0 is 2^23, which scales the result to fit within 23 bits if sig_abs < 1.0.
 *
 * This should be a relatively fast and efficient way to serialize
 * floating point numbers in a fully defined manner.
 */
void buffer_append_float32_auto(uint8_t* buffer, float number, int32_t *index) {
    int e = 0;
    float sig = frexpf(number, &e);
    float sig_abs = fabsf(sig);
    uint32_t sig_i = 0;

    if (sig_abs >= 0.5f) {
        sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }

    uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
    if (sig < 0) {
        res |= 1 << 31;
    }

    buffer_append_uint32(buffer, res, index);
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res = ((uint16_t) buffer[*index]) << 8 |
                  ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
    uint16_t res = ((uint16_t) buffer[*index]) << 8 |
                   ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res = ((uint32_t) buffer[*index]) << 24 |
                  ((uint32_t) buffer[*index + 1]) << 16 |
                  ((uint32_t) buffer[*index + 2]) << 8 |
                  ((uint32_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
    uint32_t res = ((uint32_t) buffer[*index]) << 24 |
                   ((uint32_t) buffer[*index + 1]) << 16 |
                   ((uint32_t) buffer[*index + 2]) << 8 |
                   ((uint32_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int16(buffer, index) / scale;
}

float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int32(buffer, index) / scale;
}

float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index) {
    uint32_t res = buffer_get_uint32(buffer, index);

    int e = (res >> 23) & 0xFF;
    uint32_t sig_i = res & 0x7FFFFF;
    bool neg = res & (1 << 31);

    float sig = 0.0f;
    if (e != 0 || sig_i != 0) {
        sig = (float)sig_i / (8388608.0f * 2.0f) + 0.5f;
        e -= 126;
    }

    if (neg) {
        sig = -sig;
    }

    return ldexpf(sig, e);
}
