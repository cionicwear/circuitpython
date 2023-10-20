#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>

typedef uint32_t elapsed_t;

#define PACKED __attribute__((packed))

typedef struct PACKED int24_t {
    signed value : 24;
} int24_t;

typedef struct PACKED uint24_t {
    unsigned value : 24;
} uint24_t;

// returns value of TYPE from uint8_t INPTR (advancing it), reversing endianness if REVERSE
#define _READ_ENDIAN_ADVPTR(TYPE, INPTR, __REVERSE) ({ \
    TYPE val; \
    uint8_t *valptr = (uint8_t *) &val; \
    if (__REVERSE) { \
        if (sizeof(TYPE) >= 8) { valptr[7] = *INPTR++; } \
        if (sizeof(TYPE) >= 7) { valptr[6] = *INPTR++; } \
        if (sizeof(TYPE) >= 6) { valptr[5] = *INPTR++; } \
        if (sizeof(TYPE) >= 5) { valptr[4] = *INPTR++; } \
        if (sizeof(TYPE) >= 4) { valptr[3] = *INPTR++; } \
        if (sizeof(TYPE) >= 3) { valptr[2] = *INPTR++; } \
        if (sizeof(TYPE) >= 2) { valptr[1] = *INPTR++; } \
                                 valptr[0] = *INPTR++;   \
    } else { \
        val = *(const TYPE *) INPTR; \
        INPTR += sizeof(TYPE); \
    } \
    val; \
})

// returns value of TYPE from PTR, reversing endianness if REVERSE
#define _READ_ENDIAN(TYPE, PTR, __REVERSE) ({    \
    const uint8_t *inptr = (uint8_t *) (PTR);    \
    _READ_ENDIAN_ADVPTR(TYPE, inptr, __REVERSE); \
})

// writes VAL of TYPE to PTR, reversing endianness if REVERSE
#define _WRITE_ENDIAN(TYPE, PTR, VAL, REVERSE) ({ \
    TYPE val = VAL; \
    uint8_t *valptr = (uint8_t *) &val; \
    uint8_t *outptr = (uint8_t *) (PTR); \
    if (REVERSE) { \
        if (sizeof(TYPE) >= 8) { *outptr++ = valptr[7]; } \
        if (sizeof(TYPE) >= 7) { *outptr++ = valptr[6]; } \
        if (sizeof(TYPE) >= 6) { *outptr++ = valptr[5]; } \
        if (sizeof(TYPE) >= 5) { *outptr++ = valptr[4]; } \
        if (sizeof(TYPE) >= 4) { *outptr++ = valptr[3]; } \
        if (sizeof(TYPE) >= 3) { *outptr++ = valptr[2]; } \
        if (sizeof(TYPE) >= 2) { *outptr++ = valptr[1]; } \
                                 *outptr++ = valptr[0];   \
    } else { \
        *(TYPE *) outptr = val; \
        outptr += sizeof(TYPE); \
    } \
})


// These depend on the actual endianness of the system.
// BIG_ENDIAN == 0 == !LITTLE_ENDIAN since our systems are all little-endian, 
#define CIONIC_BIG_ENDIAN 0
#define CIONIC_LITTLE_ENDIAN 1

#define READ_LE(TYPE, PTR) _READ_ENDIAN(TYPE, PTR, CIONIC_BIG_ENDIAN)
#define READ_BE(TYPE, PTR) _READ_ENDIAN(TYPE, PTR, CIONIC_LITTLE_ENDIAN)

#define WRITE_LE(TYPE, PTR, VAL) _WRITE_ENDIAN(TYPE, PTR, VAL, CIONIC_BIG_ENDIAN)
#define WRITE_BE(TYPE, PTR, VAL) _WRITE_ENDIAN(TYPE, PTR, VAL, CIONIC_LITTLE_ENDIAN)

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) \
        (sizeof(array) / sizeof((array)[0]))
#endif

#endif //#ifndef __UTILS_H__