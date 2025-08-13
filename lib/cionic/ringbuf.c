#include "shared-bindings/util.h"
#include "py/gc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ringbuf.h"


ringbuf_t *cionic_ringbuf_alloc(uint16_t sample_size, uint16_t nsamples)
{
    int buflen = nsamples*sample_size;
    ringbuf_t *rb = NULL;

    rb = gc_alloc((sizeof(ringbuf_t) + buflen), false, true);
    if(rb == NULL){
        return NULL;
    }

    rb->cbuflen = buflen;
    rb->sample_size = sample_size;
    rb->write_idx = rb->read_idx = 0;

    return rb;
}

void cionic_ringbuf_clear(ringbuf_t *rb)
{
    rb->write_idx = rb->read_idx = 0;
}

int cionic_ringbuf_len(ringbuf_t *rb)
{
    return (rb->write_idx + rb->cbuflen - rb->read_idx) % rb->cbuflen;
}

// fill buf with as many whole samples as will fit; return number of bytes written into buf
int cionic_ringbuf_read_samples(ringbuf_t *rb, void *_buf, int buflen)
{
    uint8_t *buf = _buf;
    const int nsamples = buflen / rb->sample_size;
    const int copylen = nsamples * rb->sample_size;

    // bytes in buffer waiting to be read
    int backavail = 0;  // nbytes from read_idx
    int frontavail = 0; // nbytes from 0 to write_idx

    if (rb->write_idx >= rb->read_idx) {
        // from read_idx to write_idx
        backavail = rb->write_idx - rb->read_idx;
        frontavail = 0;
    } else {
        // from read_idx to end of buf, and then from 0 to write_idx
        backavail = rb->cbuflen - rb->read_idx;
        frontavail = rb->write_idx;
    }

    int backlen = MIN(backavail, copylen);
    memcpy(buf, &rb->cbuf[rb->read_idx], backlen);

    int frontlen = MIN(frontavail, copylen-backlen);
    if (frontlen > 0) {
        memcpy(buf+backlen, &rb->cbuf[0], frontlen);
    } else {
        frontlen = 0;
    }

    rb->read_idx = (rb->read_idx + frontlen + backlen) % rb->cbuflen;

    return backlen + frontlen;
}

bool cionic_ringbuf_write_sample(ringbuf_t *rb, const void *_buf, int buflen)
{
    const uint8_t *buf = _buf;

    // space available in rb
    int backavail = 0;  // nbytes from write_idx
    int frontavail = 0; // nbytes from 0
    if (rb->write_idx >= rb->read_idx) {
        // from write_idx to end of buf and then from 0 to read_idx-1
        backavail = rb->cbuflen - rb->write_idx;
        frontavail = rb->read_idx - 1;
    } else {
        // from write_idx to read_idx-1
        backavail = rb->read_idx - rb->write_idx - 1;
        frontavail = 0;
    }

    if (backavail + frontavail < rb->sample_size) {
        // not enough space
        return false;
    }

    int write_idx = rb->write_idx;

    int backlen = MIN(backavail, rb->sample_size);
    memcpy(&rb->cbuf[write_idx], buf, backlen);

    int frontlen = MIN(frontavail, rb->sample_size - backlen);
    if (frontlen > 0) {
        memcpy(&rb->cbuf[0], buf+backlen, frontlen);
    }

    // set write_idx last for thread immunity
    // consider lock around read and writes for greater protection
    rb->write_idx = (rb->write_idx + rb->sample_size) % rb->cbuflen;

    return true;
}

