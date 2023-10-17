#ifndef CIONIC_RINGBUF_H_
#define CIONIC_RINGBUF_H_

typedef struct ringbuf_t {
    uint16_t cbuflen;
    uint16_t sample_size;
    uint16_t write_idx;
    uint16_t read_idx;
    uint8_t cbuf[];
} ringbuf_t;

// allocate new ringbuf
ringbuf_t *cionic_ringbuf_alloc(uint16_t sample_size, uint16_t nsamples);

// clear any existing data in ringbuf
void cionic_ringbuf_clear(ringbuf_t *rb);

// return nbytes in ringbuf available to be read
int cionic_ringbuf_len(ringbuf_t *rb);

// fill buf with as many whole samples as will fit; return number of bytes written into buf
int cionic_ringbuf_read_samples(ringbuf_t *rb, void *buf, int buflen);

// write all buflen bytes from buf into ringbuf, or return false
bool cionic_ringbuf_write_sample(ringbuf_t *rb, const void *buf, int buflen);

// return number of samples in buffer
static inline int cionic_ringbuf_num_samples(ringbuf_t *rb) {
    return cionic_ringbuf_len(rb)/rb->sample_size;
}

#endif
