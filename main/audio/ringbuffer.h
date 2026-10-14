/*
 * ringbuffer.h — single-producer / single-consumer byte ring buffer.
 * Lock-free: producer = I2S RX DMA callback, consumer = DSP task.
 */
#ifndef __GTR_RINGBUFFER_H__
#define __GTR_RINGBUFFER_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gtr_ring {
    uint8_t         *storage;   /* RING_BYTES */
    size_t           size;      /* total bytes */
    volatile size_t  write_pos; /* only modified by producer */
    volatile size_t  read_pos;  /* only modified by consumer */
} gtr_ring_t;

bool gtr_ring_init(gtr_ring_t *ring);
void gtr_ring_deinit(gtr_ring_t *ring);

/* Producer side (RX DMA callback). Returns bytes actually written. */
size_t gtr_ring_write(gtr_ring_t *ring, const void *data, size_t len);

/* Consumer side (DSP task). Waits up to timeout_ms for `len` free bytes. */
size_t gtr_ring_read_timed(gtr_ring_t *ring, void *data, size_t len, int timeout_ms);

/* Non-blocking variant used by the TX callback (never waits). */
size_t gtr_ring_read_nb(gtr_ring_t *ring, void *data, size_t len);

static inline size_t gtr_ring_used(const gtr_ring_t *ring)
{
    return (ring->write_pos - ring->read_pos + ring->size) % ring->size;
}

static inline size_t gtr_ring_free(const gtr_ring_t *ring)
{
    return ring->size - 1 - gtr_ring_used(ring);
}

#ifdef __cplusplus
}
#endif

#endif
