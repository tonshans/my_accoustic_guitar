/*
 * ringbuffer.c — SPSC lock-free ring implementation.
 */
#include "ringbuffer.h"
#include "board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <string.h>

IRAM_ATTR bool gtr_ring_init(gtr_ring_t *ring)
{
    /* Internal RAM keeps the audio path immune to PSRAM access stalls. */
    ring->storage = heap_caps_malloc(RING_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!ring->storage) {
        return false;
    }
    ring->size       = RING_BYTES;
    ring->write_pos  = 0;
    ring->read_pos   = 0;
    return true;
}

void gtr_ring_deinit(gtr_ring_t *ring)
{
    if (ring->storage) {
        free(ring->storage);
        ring->storage = NULL;
    }
}

IRAM_ATTR size_t gtr_ring_write(gtr_ring_t *ring, const void *data, size_t len)
{
    if (len > gtr_ring_free(ring)) {
        len = gtr_ring_free(ring);   /* overwrite-guard: drop instead of corrupt */
    }
    const uint8_t *src = (const uint8_t *)data;
    size_t wp = ring->write_pos;
    size_t first = ring->size - wp;
    if (first > len) first = len;

    memcpy(ring->storage + wp, src, first);
    if (len > first) {
        memcpy(ring->storage, src + first, len - first);
    }
    ring->write_pos = (wp + len) % ring->size;
    return len;
}

IRAM_ATTR size_t gtr_ring_read_nb(gtr_ring_t *ring, void *data, size_t len)
{
    size_t avail = gtr_ring_used(ring);
    if (avail == 0) {
        return 0;
    }
    if (len > avail) {
        len = avail;
    }
    uint8_t *dst = (uint8_t *)data;
    size_t rp = ring->read_pos;
    size_t first = ring->size - rp;
    if (first > len) first = len;

    memcpy(dst, ring->storage + rp, first);
    if (len > first) {
        memcpy(dst + first, ring->storage, len - first);
    }
    ring->read_pos = (rp + len) % ring->size;
    return len;
}

size_t gtr_ring_read_timed(gtr_ring_t *ring, void *data, size_t len, int timeout_ms)
{
    size_t got = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (got < len) {
        size_t n = gtr_ring_read_nb(ring, (uint8_t *)data + got, len - got);
        got += n;
        if (got >= len) break;
        if (esp_timer_get_time() > deadline) break;
        /* Data is produced every ~2.9 ms block; short sleeps keep latency low. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return got;
}
