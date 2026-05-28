#ifndef PROBE_RING_BUFFER_H
#define PROBE_RING_BUFFER_H


#include <stddef.h>
#include <stdint.h>

//  Must be power-of-two.
#define RingBuffer_BUFFER_SIZE  ((size_t)256)
const size_t RingBuffer_BUFFER_MASK = RingBuffer_BUFFER_SIZE - 1;

typedef struct{
    size_t head;
    size_t tail;
    uint8_t buffer[RingBuffer_BUFFER_SIZE];
} RingBuffer;

void RingBuffer_clear(RingBuffer* buffer){
    buffer->head = 0;
    buffer->tail = 0;
}
size_t RingBuffer_size(const RingBuffer* buffer){
    return buffer->tail - buffer->head;
}
bool RingBuffer_putc(RingBuffer* buffer, uint8_t data){
    size_t head = buffer->head;
    size_t tail = buffer->tail;

    if (tail - head >= RingBuffer_BUFFER_SIZE){
        return false;
    }

//    printf("RingBuffer_putc(): head = %zu, tail = %zu\n", head, tail);

    buffer->buffer[tail & RingBuffer_BUFFER_MASK] = data;
    buffer->tail = tail + 1;
    return true;
}
uint8_t* RingBuffer_write_buffer(RingBuffer* buffer, size_t* available_bytes){
    size_t head = buffer->head;
    size_t tail = buffer->tail;
    size_t available = RingBuffer_BUFFER_SIZE - (tail - head);

    if (available == 0){
        *available_bytes = available;
        return NULL;
    }

    tail &= RingBuffer_BUFFER_MASK;
    head &= RingBuffer_BUFFER_MASK;
    if (tail >= head){
        available = RingBuffer_BUFFER_SIZE - tail;
    }

    *available_bytes = available;
    return &buffer->buffer[tail];
}
void RingBuffer_push_back(RingBuffer* buffer, size_t bytes){
    buffer->tail += bytes;
}
const uint8_t* RingBuffer_read_buffer(RingBuffer* buffer, size_t* available_bytes){
    size_t head = buffer->head;
    size_t tail = buffer->tail;
    size_t available = tail - head;

    if (available == 0){
        *available_bytes = available;
        return NULL;
    }

//    printf("RingBuffer_read_consecutive(): head = %zu, tail = %zu, max_bytes = %zu\n", head, tail, max_bytes);
    tail &= RingBuffer_BUFFER_MASK;
    head &= RingBuffer_BUFFER_MASK;
    if (tail < head){
        available = RingBuffer_BUFFER_SIZE - head;
    }

//    printf("RingBuffer_read_consecutive(): head = %zu, tail = %zu, available = %zu\n", head, tail, available);
//    printf("RingBuffer_read_consecutive(): head = %zu, tail = %zu, max_bytes = %zu\n", head, tail, max_bytes);

//    printf("bytes = %zu\n", max_bytes);
//    printf("RingBuffer_read_consecutive(): head = %zu, tail = %zu, reading = %zu\n", head, tail, max_bytes);

    *available_bytes = available;
    return &buffer->buffer[head];
}
void RingBuffer_pop_front(RingBuffer* buffer, size_t bytes){
    buffer->head += bytes;
}


#endif
