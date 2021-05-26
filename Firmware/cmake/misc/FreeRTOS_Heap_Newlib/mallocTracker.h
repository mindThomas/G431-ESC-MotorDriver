#ifndef MALLOC_TRACKER_H
#define MALLOC_TRACKER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void enableMallocTracking();
void disableMallocTracking();
void addMallocEntry(void * ptr, size_t xSize);
void assignMallocCaller(void * ptr, const char * callerName, int line);
void removeMallocEntry(void * ptr);

#ifdef __cplusplus
}

extern "C++" {
#include <array>
typedef struct __attribute__((__packed__)) {
    void * ptr;
    const char * assignCaller;
    int line;
    size_t size;
} allocatedMemory_t;

#define NUM_SLOTS 100

const std::array<allocatedMemory_t, NUM_SLOTS>& getAllocatedMemorySlots();
}
#endif

#endif // MALLOC_TRACKER_H