#include "mallocTracker.h"
#include <vector>
#include <utility>
#include <malloc.h>
#include <array>

#define _DEBUG_NEW_REDEFINE_NEW 0
#include "mallocTracker.hpp"

bool trackingEnabled = false;
//std::vector<size_t> mallocEntries;
//std::vector<std::pair<const char *, size_t>> portMallocEntries;

std::array<allocatedMemory_t, NUM_SLOTS> allocatedMemorySlots;

void enableMallocTracking()
{
    //mallocEntries.reserve(100);
    //portMallocEntries.reserve(100);
    for (size_t i = 0; i < allocatedMemorySlots.size(); ++i) {
        allocatedMemorySlots[i].ptr = 0;
        allocatedMemorySlots[i].assignCaller = 0;
        allocatedMemorySlots[i].line = 0;
        allocatedMemorySlots[i].size = 0;
    }

    trackingEnabled = true;
}

const std::array<allocatedMemory_t, NUM_SLOTS>& getAllocatedMemorySlots()
{
    return allocatedMemorySlots;
}

void disableMallocTracking()
{
    trackingEnabled = false;
}

size_t findFreeSlot()
{
    for (size_t i = 0; i < allocatedMemorySlots.size(); ++i) {
        if (allocatedMemorySlots[i].ptr == nullptr) {
            return i;
        }
    }
    return allocatedMemorySlots.size();
}

size_t findPtr(void * ptr)
{
    for (size_t i = 0; i < allocatedMemorySlots.size(); ++i) {
        if (allocatedMemorySlots[i].ptr == ptr) {
            return i;
        }
    }
    return allocatedMemorySlots.size();
}

void addMallocEntry(void * ptr, size_t xSize)
{
    /*if (trackingEnabled && portMallocEntries.size() < portMallocEntries.capacity() - 1) {
        portMallocEntries.push_back(std::pair<const char *, size_t>{callerName, xSize});
    }*/
    const auto idx = findFreeSlot();
    if (idx < allocatedMemorySlots.size()) {
        allocatedMemorySlots[idx].ptr = ptr;
        allocatedMemorySlots[idx].size = xSize;
        allocatedMemorySlots[idx].assignCaller = 0;
        allocatedMemorySlots[idx].line = 0;
    }
}

void assignMallocCaller(void * ptr, const char * callerName, int line)
{
    const auto idx = findPtr(ptr);
    if (idx < allocatedMemorySlots.size()) {
        allocatedMemorySlots[idx].assignCaller = callerName;
        allocatedMemorySlots[idx].line = line;
    }
}

void removeMallocEntry(void * ptr)
{
    const auto idx = findPtr(ptr);
    if (idx < allocatedMemorySlots.size()) {
        allocatedMemorySlots[idx].ptr = 0;
        allocatedMemorySlots[idx].assignCaller = 0;
        allocatedMemorySlots[idx].line = 0;
        allocatedMemorySlots[idx].size = 0;
    }
}

// Define the 'new' operator for C++ to use the freeRTOS memory management
// functions. THIS IS NOT OPTIONAL!
//
//void * operator new(size_t size)
void* newOverload(std::size_t size, char const* function, int line)
{
	void * p;
    p = malloc(size);
    assignMallocCaller(p, function, line);

	#ifdef __EXCEPTIONS
		if (p==0) // did pvPortMalloc succeed?
		throw std::bad_alloc(); // ANSI/ISO compliant behavior
	#endif
	return p;
}

/*
void* operator new(size_t size)
{
    void * p;
    p = malloc(size);
    assignMallocCaller(p, "UNKNOWN", 0);
    return p;
}
*/

// Normal new operator does two things : (1) Allocates memory (2) Constructs an object in allocated memory.
void* operator new(size_t size, const char* function, int line)
{
    //return newOverload(size, function, line);
    void * p = ::new char[size];
    assignMallocCaller(p, function, line);
    return p;
}

// Placement new: we can pass a preallocated memory and construct an object in the passed memory.
void* operator new(size_t size, void* ptr, const char* function, int line)
{
    void * p = ::new(ptr) char[size];
    assignMallocCaller(p, function, line);
    return p;
}

void* operator new[](size_t size, const char* function, int line)
{
    return newOverload(size, function, line);
}

#if 0
//
// Define the 'delete' operator for C++ to use the freeRTOS memory management
// functions. THIS IS NOT OPTIONAL!
//
void operator delete(void *p)
{
    free( p );
	p = NULL;
}
#endif