#ifndef MALLOC_TRACKER_OVERLOAD_H
#define MALLOC_TRACKER_OVERLOAD_H

#ifdef __cplusplus
extern "C++" {

#include <cstddef>

struct traced_new_tag_t {};
constexpr traced_new_tag_t traced_new_tag;

//extern void* operator new(std::size_t sz,char const* file, int line);
void* newOverload(std::size_t sz, char const* function, int line);

//void* operator new (std::size_t size);

void* operator new (std::size_t size, const char* function, int line);
void* operator new (std::size_t size, const std::nothrow_t& nothrow_value, const char* function, int line) noexcept;
void* operator new (std::size_t size, void* ptr, const char* function, int line) noexcept;

void* operator new[] (std::size_t size, const char* function, int line);
void* operator new[] (std::size_t size, const std::nothrow_t& nothrow_value, const char* function, int line) noexcept;
void* operator new[] (std::size_t size, void* ptr, const char* function, int line) noexcept;

//void* operator new(size_t size, const char* function, int line);
//void* operator new[](size_t size, const char* function, int line);
//void operator delete(void* ptr, const char* file, int line) _NOEXCEPT;
//void operator delete[](void* ptr, const char* file, int line) _NOEXCEPT;

}

#ifndef _DEBUG_NEW_REDEFINE_NEW
#define _DEBUG_NEW_REDEFINE_NEW 1
#endif

//#define new new(__FUNCTION__, __LINE__)
#define DEBUG_NEW new(__FUNCTION__, __LINE__)
#define DEBUG_NEW2(args...) new(args, __FUNCTION__, __LINE__)

#if _DEBUG_NEW_REDEFINE_NEW
//#define new(size, args...) DEBUG_NEW2(size)
#define FIRST_ARG_(N, ...) N
#define FIRST_ARG(args) FIRST_ARG_ args
#define new DEBUG_NEW // captures (1)
//#define new(args...) DEBUG_NEW2(args) // captures (2) and (3)

// new will be called with:
// - new Type            (1)
// - new(pointer) Type   (2)
// - new(others...) Type (3)
// How do we make the macro support all three?


#endif

#endif

#endif