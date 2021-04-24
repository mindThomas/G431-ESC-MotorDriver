/*
 * Since at least FreeRTOS V7.5.3 uxTopUsedPriority is no longer
 * present in the kernel, so it has to be supplied by other means for
 * OpenOCD's threads awareness.
 *
 * Add this file to your project, and, if you're using --gc-sections,
 * ``--undefined=uxTopUsedPriority'' (or
 * ``-Wl,--undefined=uxTopUsedPriority'' when using gcc for final
 * linking) to your LDFLAGS; same with all the other symbols you need.
 */

#include "FreeRTOS.h"

/*
#ifdef __GNUC__
#define USED __attribute__((used))
#else
#define USED
#endif

const volatile int USED uxTopUsedPriority = configMAX_PRIORITIES - 1;
*/

/* This is only really needed for debugging with openOCD:
* Since at least FreeRTOS V7.5.3 uxTopUsedPriority is no longer
* present in the kernel, so it has to be supplied by other means for
* OpenOCD’s threads awareness.
*
* Add this file to your project, and, if you’re using –gc-sections,
* “–undefined=uxTopUsedPriority” (or
* “-Wl,–undefined=uxTopUsedPriority” when using gcc for final
* linking) to your LDFLAGS; same with all the other symbols you need.
*/
volatile const int
#ifdef __GNUC__
__attribute__((used))
#endif
uxTopUsedPriority = configMAX_PRIORITIES;
