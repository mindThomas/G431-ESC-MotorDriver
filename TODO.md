# To Do list

1. Move heap memory allocation tracking code into a library. This should also remove the printout from CPULoad and it should remove the vector needed to store allocated memory info. But how about the patch to FreeRTOS pvPortMalloc to trace the caller name? How do we wrap that into a library?