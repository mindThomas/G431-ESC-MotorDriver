find_package(CUDA 9.2)
if (${CUDA_FOUND})
	include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})

	list(APPEND CUDA_NVCC_FLAGS "-std=c++11")
	# set(CUDA_VERBOSE_BUILD ON)  # uncomment to debug

	# reduce scope of arch to suppress warning regarding to sm_20
	list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_61,code=sm_61") # GTX 1080
	list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_52,code=sm_52") # GTX 970
	list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_50,code=sm_50") # GTX 960

	mark_as_advanced(
	    CUDA_HOST_COMPILER
	    CUDA_SDK_ROOT_DIR
	    CUDA_TOOLKIT_ROOT_DIR
	    CUDA_USE_STATIC_CUDA_RUNTIME
	    CUDA_rt_LIBRARY
	    NVML_LIB
	)
endif()
