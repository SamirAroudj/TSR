#
# Copyright (C) 2018 by Author: Aroudj, Samir
# TU Darmstadt - Graphics, Capture and Massively Parallel Computing
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD 3-Clause license. See the License.txt file for details.
#

# project SurfaceReconstruction
# main CMakeLists.txt file
cmake_minimum_required(VERSION 2.8)

project(SurfaceReconstruction)

# user options
option(PCS_REFINEMENT "Enables PCS refinement - not finished and disabled at the moment." off)

# external projects: cached paths to directories
set(BASE_PROJECT_DIR "../../BaseProject" CACHE PATH "Path to BaseProject root directory.")

# extra CMake functionality from base project
include(${BASE_PROJECT_DIR}/CMake/CommonProject.h.cmake)

# add embree
include(${BASE_PROJECT_DIR}/CMake/Embree.h.cmake)

# required flags
if (PCS_REFINEMENT)
	add_definitions(-DPCS_REFINEMENT)
endif (PCS_REFINEMENT)

# flags for SIMD compiler intrinsics
if (${LINUX})
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif (${LINUX})

# flags for OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif (OPENMP_FOUND)

# external link directories

# projects
subdirs(
	SurfaceKernelColoring
	SurfaceReconstruction
	SyntheticSceneEvaluation
	App
)
