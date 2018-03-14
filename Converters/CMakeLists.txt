#
# Copyright (C) 2017 by Author: Aroudj, Samir
# TU Darmstadt - Graphics, Capture and Massively Parallel Computing
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD 3-Clause license. See the License.txt file for details.
#

# project Converters
# main CMakeLists.txt file
cmake_minimum_required(VERSION 2.8)

project(Converters)

# external projects: cached paths to directories
set(BASE_PROJECT_DIR "../BaseProject" CACHE PATH "Path to BaseProject root directory.")

set(SURFRECON_BUILD_DIR "../SurfaceReconstruction/Build" CACHE PATH "Path to the TSR Surface Reconstruction Build directory.")
set(SURFRECON_INCLUDE_DIR "../SurfaceReconstruction/" CACHE PATH "Paths to desired Surface Reconstruction directories.")

# extra CMake functionality from base project
include(${BASE_PROJECT_DIR}/CMake/CommonProject.h.cmake)

# external include directories
include_directories(${SURFRECON_INCLUDE_DIR})

# external link directories
link_directories(${SURFRECON_BUILD_DIR}/${CMAKE_BUILD_TYPE})

# projects
subdirs(
	DisneyResearchLightFieldToMVE
)
