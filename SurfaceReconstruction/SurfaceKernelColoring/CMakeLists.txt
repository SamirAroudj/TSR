#
# Copyright (C) 2017 by Author: Aroudj, Samir
# TU Darmstadt - Graphics, Capture and Massively Parallel Computing
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD 3-Clause license. See the License.txt file for details.
#

# Surface kernel coloring executable for SurfaceReconstruction library to visualize spread of data-driven surface kernels over meshes
set(componentName SurfaceKernelColoring)
set(appName SurfaceKernelColoring)
set(componentPath ${PROJECT_SOURCE_DIR}/${componentName})

#include OpenGL, glut and glew headers
include(${BASE_PROJECT_DIR}/CMake/LibPNG.h.cmake)
include(${BASE_PROJECT_DIR}/CMake/OpenGL.h.cmake)

# external include directories
link_directories(${EMBREE_BUILD_DIR}/${CMAKE_BUILD_TYPE})

# CMake files
set(cmakeFiles
	${componentPath}/CMakeLists.txt
)

# header files
set(headerFiles
	${componentPath}/SurfaceKernelColoring.h
)

# source files
set(sourceFiles
	${componentPath}/Main.cpp
	${componentPath}/SurfaceKernelColoring.cpp
)

# get all file groups together
set(sourceCode
	${cmakeFiles}
	${generatedHeaderFiles}
	${generatedSourceFiles}
	${headerFiles}
	${sourceFiles}
)

# define executable
add_executable(${appName} ${sourceCode})

# required libs
set(requiredLibs ${requiredLibs}
	SurfaceReconstruction
	embree
	Graphics
	Platform
	Utilities	
	CollisionDetection
	${mathLibName}	
	Patterns
)

# required 3rd party libs
addOpenGLLibs(requiredLibs)
addPNGLibs(requiredLibs)

# do platform specific things (link required libs, enable windows window support etc.)
if (WIN32)
	addWindowsLibs(requiredLibs)
	enableWindowsSubsystem(${appName})

elseif (${LINUX})
	addLinuxLibs(requiredLibs)

endif (WIN32)

target_link_libraries(${appName} ${requiredLibs})

# define source groups for file management within IDE
source_group("CMake Files" FILES ${cmakeFiles})
source_group("Header Files" FILES ${headerFiles})
source_group("Source Files" FILES ${sourceFiles})

