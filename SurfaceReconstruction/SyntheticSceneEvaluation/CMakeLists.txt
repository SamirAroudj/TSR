#
# Copyright (C) 2018 by Author: Aroudj, Samir
# TU Darmstadt - Graphics, Capture and Massively Parallel Computing
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD 3-Clause license. See the License.txt file for details.
#

# synthetic scene evaluation executable for SurfaceReconstruction library
set(componentName SyntheticSceneEvaluation)
set(appName SyntheticSceneEvaluator)
set(componentPath ${PROJECT_SOURCE_DIR}/${componentName})

# CMake files
set(cmakeFiles
	${componentPath}/CMakeLists.txt
)

# header files
set(headerFiles
	${componentPath}/Evaluator.h
)

# source files
set(sourceFiles
	${componentPath}/Main.cpp
	${componentPath}/Evaluator.cpp
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
	Graphics
	Platform
	CollisionDetection
	${mathLibName}	
	Patterns
	${TINYXML2_LIBRARIES}
)

target_link_libraries(${appName} ${requiredLibs})

# define source groups for file management within IDE
source_group("Header Files" FILES ${headerFiles})
source_group("CMake Files" FILES ${cmakeFiles})
source_group("Source Files" FILES ${sourceFiles})
