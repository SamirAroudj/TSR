# author: Samir Aroudj
# executable for Disney Research light field to MVE converter
set(componentName DisneyResearchLightFieldToMVE)
set(appName ${componentName}.exe)
set(componentPath ${PROJECT_SOURCE_DIR}/${componentName})

# CMake files
set(cmakeFiles
	${componentPath}/CMakeLists.txt
)

# header files
set(headerFiles
)

# source files
set(sourceFiles
	${componentPath}/Main.cpp
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
	${mathLibName}
	Platform
	Utilities
)

# do platform specific things (link required libs, enable windows window support etc.)
if (WIN32)
	addWindowsLibs(requiredLibs)
	#enableWindowsSubsystem(${appName})

elseif (${LINUX})
	addLinuxLibs(requiredLibs)

endif (WIN32)

target_link_libraries(${appName} ${requiredLibs})

# define source groups for file management within IDE
source_group("CMake Files" FILES ${cmakeFiles})
source_group("Header Files" FILES ${headerFiles})
source_group("Source Files" FILES ${sourceFiles})


