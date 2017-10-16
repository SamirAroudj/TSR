# author: Samir Aroudj
# project Converters
# main CMakeLists.txt file
cmake_minimum_required(VERSION 2.8)

project(Converters)

# external projects: cached paths to directories
set(BASE_PROJECT_DIR "../BaseProject" CACHE PATH "Path to BaseProject root directory.")

# extra CMake functionality from base project
include(${BASE_PROJECT_DIR}/CMake/CommonProject.h.cmake)

# projects
subdirs(
	DisneyResearchLightFieldToMVE
)
