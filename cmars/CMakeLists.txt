#--------------------------------------------------------------
# CMake configuration script for linking an external project to
# Chrono libraries.
# For a particular user project, modify sections 1 - 4 below,
# as appropriate.
#--------------------------------------------------------------
 
cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

#--------------------------------------------------------------
# 1. Set the project name
#--------------------------------------------------------------

project(CMars)

set(IMAGE_DATA_LIB "" CACHE PATH "Path to the pre-built image-data shared library (set to the .a file)")
if(NOT IMAGE_DATA_LIB)
    message(FATAL_ERROR "IMAGE_DATA_LIB is not set. Please provide the full path to the shared library")
endif()

set(IMAGE_DATA_INCLUDES "" CACHE PATH "Path to the image-data header directory")
if(NOT IMAGE_DATA_INCLUDES)
    message(FATAL_ERROR "IMAGE_DATA_INCLUDES is not set. Please provide the full path to the headers")
endif()


find_package(OpenGL REQUIRED)

find_package(Chrono
	           COMPONENTS PARSERS FSI VEHICLE
                OPTIONAL_COMPONENTS PardisoMKL VSG IRRLICHT
                CONFIG)


if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

add_subdirectory(demos)

#add_DLL_copy_command()
