##############################
# Root build description.
# Path: ./
##############################

cmake_minimum_required (VERSION 2.8.11)
project (Computer_Vision)

# Find includes in corresponding build directories
set (CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set (CMAKE_AUTOMOC ON)
# Necessary flag for building with link to Qt5.
SET (CMAKE_CXX_FLAGS -fPIC)

set (MODEL_DIR "model")
set (UI_DIR "ui")

set (HEADER_DIR "header")
set (CPP_DIR "src")
set (FORM_DIR "form")

# Set include dirs.
find_package (Qt5Gui REQUIRED)
find_package (Qt5Widgets REQUIRED)
find_package (Qt5PrintSupport REQUIRED)
include_directories (${Qt5Gui_INCLUDE_DIRS})
include_directories (${Qt5Widgets_INCLUDE_DIRS})
include_directories (${Qt5PrintSupport_INCLUDE_DIRS})

include_directories (${UI_DIR}/${HEADER_DIR}/)
include_directories (${MODEL_DIR}/${HEADER_DIR}/)

# Creating file groups
# Specifying sources.
file (GLOB_RECURSE MODELS ${MODEL_DIR}/*)
file (GLOB_RECURSE UI ${UI_DIR}/*)
# Creating source groups.
source_group (models FILES ${MODELS})
source_group (headers FILES ${UI})

# Add the executable
add_executable (Computer_Vision
	${UI_DIR}/${HEADER_DIR}/ImageRectificator.h
	${UI_DIR}/${CPP_DIR}/ImageRectificator.cpp
	
	./main.cpp)

# Use the Widgets module from Qt 5.
target_link_libraries (Computer_Vision Qt5::Gui)
target_link_libraries (Computer_Vision Qt5::Widgets)
target_link_libraries (Computer_Vision Qt5::PrintSupport)