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
# Necessary flags for C++11 and linking to Qt5.
SET (CMAKE_CXX_FLAGS "-std=c++11 -fPIC")

set (UI_DIR "ui")
set (MODEL_DIR "models")
set (MATH_DIR "math")
set (UTILS_DIR "utils")

set (HEADER_DIR "header")
set (CPP_DIR "src")

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Set include dirs.
find_package (Qt5Core REQUIRED)
find_package (Qt5Gui REQUIRED)
find_package (Qt5Widgets REQUIRED)
find_package (Eigen3 REQUIRED)

include_directories (${Qt5Core_INCLUDE_DIRS})
include_directories (${Qt5Gui_INCLUDE_DIRS})
include_directories (${Qt5Widgets_INCLUDE_DIRS})
include_directories (${EIGEN3_INCLUDE_DIR})

include_directories (${UI_DIR}/${HEADER_DIR}/)
include_directories (${MODEL_DIR}/${HEADER_DIR}/)
include_directories (${MATH_DIR}/${HEADER_DIR}/)
include_directories (${UTILS_DIR}/${HEADER_DIR}/)

# Creating file groups
# Specifying sources.
file (GLOB_RECURSE UI ${UI_DIR}/*)
file (GLOB_RECURSE MODELS ${MODEL_DIR}/*)
file (GLOB_RECURSE MATH ${MATH_DIR}/*)
file (GLOB_RECURSE UTILS ${UTILS_DIR}/*)
# Creating source groups.
source_group (ui FILES ${UI})
source_group (models FILES ${MODELS})
source_group (math FILES ${MATH})
source_group (utils FILES ${UTILS})

# Add the executable
add_executable (Computer_Vision
	${UI_DIR}/${HEADER_DIR}/ImageRectificator.h	${UI_DIR}/${CPP_DIR}/ImageRectificator.cpp
	${UI_DIR}/${HEADER_DIR}/ClickableLabel.h	${UI_DIR}/${CPP_DIR}/ClickableLabel.cpp
	${UI_DIR}/${HEADER_DIR}/SelectedPixel.h		${UI_DIR}/${CPP_DIR}/SelectedPixel.cpp
	
	${MODEL_DIR}/${HEADER_DIR}/RectificationController.h	${MODEL_DIR}/${CPP_DIR}/RectificationController.cpp
	
	${MATH_DIR}/${HEADER_DIR}/IPointMapper.h
	${MATH_DIR}/${HEADER_DIR}/ProjectionRectificator.h	${MATH_DIR}/${CPP_DIR}/ProjectionRectificator.cpp
	
	${UTILS_DIR}/${HEADER_DIR}/CircularList.h
	./main.cpp
	./CMakeLists.txt)

# Qt 5 modules.
target_link_libraries (Computer_Vision Qt5::Core)
target_link_libraries (Computer_Vision Qt5::Gui)
target_link_libraries (Computer_Vision Qt5::Widgets)