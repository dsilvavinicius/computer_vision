##############################
# Root build description.
# Path: ./
##############################

cmake_minimum_required (VERSION 2.8.11.2)
project (Computer_Vision)

# Creates the same source groups for library and tests.
macro (create_source_groups ui_dir model_dir math_dir util_dir)
	# Creating file groups
	# Specifying sources.
	file (GLOB_RECURSE UI ${ui_dir}/*)
	file (GLOB_RECURSE MODEL ${model_dir}/*)
	file (GLOB_RECURSE MATH ${math_dir}/*)
	file (GLOB_RECURSE UTIL ${util_dir}/*)
	# Creating source groups.
	source_group (ui FILES ${UI})
	source_group (model FILES ${MODEL})
	source_group (math FILES ${MATH})
	source_group (util FILES ${UTIL})
endmacro (create_source_groups)

# Find includes in corresponding build directories
set (CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set (CMAKE_AUTOMOC ON)
# Necessary flags for C++11 and linking to Qt5.
SET (CMAKE_CXX_FLAGS "-std=c++11 -fPIC")

set (UI_DIR "ui")
set (MODEL_DIR "model")
set (MATH_DIR "math")
set (UTILS_DIR "util")

set (HEADER_DIR "header")
set (CPP_DIR "cpp")

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Set include dirs.
find_package (OpenGL REQUIRED)
include_directories (${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})

find_package (Qt5Core REQUIRED)
find_package (Qt5Gui REQUIRED)
find_package (Qt5Widgets REQUIRED)
find_package (Qt53D REQUIRED)
find_package (Eigen3 REQUIRED)

include_directories (${Qt5Core_INCLUDE_DIRS}
	${Qt5Gui_INCLUDE_DIRS}
	${Qt5Widgets_INCLUDE_DIRS}
	${Qt53D_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}

	${HEADER_DIR}/${UI_DIR}/
	${HEADER_DIR}/${MODEL_DIR}/
	${HEADER_DIR}/${MATH_DIR}/
	${HEADER_DIR}/${UTILS_DIR}/
)

find_package (OpenCV REQUIRED)

create_source_groups (UI_DIR MODEL_DIR MATH_DIR UTILS_DIR)

add_library(Computer_Vision_Lib
	${HEADER_DIR}/${UI_DIR}/ImageRectificator.h			${CPP_DIR}/${UI_DIR}/ImageRectificator.cpp
	${HEADER_DIR}/${UI_DIR}/ClickableLabel.h			${CPP_DIR}/${UI_DIR}/ClickableLabel.cpp
	${HEADER_DIR}/${UI_DIR}/SelectedPixel.h				${CPP_DIR}/${UI_DIR}/SelectedPixel.cpp
	${HEADER_DIR}/${UI_DIR}/ReconstructionViewer.h		${CPP_DIR}/${UI_DIR}/ReconstructionViewer.cpp
	
	${HEADER_DIR}/${MODEL_DIR}/RectificationController.h	${CPP_DIR}/${MODEL_DIR}/RectificationController.cpp
	${HEADER_DIR}/${MODEL_DIR}/PanoramaController.h			${CPP_DIR}/${MODEL_DIR}/PanoramaController.cpp
	${HEADER_DIR}/${MODEL_DIR}/ReconstructionController.h	${CPP_DIR}/${MODEL_DIR}/ReconstructionController.cpp
	
	${HEADER_DIR}/${MATH_DIR}/IPointMapper.h							${CPP_DIR}/${MATH_DIR}/IPointMapper.cpp
	${HEADER_DIR}/${MATH_DIR}/AssistedSimilarityFromProjRectificator.h	${CPP_DIR}/${MATH_DIR}/AssistedSimilarityFromProjRectificator.cpp
	${HEADER_DIR}/${MATH_DIR}/AffineFromProjRectificator.h				${CPP_DIR}/${MATH_DIR}/AffineFromProjRectificator.cpp
	${HEADER_DIR}/${MATH_DIR}/SimilarityFromAffineRectificator.h		${CPP_DIR}/${MATH_DIR}/SimilarityFromAffineRectificator.cpp
	${HEADER_DIR}/${MATH_DIR}/SimilarityFromProjRectificator.h			${CPP_DIR}/${MATH_DIR}/SimilarityFromProjRectificator.cpp
	${HEADER_DIR}/${MATH_DIR}/ReconstructionRansac.h					${CPP_DIR}/${MATH_DIR}/ReconstructionRansac.cpp
	${HEADER_DIR}/${MATH_DIR}/Ransac.h									${CPP_DIR}/${MATH_DIR}/Ransac.cpp
	${HEADER_DIR}/${MATH_DIR}/TriangulationDlt.h						${CPP_DIR}/${MATH_DIR}/TriangulationDlt.cpp
	${HEADER_DIR}/${MATH_DIR}/CameraMatrixDlt.h							${CPP_DIR}/${MATH_DIR}/CameraMatrixDlt.cpp
	${HEADER_DIR}/${MATH_DIR}/PanoramaDlt.h								${CPP_DIR}/${MATH_DIR}/PanoramaDlt.cpp
	${HEADER_DIR}/${MATH_DIR}/DltBase.h									${CPP_DIR}/${MATH_DIR}/DltBase.cpp
	${HEADER_DIR}/${MATH_DIR}/Correspondence.h							${CPP_DIR}/${MATH_DIR}/Correspondence.cpp
	
	${HEADER_DIR}/${UTILS_DIR}/CircularList.h
)
target_link_libraries (Computer_Vision_Lib
	Qt5::Core
	Qt5::Gui
	Qt5::Widgets
	Qt5::3D
	${OpenCV_LIBS}
	${OPENGL_LIBRARIES}
)
add_subdirectory(test/)

# Add the executable
add_executable (Computer_Vision	
	./main.cpp
)
target_link_libraries (Computer_Vision
	Computer_Vision_Lib
)