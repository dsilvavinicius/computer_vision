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

# Set include dirs.
find_package (Qt5Widgets)

set (HEADER_DIR "headers")
set (CPP_DIR "src")

# Creating file groups
# Specifying sources.
file (GLOB_RECURSE HEADERS ${HEADER_DIR}/*.h)
file (GLOB_RECURSE CPP ${CPP_DIR}/*.cpp)
# Creating source groups.
source_group (headers FILES ${HEADERS})
source_group (sources FILES ${CPP})

# Add the executable
add_executable (Computer_Vision
	${HEADER_DIR}/notepad.h	${CPP_DIR}/notepad.cpp
	${CPP_DIR}/main.cpp)

# Use the Widgets module from Qt 5.
target_link_libraries (Computer_Vision Qt5::Widgets)