##############################
# Tests build description.
# Path: src/test
##############################

add_subdirectory (gtest/)

create_source_groups (UI_DIR MODEL_DIR MATH_DIR UTILS_DIR)

# Adds the executable
add_executable (Tests
	${CPP_DIR}/main.cpp)

target_include_directories (Tests PUBLIC include/)
target_include_directories (Tests PUBLIC gtest
	gtest_main)
target_link_libraries (Tests Computer_Vision_Lib
	gtest
	gtest_main
	pthread)