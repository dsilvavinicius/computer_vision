##############################
# Tests build description.
# Path: src/test
##############################

create_source_groups (UI_DIR MODEL_DIR MATH_DIR UTILS_DIR)

include_directories (include/)

add_subdirectory (gtest/)

# Adds the executable
add_executable (Tests
	${CPP_DIR}/${MODEL_DIR}/PanoramaControllerTest.cpp
	${CPP_DIR}/main.cpp)

target_include_directories (Tests PUBLIC
	include/
	gtest
	gtest_main
)
target_link_libraries (Tests Computer_Vision_Lib
	gtest
	gtest_main
	pthread
)