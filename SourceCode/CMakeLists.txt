# This is the main CMake
cmake_minimum_required(VERSION 3.27.0)
project(RayTracer VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-sign-compare -Wno-unused-parameter -pedantic")

# Test for problems with Thread Sanitizer
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fsanitize=thread -g")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DMEASURE_TIME")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DMEASURE_TIME")

# HW09 Barycentric coordinates
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DBARYCENTRIC")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DBARYCENTRIC")

# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DUSE_TEXTURES")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DUSE_TEXTURES")

include("${CMAKE_SOURCE_DIR}/cmake/FindRapidJSON.cmake")
include("${CMAKE_SOURCE_DIR}/cmake/FindSTB.cmake")

include_directories(${RAPIDJSON_INCLUDE_DIR} ${STB_INCLUDE_DIR})

# include(CTest)
# enable_testing()

add_subdirectory(src)
add_subdirectory(app)

# Testing only available if this is the main app
# Emergency override MODERN_CMAKE_BUILD_TESTING provided as well
# if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MODERN_CMAKE_BUILD_TESTING)
#     AND BUILD_TESTING)
#     add_subdirectory(tests)
# endif()


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
