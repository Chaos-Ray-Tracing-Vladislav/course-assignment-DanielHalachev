# Download STB
include(ExternalProject)
ExternalProject_Add(
    stb
    PREFIX "vendor/stb"
    GIT_REPOSITORY "https://github.com/nothings/stb.git"
    GIT_TAG master
    TIMEOUT 10
    # CMAKE_ARGS
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    UPDATE_COMMAND ""
)

# Prepare STB (STB is a header-only library)
ExternalProject_Get_Property(stb source_dir)
set(STB_INCLUDE_DIR ${source_dir}/include)
