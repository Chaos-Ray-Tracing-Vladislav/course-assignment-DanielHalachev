# Get all header files
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${RayTracer_SOURCE_DIR}/include/tracer/*.h")
# Make a library using the sources and the headers
add_library(tracer Vector.cpp Vertex.cpp Ray.cpp Triangle.cpp Color.cpp Texture.cpp Material.cpp Camera.cpp Scene.cpp SceneParser.cpp RayTracer.cpp ${HEADER_LIST})
add_dependencies(tracer rapidjson stb)
# This directory is needed in order to access the headers
target_include_directories(tracer PUBLIC ../include ../external)
# Make grouping pretty in IDEs
source_group(
    TREE "${PROJECT_SOURCE_DIR}/include"
    PREFIX "Header Files"
    FILES ${HEADER_LIST})
