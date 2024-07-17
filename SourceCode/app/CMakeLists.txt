# Create an executable
add_executable(main main.cpp)
# We use the tracer library in it
target_link_libraries(main PRIVATE tracer)
