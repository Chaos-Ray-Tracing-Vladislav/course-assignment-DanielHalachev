# Create an executable
add_executable(main main.cpp)
add_executable(animation animation.cpp)
# We use the tracer library in it
target_link_libraries(main PRIVATE tracer)
target_link_libraries(animation PRIVATE tracer)
