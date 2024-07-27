# Ray Tracer Final Version
## Results
- `dragon.ppm` and `dragon.png` contain a rendered image of `scene1.crtscene` and demonstrates lighting and reflection.
- `room.ppm` and `room.png` contain a rendered image of `scene2.crtscene` and demonstrates lighting, reflection and refraction.
- `dragon.mp4` contains an animation of `scene1.crtscene`
- `room.mp4` contains an animation of `scene2.crtscene`
- `roomGI.ppm` and `roomGI.png` contain a rendered image of `scene2.crtscene` using Global Illumination

## Features
The ray tracer offers the following features, studied in the course:
- lighting
- shadows
- reflection
- refraction
- textures
- speed optimizations - multithreading, bounding boxes and KD tree acceleration structure
- camera movements

Additionally, I experimented with global illumination, which produced decent results, but was very expensive.

I didn't just rely on the assumption that "everything works". I performed profiling using `Thread Sanitizer` and `valgrind` to examine the thread-safety and memory safety of my program. Thankfully, neither tool discovered a problem in the final version of the program. 

## Other things I tried
I experimented with SIMD optimizations for the `Vector` class, namely using `__m128` (4 vectorized floats), instead of `std::array<float, 3>`. I achieved no performance gains. Refactoring the element-wise operations to use loops (in hopes of compiler-induced vectorization) instead of direct indexes didn't result in any performance gain either and I didn't commit this code. It is possible the dissapointing results were caused by my old hardware. 

## What could have been done differently
- In functions, where the result is optional, I could have passed the resulting objects as parameters (references), and returned `bool` instead. Perhaps that would have resulted in slightly better performance. However, I chose to return `std::optional` on purpose, because I find the alternative C-style practice ugly and bug-prone. Code readability and maintainability is also the reason I chose to use stack recursion, instead of a data structure for the core logic.  
- There are several occasions, where I use raw pointers, which are inherently unsafe. I made this decision in order to avoid the overhead of `std::unique_ptr` and especially `std::shared_ptr`. 
- I could have switched to template classes to denote the different types of rays - this is a zero-cost abstraction, unlike the current model, which uses a field to denote the type. That way, I could have squeezed a few additional bytes off every ray and fit more into the cache. However, debugging would have been harder. 
- Perhaps some unit tests would have been a nice addition, but I decided not to do any, due to little time left and the fact that all features work as expected. 
- I wanted to do some profiling using `VTune` or `valgrind`, but an update (perhaps?) changed some necessary kernel parameters and I wasn't able to gain any valuable information from the profiling.

## Conclusion
Overall, I am very happy to have made it this far. Most of the ray tracing theory, as well as multithreading in C++, were new to me and this was my first endeavour in the realm of computer graphics. I am happy with the final version of my ray tracer. While coding it, I gained valuable experience. Thank you for giving me the opportunity to be a part of this course.