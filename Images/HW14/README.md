 # Results of HW14
### Tested using Intel Core i5 2540M @2.6GHz
- 2 hardware cores
- 4 logical cores
- 64 KiB L1 Cache
- 512 KiB L2 Cache
- 3 MiB L3 Cache
- 8GB DDR3 RAM

### Acceleration Tree Construction Time
Single-threaded and multi-threaded construction times are mostly identical - approximately 0.19s. Perhaps a larger scene with more objects would make a difference. 

### Render Times

| Optimization                   |    $T_1$ |    $T_2$ |    $T_3$ | $T_\text{AVG}$ |
| ------------------------------ | -------: | -------: | -------: | -------------: |
| None                           | 540.102s | 503.194s | 520.610s |       521.302s |
| Regions                        | 220.674s | 221.076s | 221.327s |       221.025s |
| ThreadPool Buckets             | 205.772s | 202.737s | 203.241s |       203.910s |
| Queue Buckets                  | 205.547s | 210.618s | 207.539s |       203.841s |
| AABB                           | 286.200s | 290.446s | 287.963s |       288.203s |
| AABB + Buckets + Queue         | 112.311s | 112.948s | 112.472s |       112.577s |
| AABB + Buckets + ThreadPool    | 113.403s | 114.703s | 111.399s |       113.168s |
| Acceleration Tree              |  15.608s |  15.600s |  15.702s |        15.636s |
| Acceleration Tree + ThreadPool |   8.674s |   8.620s |   8.698s |         8.664s |
