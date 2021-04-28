# Coverage Path Plannig with Genetic Algorithms
This project utilizes a custom implementation of Genetic Algorithms (GA) with a variable length genome representation that
generates coverage paths on a 2D grid map.

## General Structure
The `tools` are used to implement different path manipulations `path_tools` and provide the foundation for the genome representation `genome_tools`.
Furthermore, the map creation and manipulation process in described in `mapGen`.
In order to serialize or load a GA population `pa_serializer` creates a custom encoding 
```
src/
├── main.cpp
├── optimizer
│   ├── ga
│   │   ├── crossover.cpp
│   │   ├── crossover.h
│   │   ├── fitness.cpp
│   │   ├── fitness.h
│   │   ├── init.cpp
│   │   ├── init.h
│   │   ├── mutation.cpp
│   │   ├── mutation.h
│   │   ├── selection.cpp
│   │   └── selection.h
│   ├── optimizer.cpp
│   ├── optimizer.h
└── tools
    ├── configuration.cpp
    ├── configuration.h
    ├── debug.cpp
    ├── debug.h
    ├── genome_tools.cpp
    ├── genome_tools.h
    ├── mapGen.cpp
    ├── mapGen.h
    ├── pa_serializer.cpp
    ├── pa_serializer.h
    ├── path_tools.cpp
    └── path_tools.h
```


```
test/
├── config.yml
├── test_eigen.cpp
├── test_genome.cpp
├── test_new_optimizer.cpp
└── test_tools.cpp
```
0 directories, 10 files
