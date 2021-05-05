# Coverage Path Plannig with Genetic Algorithms
This project utilizes a custom implementation of Genetic Algorithms (GA) with a variable length genome representation that
generates coverage paths on a 2D grid map.

## General Structure
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

### Path Generation Toolbox
The `tools` are used to implement different path manipulations `path_tools` and provide the foundation for the genome representation `genome_tools`.
Furthermore, the map creation and manipulation process in described in `mapGen`.
In order to serialize or load a GA population `pa_serializer` is utilized.
Some debugging functionality with levels `DEBUG`, `INFO` and `WARN` is provided as well as a logger that write the internal state of the `optimizer` to a CSV file.
The behavior of the entire system is controlled by the `configuation`. The user can provide a configuration file at program start that determines the parameter setting
of the GA as well as several options that influence the map generation and logging directories.
A complete list of those parameters as well as their respective default value is described [here](#execution-configuration).


### Genetic Algorithm Operators
The `optimizer` provides the general architectures (two in total). Choosing a `selection` strategy will automatically determine which architecture is used.
Furthermore, logging is performed at the end of each iteration step (fitness, diversity, coverage, time, chromosome length, etc.).
The genome modification strategies are grouped in `init`, `crossover` and `mutation`.
Finally the `fitness` contains several strategies for calculating the fitness of a genome.

## Setup

## Execution Configuration
