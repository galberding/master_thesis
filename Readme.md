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
* General configuration

| Parameter         | Default          | Options            | Comment                                            |
|:------------------|:-----------------|:-------------------|:---------------------------------------------------|
| logDir            | -                | string             | Name of the logging directory                      |
| logName           | run.log          | string             | Prefix for csv file -> run.log.csv                 |
| maxIterations     | 2000             | >= 0               | Maximum amount of iterations                       |
| visualize         | true             | true, false        | Show live preview of path optimization (best path) |
| printInfo         | true             | true, false        | Print basic status info (*)                        |
| genSeed           | 42               | >= 0               | Random seed                                        |
| retrain           | 0                | >= 0               | See [Retrain](#retrain-procedure)                  |
| restore           | false            | true, false        | See [snapshots](#snapshot-and-restore)             |
| tSnap             | pool.actions     | string             | ""                                                 |
| tPerformanceSnap  | pool.performance | string             | ""                                                 |
| snapshot          | -                | string             | ""                                                 |
| takeSnapshot      | true             | true, false        | ""                                                 |
| takeSnapshotEvery | 1                | >= 1               | ""                                                 |
| mapType           | 1                | 1,2                | Type of map that is generated                      |
| Rob\_width        | 0.3              | > 0                | Robot tool width [m]                               |
| Rob\_speed        | 0.2              | > 0                | Robot speed   [m/s]                                |
| Rob\_RPM          | 60               | >= 0               | Rounds per Minute                                  |
| mapWidth          | 11               | >= 3               |                                                    |
| mapHeight         | 11               | >= 3               |                                                    |
| mapResolution     | 0.2              | > 0, <= Rob\_width |                                                    |

* Genetic Algorithm Configuration

| Parameter         | Default          | Options            | Comment                                            |
|:------------------|:-----------------|:-------------------|:---------------------------------------------------|
| clearZeros        | 0                | >= 0               | At which iteration to remove Zero actions          |
|                   |                  |                    |                                                    |

### Retrain Procedure
The retrain procedure is preformed after `maxIterations` is reached.
After the optimization one predefined part of the map is marked as already covered and the previous population is trained for
as much iterations as stated by `retrain` under the new conditions.
Note that all logging parameters for the retrain process remain the same except that `logDir` is altered to: `logDir/retrain_run/`.

### Snapshot and Restore
A population can be saved to a file. The user can control this by behavior with `takeSnapshot` where `takeSnapshotEvery`
determines after how much iterations the population should be saved to a file.
Additional parameter `tSnap` and `tPerformanceSnap` control the naming of the actual population (`tSnap`) as well as some basic
fitness, coverage and time values (`tPerformanceSnap`). It es recommended to leave those at default, because all evaluation scripts work after those naming conventions.
The name is generated accordingly: `<currentIteration>_<suffix>` in order to differentiate the snapshots from each generation.

If one wants to `restore` a population the following path needs to be passed to `snapshot`:`<logDir>/<iteration>_<tSnap>` where `<*>` means to replace the corresponding content.

(*) Status info contains: `Iteration, best time, best cov, best rotation time, best chromosome size, Avg time, Avg cov, Avg chromosome length, crossover proba, mutation proba, Avg diversity, Std diversity`
