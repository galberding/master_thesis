# Coverage Path Plannig with Genetic Algorithms
This project utilizes a custom implementation of Genetic Algorithms (GA) with a variable length genome representation that
generates coverage paths on a 2D grid map.

## Examples
<!-- ![Type:1](https://github.com/galberding/ros_optimizer/blob/master/assets/mtype1.png) -->

<p float="left">
  <img src="https://github.com/galberding/ros_optimizer/blob/master/assets/mtype1.png" width="300" />
  <img src="https://github.com/galberding/ros_optimizer/blob/master/assets/mtype1_retrain.png" width="300" />
  <img src="https://github.com/galberding/ros_optimizer/blob/master/assets/mtype2.png" width="300" />
  <img src="https://github.com/galberding/ros_optimizer/blob/master/assets/mtype2_retrain.png" width="300" />
</p>

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

| Parameter              | Default | Options          | Comment                                                                                                                                            |
|:-----------------------|:--------|:-----------------|:---------------------------------------------------------------------------------------------------------------------------------------------------|
| clearZeros             | 0       | n >= 0           | At which iteration to remove Zero actions                                                                                                          |
| penalizeZeroActions    | false   | true, false      | Reduce fitness when zero actions appear                                                                                                            |
| penalizeRotation       | true    | true, false      | Whether `Rob\_RPM` should be included in the fitness                                                                                               |
| funSelect              | 0       | 0,1,2,3,4        | Fitness function selection (See: [Fitness Calculation](https://github.com/galberding/ros_optimizer/blob/master/src/optimizer/ga/fitness.cpp#L344)) |
| fitSselect             | 1       | 0,1              | See [Coverage Calculation](#coverage-calculation)                                                                                                  |
| initActions            | 50      | n >= 10          | Corresponds to the initial chromosome length                                                                                                       |
| initIndividuals        | 1000    | n >= 1           | Individuals in the initial population                                                                                                              |
| popMin                 | 20      | n >= 1           | Guarantees a minimal amount of individuals inside a population                                                                                     |
| scenario               | 0       | 0,1,2,3          | 0 -> Elite, 1 -> TS, 2 -> PRWS, 3 -> RRWS                                                                                                          |
| keep                   | 0       | n >= 0           | Selection parameter, keep n best individuals                                                                                                       |
| select                 | 10      | n >= 1           | Select individuals for recombination                                                                                                               |
| tournamentSize         | 2       | n >= 1 <= popMin | Parameter for Tournamen Selection (TS)                                                                                                             |
| selPressure            | 1.5     | 1<= n <= 2       | Parameter for Ranked Roulette Wheel Selection (RRWS)                                                                                               |
| crossoverProba         | 0.8     | 0 <= n <= 1      | Crossover probability                                                                                                                              |
| crossLength            | 0.4     | 0.1 <= n <= 0.8  | Information sharing probability during crossover                                                                                                   |
| crossChildSelector     | 2       | 0,1,2            | 0 -> Change locality, 1 -> preserve, 2 -> combined                                                                                                 |
| crossStrategy          | 0       | 0,1              | See [Crossover Strategy](#crossover-strategy)                                                                                                      |
| mutaOrtoAngleProba     | 0       | 0 <= n <= 1      | Mutation Probability: Orthogonal angle offset                                                                                                      |
| mutaRandAngleProba     | 0       | 0 <= n <= 1      | Mutation Probability: Random angle offset                                                                                                          |
| mutaPosDistProba       | 0       | 0 <= n <= 1      | Mutation Probability: Positive distance offset                                                                                                     |
| mutaNegDistProba       | 0       | 0 <= n <= 1      | Mutation Probability: Negative distance offset                                                                                                     |
| mutaRandScaleDistProba | 0       | 0 <= n <= 1      | Mutation Probability: Random distance scale                                                                                                        |
| mutaReplaceGen         | 0       | 0 <= n <= 1      | Mutation Probability: Genome reinitialization                                                                                                        |


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

### Coverage Calculation
The parameter `fitSselect` states what strategy or backend is utilized to calculate the coverage, redundant path segments and how obstacles should be treated.
Setting `fitSselect` to zero causes the backend to work on pixel level. That is, `Rob\_width = MapResolution`. Additionally paths that are generated are guaranteed collision free because those paths have zero fitness.
This is not the case for `fitSselect = 1`, the most recent backend utilizing rectangles to select pixels on the path.
Here objects can intersect with the path. Instead of setting fitness to zero a penalty is applied.

### Crossover Strategy
TODO


(*) Status info contains: `Iteration, best time, best cov, best rotation time, best chromosome size, Avg time, Avg cov, Avg chromosome length, crossover proba, mutation proba, Avg diversity, Std diversity`
