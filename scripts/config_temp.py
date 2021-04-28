config = {
    "logDir": "nonConvexRegionTest",
    "logName": "run.log",
    "maxIterations": 1000,
    "genSeed": 42,

    "printInfo": False,
    "visualize": True,
    # 0: elitist selction,
    # 1: Tournament selection,
    # 2: Roulettwheel selection,
    "scenario": 0,

    # Snapshots,
    "restore": False,
    "snapshot": "",
    "takeSnapshot": True,
    "takeSnapshotEvery": 500,
    "tSnap": "pop.pas",

    # Clear Zero action every X iteration,
    "clearZeros": 0,
    "penalizeZeroActions": True,

    # Fitness:
    "penalizeRotation": False,
    "funSelect": 2,             # Fitness function (formalize fitness itself)
    "fitSselect": 2,            # Fitness strategy (How to calculate coverage)

    # Adaptive parameter:
    "adaptParameter": False,
    "crossUpper": 1,
    "crossLower": 0.5,
    "cLenUpper": 0.1,
    "cLenLower": 0.5,
    "mutUpper": 0.01,

    # Population
    "popMin": 20,

    # Robot:
    "Rob_width": 0.3,           # [m]
    "Rob_speed": 0.2,           # [m/s]
    "Rob_RPM": 60,              # rounds per minute


    # Map,
    "mapType": 1,
    "mapWidth": 11,
    "mapHeight": 11,
    "mapResolution": 0.3,

    # Initialization,
    "initActions": 50,
    "initIndividuals": 100,


    # Selection,
    "keep": 0,
    "select": 50,
    "tournamentSize": 2,
    "selPressure": 2,

    # Crossover,
    "crossoverProba": 0.8,
    "crossLength": 0.3,
    # 0 - local
    # 1 - global
    # 2 - both
    "crossChildSelector": 0,
    "crossStrategy": 0,

    # Mutation stuff
    "mutaOrtoAngleProba": 0.0,
    "mutaRandAngleProba": 0.0,
    "mutaPosDistProba": 0.0,
    "mutaNegDistProba": 0.0,
    "mutaRandScaleDistProba": 0.0,
    "mutaReplaceGen": 0.0,
}
