from gridsearch import trainScenario
from tabulate import tabulate


def collectParameter(confs, collect):
    # collect = {}
    for conf in confs:
        for k,v in conf.items():
            if k in {"maxIterations", "logName", "genSeed", "printInfo", "takeSnapshot", "tSnap", "restore", "scenario", "takeSnapshotEvery", "clearZeros", "penalizeActions", "visualize", "snaps", "snapshot", "penalizeZeroActions", "crossUpper", "crossLower", "cLenUpper", "cLenLower", "mutUpper", "crossStrategy", "mutaOrthoAngleProba", "adaptSP", "mutaNegDistProba", "mutaPosDistProba", "mutaOrtoAngleProba", "mapWidth", "mapHeight", "mapResolution", "logDir", "fitSselect", "adaptParameter", "Rob_width", "Rob_speed", "Rob_RPM", "initActions", "crossoverProba"}:
                continue
            if k not in collect:
                collect[k] = {v}
            else:
                collect[k].add(v)
    # for k,v in collect.items():
    #     print(k , v)

def buildTable():
    col = {}
    for sc in [0,1,2,3]:
        if sc not in col:
            col[sc] = {}
        for mt in [1,2]:
            confs = trainScenario(sc, mt, 2000, "")
            confs2 = trainScenario(sc, mt, 2000, "", rotPanel=True)
            print(sc,"len:", len(confs+confs2))
            collectParameter(confs+confs2, col[sc])
    return (col)

cc = buildTable()

print(tabulate(cc))

col1 = cc[0].keys()
print(col1)
table = [["Para", "0", "1", "2", "3"]]
for c1 in col1:
    row = [c1]
    for i in [0,1,2,3]:
        row.append(", ".join([str(i) for i in list(cc[i][c1])]))
        # print("--------------------")
        # print("Scenario:", i)
        # for k,v in cc[i]self.items():
    table.append(row)
print(tabulate(table, tablefmt="latex"))
