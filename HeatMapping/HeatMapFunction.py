import csv
import seaborn as sb
from matplotlib import pyplot as plt
import numpy as np
import math
import os


def placeValue(map, x, y, carbon):
    map[y][x] = (carbon, True)
    return


def fillGaps(map):
    (num_rows, num_cols) = map.shape
    true_indices = [(i, j) for i, row in enumerate(map)
                    for j, item in enumerate(row) if item[1]]
    for i in range(num_rows):
        for j in range(num_cols):
            if not map[i][j][1]:
                distances = np.abs(true_indices - np.array((i, j)))
                weights = 1 / np.sum(distances, axis=1)
                weighted_average = np.average(
                    [map[tuple(idx)][0] for idx in true_indices], weights=weights)
                map[i][j] = (weighted_average, False)
    return


def saveMap(map):
    ax = sb.heatmap(map['carbon_concentration'], vmin=600, vmax=1200,
                    cmap='RdYlGn_r', cbar_kws={'pad': 0.1, "shrink": 1}, square=True)
    ax.invert_yaxis()
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_xlabel('')
    ax.set_ylabel('')
    ax.tick_params(axis='both', which='both', length=0)
    plt.savefig("carbon_map.png", dpi=300)
    return


script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)
filename = "co2_concentrations.csv"

dimension = 201  # ---> Adjust as necessary ---> There is a lot dependent on this
scale = 6  # (dimension/2) / scale = max read distance from origin

carbon_map = np.zeros((dimension, dimension), dtype=[
                      ('carbon_concentration', 'f4'), ('is_value_set', '?')])

with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for line in csv_reader:
        A = (float(line[0]))
        B = (float(line[1]))
        z = (float(line[2]))
        # Adjust input by scale and shift so (0,0) is centered
        a = int(A * scale) + math.floor(dimension/2)
        b = int(B * scale) + math.floor(dimension/2)
        placeValue(carbon_map, a, b, z)

fillGaps(carbon_map)
saveMap(carbon_map)
