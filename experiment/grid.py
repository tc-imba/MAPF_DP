from matplotlib import pyplot as plt
import numpy as np
from pathlib import Path
import seaborn as sns

project_root = Path(__file__).parent.parent


def main():
    map_file = project_root / "maps" / "continuous-random-30-30-180-0-2.map"
    with map_file.open("r") as file:
        file.readline()
        _, height = file.readline().strip().split(" ")
        _, width = file.readline().strip().split(" ")
        file.readline()
        height, width = int(height), int(width)
        data = np.zeros((height, width), int)
        for i in range(height):
            line = file.readline()
            for j in range(width):
                if line[j] != ".":
                    data[i][j] = 1


    fig, ax = plt.subplots(figsize=(10, 10))
    sns.heatmap(data, square=True, linewidths=0, xticklabels=False, yticklabels=False, cmap=["white", "black"], cbar=False)
    plt.tight_layout()
    plt.savefig(project_root / "plot" / "grid-map.pdf")


if __name__ == '__main__':
    main()