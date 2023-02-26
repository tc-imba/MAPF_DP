import subprocess
import platform
from pathlib import Path
from typing import Dict
import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
import csv

project_root = Path(__file__).parent.parent
result_dir = project_root / "result"
plot_dir = project_root / "compare_plots"

plot_dir.mkdir(parents=True, exist_ok=True)

data_full_file = result_dir / "time-full.txt"
data_prioritized_file = result_dir / "time-prioritized.txt"

DataType = Dict[int, pd.DataFrame]


def read_data(file: Path, file_type: str) -> DataType:
    result = {}
    with file.open("r") as f:
        lines = f.readlines()
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if len(line) == 0:
                break
            key, length = lines[i].split()
            instance = []
            i += 1
            for j in range(int(length)):
                finished, time = lines[i].strip().split()
                instance.append((int(finished), float(time)))
                i += 1
            df = pd.DataFrame(instance, columns=["finished", "time"])
            df["time_total"] = df["time"].cumsum()
            df["type"] = file_type
            df["index"] = df.index
            result[int(key)] = df

    return result


def print_data(data_full: DataType, data_prioritized: DataType):
    common_keys = set(data_full.keys()).intersection(data_prioritized.keys())
    for key in sorted(common_keys):
        data_full_instance = data_full[key]
        data_prioritized_instance = data_prioritized[key]

        df = pd.concat([data_full_instance, data_prioritized_instance])

        plt.figure()
        sns.scatterplot(df, x="index", y="time_total", style="finished", hue="type", s=10)

        plt.ylabel("accumulated execution time (seconds)")
        plt.xlabel("timestep")
        plt.legend()
        plt.tight_layout()
        plt.savefig(fname=plot_dir / f"{key}.pdf", dpi=300)

    pass


def main():
    data_full = read_data(data_full_file, "full")
    data_prioritized = read_data(data_prioritized_file, "prioritized")
    print_data(data_full, data_prioritized)


if __name__ == '__main__':
    main()
