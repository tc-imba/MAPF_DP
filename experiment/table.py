from scipy.stats import gaussian_kde, norm
import numpy as npy
import matplotlib.pyplot as plt
import pandas

import os

project_root = os.path.dirname(os.path.dirname(__file__))
result_dir = os.path.join(project_root, "result")
plot_dir = os.path.join(project_root, "plot")
data_dir = os.path.join(project_root, "data")


def table_online_offline(data):
    df = data[(data["feasibility"] == "heuristic") & (data["cycle"] == "naive (only cycle)")]
    print(df)


def main():
    df_infinite = pandas.read_csv(os.path.join(data_dir, "df_infinite.csv"))
    df_periodic = pandas.read_csv(os.path.join(data_dir, "df_periodic.csv"))
    table_online_offline(df_infinite)


if __name__ == '__main__':
    main()
