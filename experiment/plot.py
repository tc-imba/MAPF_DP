from scipy.stats import gaussian_kde, norm
import numpy as npy
import matplotlib.pyplot as plt
import pandas

import os

project_root = os.path.dirname(os.path.dirname(__file__))
result_dir = os.path.join(project_root, "result")
plot_dir = os.path.join(project_root, "plot")
os.makedirs(plot_dir, exist_ok=True)

obstacles_list = [90, 180, 270, 360, 450]


def plot_distribution(scores, title='Grades', xmin=0, xmax=100, bins=20, ytick=5, filename='fig.pdf', preview=False,
                      dpi=300):
    x_grid = npy.linspace(xmin, xmax, 2000)
    bin_grid = npy.linspace(xmin, xmax, bins + 1)
    mean = npy.around(npy.mean(scores), 3)
    std = npy.around(npy.std(scores), 3)
    q1 = npy.around(npy.quantile(scores, 0.25), 3)
    q2 = npy.around(npy.quantile(scores, 0.5), 3)
    q3 = npy.around(npy.quantile(scores, 0.75), 3)
    upper_bound = q3 + 1.5 * (q3 - q1)
    lower_bound = q1 - 1.5 * (q3 - q1)
    outliers = len(scores[scores > upper_bound])

    # bandwidth = 0.2
    kde = gaussian_kde(scores)
    trans = len(scores) * (xmax - xmin) / bins
    pdf_estimate = kde.evaluate(x_grid) * trans
    pdf_normal = norm(mean, std).pdf(x_grid) * trans
    # print(pdf_normal)

    count, _ = npy.histogram(scores, bins=bin_grid)
    ymax = npy.ceil(npy.max(count) / ytick) * ytick

    fig = plt.figure(figsize=(10, 10), dpi=100)
    plt.rcParams.update({'font.size': 16, 'font.family': 'monospace'})

    plt.hist(scores, fill=False, bins=bin_grid)
    plt.plot(x_grid, pdf_normal, color='blue', linewidth=1, label='Normal Distribution')
    plt.plot(x_grid, pdf_estimate, color='red', linewidth=1, dashes=[2, 2], label='Estimated Distribution')

    # locs, labels = plt.yticks()
    box_width = ymax / 5
    plt.boxplot(scores, vert=False, widths=box_width, positions=[ymax + box_width * 2])
    locs = npy.arange(0, ymax + 1, ytick)
    labels = map(lambda x: str(int(x)), locs)
    plt.yticks(locs, labels)
    plt.ylim(0, ymax + box_width * 5)
    plt.xlim(xmin, xmax)

    with npy.printoptions(precision=3):
        text = '  Q1: %s\n  Q2: %s\n  Q3: %s\nMean: %s\n Std: %s\nOutliers: %s\n' % (q1, q2, q3, mean, std, outliers)
    plt.text(xmin, ymax + box_width * 4.5, text, verticalalignment='top')

    plt.legend()
    plt.title(title)
    plt.xlabel('Agents')
    plt.ylabel('Frequency')
    plt.tight_layout()
    if preview:
        plt.show()
    fig.savefig(fname=filename, dpi=dpi)
    plt.close()


def parse_infinite_data():
    main_df = pandas.DataFrame()
    for obstacles in obstacles_list:
        for simulator in ["default", "online"]:
            for agents in [10, 20, 30]:
                for timestep in range(1, 10):
                    for rate in [0.2, 0.4]:
                        file = f"{simulator}-{obstacles}-{agents}-{timestep}-{rate}-0.csv"
                        try:
                            df = pandas.read_csv(os.path.join(result_dir, file), header=None)
                            scores = df.iloc[:, 0]
                            mean = npy.around(npy.mean(scores), 3)
                        except:
                            mean = 0
                        # print(file, mean)
                        row = {
                            "simulator": simulator,
                            "obstacles": obstacles,
                            "agents": agents,
                            "timestep": timestep,
                            "rate": rate,
                            "value": mean
                        }
                        main_df = main_df.append(row, ignore_index=True)
    return main_df


def plot_infinite(data, obstacles, rate):
    df = data[(data["rate"] == rate) & (data["obstacles"] == obstacles)]
    fig = plt.figure(figsize=(16, 9), dpi=100)
    plt.rcParams.update({'font.size': 16, 'font.family': 'monospace'})
    xticks = []
    for simulator, df2 in df.groupby("simulator"):
        if not xticks:
            for i, row in df2.iterrows():
                xticks.append(f"{int(row['timestep'])}")
        x = npy.arange(len(df2))
        y = npy.array(df2["value"] / df2["agents"] * 100)
        agents = npy.array(df2["agents"])
        group_size = 9
        for i in range(0, len(x), group_size):
            plt.plot(
                x[i:i + group_size], y[i:i + group_size], "o-",
                label=f"{simulator} ({int(agents[i])} agents)",
            )
        plt.xticks(x, xticks)
    plt.legend()
    plt.title(f"{obstacles} obstacles, {int(rate * 100)}% of agents blocked infinitely")
    plt.xlabel('Block Timestep')
    plt.ylabel('Success Rate (%)')
    plt.tight_layout()
    # plt.show()
    output_file = os.path.join(plot_dir, f"infinite-{obstacles}-{rate}.png")
    print(output_file)
    fig.savefig(fname=output_file, dpi=300)
    plt.close()


def parse_periodic_data():
    main_df = pandas.DataFrame()
    for obstacles in obstacles_list:
        for simulator in ["default", "online"]:
            for agents in [10, 20, 30]:
                for interval in range(1, 10):
                    for rate in [0.2, 0.4]:
                        timestep = 0
                        file = f"{simulator}-{obstacles}-{agents}-{timestep}-{rate}-{interval}.csv"
                        try:
                            df = pandas.read_csv(os.path.join(result_dir, file), header=None)
                            scores = df.iloc[:, 0]
                            mean = npy.around(npy.mean(scores), 3)
                        except:
                            mean = 0
                        # print(file, mean)
                        row = {
                            "simulator": simulator,
                            "obstacles": obstacles,
                            "agents": agents,
                            "timestep": timestep,
                            "interval": interval,
                            "rate": rate,
                            "value": mean
                        }
                        main_df = main_df.append(row, ignore_index=True)
    return main_df


def plot_periodic(data, obstacles, rate):
    df = data[(data["rate"] == rate) & (data["obstacles"] == obstacles)]
    fig = plt.figure(figsize=(16, 9), dpi=100)
    plt.rcParams.update({'font.size': 16, 'font.family': 'monospace'})
    xticks = []
    for simulator, df2 in df.groupby("simulator"):
        if not xticks:
            for i, row in df2.iterrows():
                xticks.append(f"{int(row['interval'])}")
        x = npy.arange(len(df2))
        y = npy.array(df2["value"])
        agents = npy.array(df2["agents"])
        for i in range(0, len(x), 4):
            plt.plot(
                x[i:i + 4], y[i:i + 4], "o-",
                label=f"{simulator} ({int(agents[i])} agents)",
            )
        plt.xticks(x, xticks)
    plt.legend()
    plt.title(f"{obstacles} obstacles, {int(rate * 100)}% of agents blocked every k timesteps")
    plt.xlabel('Block Interval')
    plt.ylabel('Sum of Cost (Average)')
    plt.tight_layout()
    # plt.show()
    output_file = os.path.join(plot_dir, f"periodic-{obstacles}-{rate}.png")
    print(output_file)
    fig.savefig(fname=output_file, dpi=300)
    plt.close()


def main():
    df_infinite = parse_infinite_data()
    df_periodic = parse_periodic_data()
    # print(df_periodic)
    for obstacles in obstacles_list:
        for rate in [0.2, 0.4]:
            plot_infinite(df_infinite, obstacles, rate)
            # plot_periodic(df_periodic, obstacles, rate)

    # for file in os.listdir(result_dir):
    #     if file.endswith('.csv'):
    #         basename = file[:-4]
    #         print(basename)
    #         df = pandas.read_csv(os.path.join(result_dir, file), header=None)
    #         data = df.iloc[:, 0]
    #         output_file = os.path.join(plot_dir, basename + ".png")
    #         if 'online' in file or 'default' in file:
    #             xmin = 0
    #             xmax = 30
    #         elif 'hardcoded' in file:
    #             xmin = 0
    #             xmax = 60
    #         else:
    #             xmin = min(data) // 10 * 10
    #             xmax = max(data) // 10 * 10
    #         # print(xmin, xmax)
    #         plot_distribution(data, title=basename, filename=output_file, ytick=20, xmin=xmin, xmax=xmax, bins=60)


if __name__ == '__main__':
    main()
