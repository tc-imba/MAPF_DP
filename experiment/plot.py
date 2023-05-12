import dataclasses
from typing import List, Any
import click
import pandas as pd
import seaborn as sns

from experiment.app import app_command, AppArguments
from experiment.utils import asyncio_wrapper, project_root, ExperimentSetup


@dataclasses.dataclass
class PlotArguments(AppArguments):
    pass


@dataclasses.dataclass
class PlotSettings:
    args: PlotArguments
    plot_type: str
    data_type: str
    subplot_type: str
    y_field: str

    # automatically initialized
    subplot_field: str = ""
    subplot_keys: List[Any] = dataclasses.field(default_factory=list)
    y_log: bool = False
    y_label: str = ""
    x_field: str = ""
    x_label: str = ""

    def __post_init__(self):
        self.y_log = False
        if self.y_field == "time":
            if self.plot_type == "simulator":
                self.y_label = 'Average Computation Time \n of Each Timestep (ms)'
            elif self.plot_type == "feasibility":
                self.y_label = 'Average Computation Time of Each Feasibility Check (ms)'
            elif self.plot_type == "cycle":
                self.y_label = 'Average Computation Time \n of Each Timestep (ms)'
                self.y_log = True
            else:
                assert False
        elif self.y_field == "makespan":
            self.y_label = '\n Makespan'
        elif self.y_field == "soc":
            self.y_label = '\n Sum of Costs'
        else:
            assert False

        self.x_label = 'length of each pause (k)'
        self.x_field = 'delay_interval'

        if self.subplot_type == "delay-ratio":
            self.subplot_field = "delay_ratio"
            self.subplot_keys = self.args.delay_ratios
        elif self.subplot_type == "obstacle":
            self.subplot_field = "obstacles"
            self.subplot_keys = self.args.obstacles
        else:
            assert False


def plot(df: pd.DataFrame, settings: PlotSettings):
    pass


@app_command("plot")
@click.pass_context
def main(ctx):
    data_dir = project_root / "data"
    args = PlotArguments(
        **ctx.obj.__dict__,
    )
    click.echo(args)
