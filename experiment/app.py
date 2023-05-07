import click
import dataclasses
from typing import List
from experiment.utils import validate_list


@dataclasses.dataclass
class AppArguments:
    obstacles: List[int]
    agents: List[int]
    simulators: List[str]
    k_neighbors: List[int]
    delay_types: List[str]
    delay_ratios: List[float]
    delay_intervals: List[int]


@click.group()
@click.option("--obstacles", type=str, default="90,270", callback=validate_list(int))
@click.option("--agents", type=str, default="10,20,30", callback=validate_list(int))
@click.option("--simulators", type=str, default="online,default,replan,pibt", callback=validate_list(str))
@click.option("--k-neighbors", type=str, default="2", callback=validate_list(int))
@click.option("--delay-types", type=str, default="agent", callback=validate_list(str))
@click.option("--delay-ratios", type=str, default="0.1,0.2,0.3", callback=validate_list(float))
@click.option("--delay-intervals", type=str, default="1,10,20", callback=validate_list(int))
# @click.option('--debug/--no-debug', default=False)
@click.pass_context
def app(ctx, obstacles, agents, simulators, k_neighbors, delay_types, delay_ratios, delay_intervals):
    ctx.obj = AppArguments(
        obstacles=obstacles,
        agents=agents,
        simulators=simulators,
        k_neighbors=k_neighbors,
        delay_types=delay_types,
        delay_ratios=delay_ratios,
        delay_intervals=delay_intervals,
    )
    # click.echo(ctx.obj)
    # click.echo('Debug mode is %s' % ('on' if debug else 'off'))

