import click
import dataclasses
from typing import List
from functools import wraps
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


def experiment_base_options(f):
    @click.option("--obstacles", type=str, default="90,180,270", callback=validate_list(int))
    @click.option("--agents", type=str, default="30", callback=validate_list(int))
    @click.option("--simulators", type=str, default="online,default,replan,pibt", callback=validate_list(str))
    @click.option("--k-neighbors", type=str, default="2", callback=validate_list(int))
    @click.option("--delay-types", type=str, default="agent", callback=validate_list(str))
    @click.option("--delay-ratios", type=str, default="0.1", callback=validate_list(float))
    @click.option("--delay-intervals", type=str, default="0,1,10,20", callback=validate_list(int))
    @click.pass_context
    @wraps(f)
    def wrapper(ctx, obstacles, agents, simulators, k_neighbors, delay_types, delay_ratios, delay_intervals,
                *args, **kwargs):
        ctx.obj = AppArguments(
            obstacles=obstacles,
            agents=agents,
            simulators=simulators,
            k_neighbors=k_neighbors,
            delay_types=delay_types,
            delay_ratios=delay_ratios,
            delay_intervals=delay_intervals,
        )
        return f(*args, **kwargs)

    return wrapper


def app_command(name):
    context_settings = dict(
        help_option_names=["-h", "--help"],
        max_content_width=88,
    )

    def decorator(func):
        @app.command(name=name, context_settings=context_settings)
        @experiment_base_options
        @wraps(func)
        def wrapped(*args, **kwargs):
            return func(*args, **kwargs)

        return wrapped

    return decorator


@click.group()
def app():
    pass
    # click.echo(ctx.obj)
    # click.echo('Debug mode is %s' % ('on' if debug else 'off'))
