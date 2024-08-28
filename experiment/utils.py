import asyncio
import dataclasses
from functools import wraps
from pathlib import Path
from pydantic import BaseModel

project_root = Path(__file__).parent.parent


def asyncio_wrapper(f):
    @wraps(f)
    def wrapper(*args, **kwargs):
        return asyncio.run(f(*args, **kwargs))

    return wrapper


def validate_list(cast_func):
    def wrapper(ctx, param, value):
        options = value.split(',')
        for i, option in enumerate(options):
            try:
                cast_func(option)
            except ValueError:
                print(f"error: can not cast {option} to {cast_func} in param {param}")
                exit(-1)
            else:
                options[i] = cast_func(option)
        return options

    return wrapper


class ExperimentSetup(BaseModel):
    timing: str
    map: str
    map_name: str
    solver: str
    simulator: str
    agents: int
    delay_type: str
    delay_ratio: float
    delay_start: int
    delay_interval: int
    feasibility: str
    cycle: str

    obstacles: int = 0
    k_neighbor: int = 2

    def get_output_prefix(self):
        if self.map == "random":
            self.map_name = "random-32-32-%d-%d" % (self.obstacles, self.k_neighbor)
        output_prefix = "%s-%s-%s-%s-%d-%s-%s-%d-%d-%s-%s" % (
            self.timing, self.map, self.map_name, self.simulator, self.agents,
            self.delay_type, self.delay_ratio, self.delay_start, self.delay_interval,
            self.feasibility, self.cycle
        )
        return output_prefix

    def get_filter_dict(self):
        if self.map == "random":
            self.map_name = "random-32-32-%d-%d" % (self.obstacles, self.k_neighbor)
        filter_dict = self.model_dump()
        if self.map != "random":
            filter_dict.pop("obstacles", None)
        return {f"setup.{k}": v for k, v in filter_dict.items()}

