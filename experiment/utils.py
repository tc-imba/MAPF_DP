import asyncio
from functools import wraps


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
