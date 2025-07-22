import random

def distance_generator(quantity_to_generate, min, max):
    if max >= min:
        if quantity_to_generate > 0:
            yield min
        if quantity_to_generate > 1:
            yield max

        for q in range(0, quantity_to_generate - 2):
            yield random.randint(min, max)

def stop_generator(quantity_to_generate):
    yield from distance_generator(quantity_to_generate, 0, 399)

def slow_generator(quantity_to_generate):
    yield from distance_generator(quantity_to_generate, 400, 799)

def full_speed_generator(quantity_to_generate):
    yield from distance_generator(quantity_to_generate, 800, 10000)

def random_generator(quantity_to_generate):
    yield from distance_generator(quantity_to_generate, 0, 10000)

def corner_cases_generator():
    yield from distance_generator(-1, 0, 10000)
    yield from distance_generator(0, 0, 10000)
    yield from distance_generator(1, 0, 10000)
    yield from distance_generator(2, 0, 10000)
    yield from distance_generator(10, -100, 100)
    yield from distance_generator(10, 100, -100)
    yield from distance_generator(10, 100, 100)

