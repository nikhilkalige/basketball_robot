from deap import base, creator, tools, cma
import numpy as np
import pickle
import os
import rospy
import datetime


def generate_name(folder_name):
    current_run = datetime.datetime.now().strftime('%d_%m_%H_%M_%S')
    location = os.path.join(folder_name, current_run)
    if not os.path.exists(location):
        os.mkdir(location)
    return location


def init_creator():
    # We are maximizing only the distance thrown param
    creator.create("FitnessMax", base.Fitness, weights=(1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMax)


def setup_cmaes(ngen, sigma, initial_params, cmaes, eval_function):
    toolbox = base.Toolbox()
    toolbox.register("evaluate", eval_function)

    if not cmaes:
        cmaes = cma.Strategy(centroid=initial_params, sigma=sigma)

    toolbox.register("generate", cmaes.generate, creator.Individual)
    toolbox.register("update", cmaes.update)

    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean, axis=0)
    stats.register("std", np.std, axis=0)
    stats.register("min", np.min, axis=0)
    stats.register("max", np.max, axis=0)

    return (toolbox, cmaes, stats)


def run_cmaes(objects, ngen, pkl_location, verbose=False):
    (toolbox, cmaes, hof, stats, logbook, start_gen, start_child, fitness, population) = objects

    # Variable to indicate first round
    rospy.loginfo("Total generations: {}, starting with gen: {}".format(ngen, start_gen))
    for gen in range(start_gen, ngen):
        rospy.loginfo("Current generation: {}".format(gen))
        # First round
        if gen == start_gen:
            if not population:
                population = toolbox.generate()
        else:
            population = toolbox.generate()
            start_child = 0
            fitness = []

        pickle_data((population, gen, hof, logbook, cmaes), pkl_location, gen)
        for idx in range(start_child, len(population)):
            child = population[idx]
            fitness.append(toolbox.evaluate(child))
            pickle_data((idx, fitness), pkl_location, gen, idx, False)

        for ind, fit in zip(population, fitness):
            ind.fitness.values = fit

        toolbox.update(population)
        hof.update(population)

        record = stats.compile(population)
        logbook.record(gen=gen, pop=population, **record)
        if verbose:
            print logbook.stream

    pickle_data((population, ngen, hof, logbook, cmaes), pkl_location, ngen)


def checkpoint_handle(ngen, checkpoint=None, folder=False, parent_name=False, child_name=False):
    if checkpoint:
        with open('{}/{}_gen.pkl'.format(folder, parent_name), 'r') as f:
            per_gen = pickle.load(f)

        population = per_gen["population"]
        start_gen = per_gen["current_gen"]
        hof = per_gen["halloffame"]
        logbook = per_gen["logbook"]
        cmaes = per_gen["cmaes"]
        np.random.set_state(per_gen["rndstate"])

        if child_name:
            with open('{}/{}_child.pkl'.format(folder, child_name), 'r') as f:
                per_child = pickle.load(f)

            start_child = per_child["current_child"]
            fitness = per_child["fitness"]
        else:
            start_child = 0
            fitness = []

    else:
        start_gen = 0
        population = []
        start_child = 0
        hof = tools.HallOfFame(ngen)
        logbook = tools.Logbook()
        cmaes = False
        fitness = []
        np.random.seed(128)

    return (population, start_gen, start_child, hof, logbook, cmaes, fitness)


def pickle_data(data, location, gen_no, child_no=0, gen=True):
    if gen:
        dict_data = dict(population=data[0], current_gen=data[1], halloffame=data[2],
                         logbook=data[3], cmaes=data[4], rndstate=np.random.get_state())
        filename = 'g{:05d}_gen.pkl'.format(gen_no)
        path = os.path.join(location, filename)
        with open(path, 'wb') as f:
            pickle.dump(dict_data, f)
    else:
        dict_data = dict(current_child=data[0], fitness=data[1])
        filename = 'g{:05d}c{:02d}_child.pkl'.format(gen_no, child_no)
        path = os.path.join(location, filename)
        with open(path, 'wb') as f:
            pickle.dump(dict_data, f)
