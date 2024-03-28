from run_experiments import work, build
from multiprocessing.pool import ThreadPool
from algorithms import Crash, Experiment, Algorithm, VirtualForces, AlgoMatching, NonFaultyAlgorithm, FaultyAlgorithm, algorithmFactory
from argparse import ArgumentParser, Namespace
import time

def parse_options() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument("-s", "--random_seed", type=int)
    parser.add_argument("-nf", "--non_faulty", type=int, default=20)
    parser.add_argument("-f", "--faulty", type=int, default=0)
    parser.add_argument("-l", "--length", type=int, default=300)
    parser.add_argument("-v", "--visualization", type=bool, default=True)
    parser.add_argument("-a", "--algorithm", type=str, default="virtual_forces")
    parser.add_argument("-fa", "--faulty_algorithm", type=str, default="virtual_forces_walk_away")
    parser.add_argument("-r", "--range", type=float, default=5)
    options = parser.parse_args()
    return options


if __name__ == "__main__":
    options = parse_options()
    tp = ThreadPool(1)
    build()
    non_faulty_algorithm = algorithmFactory(options.algorithm, range=options.range)
    faulty_algorithm = algorithmFactory(options.faulty_algorithm, range=options.range)
    print(options.non_faulty)
    experiment = Experiment(
                non_faulty_count=options.non_faulty,
                faulty_count=options.faulty,
                non_faulty_algorithm=non_faulty_algorithm,
                faulty_algorithm=faulty_algorithm,
                random_seed=options.random_seed,
                length=options.length,
                visualization=True,
                run_tag="test"
    )
    file = experiment.generate_argos_file()
    ts_start = time.time()
    tp.apply_async(work, (file,))
    tp.close()
    tp.join()
    print(time.time()-ts_start)