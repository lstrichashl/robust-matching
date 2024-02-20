from run_experiments import work, build
from algorithms import Crash, Experiment, Algorithm, VirtualForces, AlgoMatching, NonFaultyAlgorithm, FaultyAlgorithm
from argparse import ArgumentParser, Namespace

def parse_options() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument("-s", "--random_seed", type=int)
    parser.add_argument("-nf", "--non_faulty", type=int, default=20)
    parser.add_argument("-f", "--faulty", type=int, default=0)
    parser.add_argument("-l", "--length", type=int, default=300)
    parser.add_argument("-v", "--visualization", type=bool, default=True)
    parser.add_argument("-a", "--algorithm", type=str, default="virtual_forces")
    options = parser.parse_args()
    return options


if __name__ == "__main__":
    options = parse_options()
    build()
    if options.algorithm == "virtual_forces":
        non_faulty_algorithm = VirtualForces()
        faulty_algorithm = Crash()
        experiment = Experiment(
                    non_faulty_count=options.non_faulty,
                    faulty_count=options.faulty,
                    non_faulty_algorithm=non_faulty_algorithm,
                    faulty_algorithm=faulty_algorithm,
                    random_seed=options.random_seed,
                    length=options.length,
                    visualization=options.visualization
        )
    elif options.algorithm == "commited":
        non_faulty_algorithm = AlgoMatching(is_commited=True)
        faulty_algorithm = Crash()
        experiment = Experiment(
                    non_faulty_count=options.non_faulty,
                    faulty_count=options.faulty,
                    non_faulty_algorithm=non_faulty_algorithm,
                    faulty_algorithm=faulty_algorithm,
                    random_seed=options.random_seed,
                    length=options.length,
                    visualization=options.visualization
        )
    file = experiment.generate_argos_file()
    work(file)