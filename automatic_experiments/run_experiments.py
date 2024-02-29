from typing import List
import xmltodict
import os
import subprocess
from pathlib import Path
from multiprocessing.pool import ThreadPool
from functools import reduce
from algorithms import Crash, Experiment, Algorithm, VirtualForces, AlgoMatching, NonFaultyAlgorithm, FaultyAlgorithm, KeepDistance, VirtualForcesWalkAway, AlgoMatchingWalkAway

base_dir = '/Users/lior.strichash/private/robust-matching/automatic_experiments'
template_file_path = '/Users/lior.strichash/private/robust-matching/automatic_experiments/templates/template.argos'


def work(tmp_file_path):
    process = subprocess.Popen(['argos3', '-c', tmp_file_path])
    process.wait()
    process.kill()
    os.remove(tmp_file_path)

def build():
    os.system("cd ~/private/robust-matching/build && make")

def create_all_files(number_of_test_runs: int, n_robots: int, non_faulty_algorithm:NonFaultyAlgorithm, faulty_algorithm:FaultyAlgorithm, run_tag: str):
    file_paths = []
    for random_seed in range(1,number_of_test_runs+1):
        for faulty_count in range(0, int(n_robots/2 + 1)):
            experiment = Experiment(
                non_faulty_count=n_robots-faulty_count,
                faulty_count=faulty_count,
                non_faulty_algorithm=non_faulty_algorithm,
                faulty_algorithm=faulty_algorithm,
                random_seed=random_seed,
                run_tag=run_tag
            )
            argos_file_path = experiment.generate_argos_file()
            file_paths.append(argos_file_path)
    return file_paths

def main():
    num = 16  # set to the number of workers you want (the default is the cpu count of your machine)
    tp = ThreadPool(num)
    n_robots = 20
    build()
    # file_paths = reduce(lambda a,b: a+b, 
    #                         [create_all_files(number_of_test_runs=50, n_robots=n_robots,algorithm="commited"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated1"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated10"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated20"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated40"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated80"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated160"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated320"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated640"),
    #                         create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated1000")]
    #                     )
    run_tag = "repeated"
    file_paths = create_all_files(number_of_test_runs=50, n_robots=n_robots, non_faulty_algorithm=AlgoMatching(is_commited=False, repeate_interval=1), faulty_algorithm=VirtualForcesWalkAway(), run_tag=run_tag) + \
                create_all_files(number_of_test_runs=50, n_robots=n_robots, non_faulty_algorithm=AlgoMatching(is_commited=False, repeate_interval=1), faulty_algorithm=AlgoMatchingWalkAway(), run_tag=run_tag) + \
                create_all_files(number_of_test_runs=50, n_robots=n_robots, non_faulty_algorithm=AlgoMatching(is_commited=False, repeate_interval=1), faulty_algorithm=Crash(), run_tag=run_tag) +\
                create_all_files(number_of_test_runs=50, n_robots=n_robots, non_faulty_algorithm=AlgoMatching(is_commited=False, repeate_interval=1), faulty_algorithm=KeepDistance(), run_tag=run_tag)
                # create_all_files(number_of_test_runs=50, n_robots=n_robots, non_faulty_algorithm=AlgoMatching(is_commited=True), faulty_algorithm=VirtualForcesWalkAway(), run_tag=run_tag) +\
                # create_all_files(number_of_test_runs=50, n_robots=n_robots, non_faulty_algorithm=AlgoMatching(is_commited=True), faulty_algorithm=AlgoMatchingWalkAway(), run_tag=run_tag) 
    for tmp_file_path in file_paths:
        tp.apply_async(work, (tmp_file_path,))

    tp.close()
    tp.join()

if __name__ == "__main__":
    main()