from itertools import product
from typing import List
import xmltodict
import os
import subprocess
from pathlib import Path
from multiprocessing.pool import ThreadPool
from functools import reduce
from algorithms import Crash, Experiment, Algorithm, VirtualForces, AlgoMatching, NonFaultyAlgorithm, FaultyAlgorithm, KeepDistance, VirtualForcesWalkAway, AlgoMatchingWalkAway
import json
import uuid

base_dir = '/Users/lior.strichash/private/robust-matching/automatic_experiments'
template_file_path = '/Users/lior.strichash/private/robust-matching/automatic_experiments/templates/template.argos'


def work(tmp_file_path):
    process = subprocess.Popen(['argos3', '-c', tmp_file_path])
    process.wait()
    process.kill()
    os.remove(tmp_file_path)

def build():
    os.system("cd ~/private/robust-matching/build && make")

def create_all_files(non_faulty_algorithm:NonFaultyAlgorithm, faulty_algorithm:FaultyAlgorithm, run_tag: str, n_robots: int = 20, number_of_test_runs: int = 50):
    file_paths = []
    id = uuid.uuid4()
    experiments_folder = f'{base_dir}/results/{run_tag}/{non_faulty_algorithm.name}_{faulty_algorithm.name}'
    for random_seed in range(1,number_of_test_runs+1):
        for faulty_count in range(0, int(n_robots/2 + 1)):
            experiment = Experiment(
                non_faulty_count=n_robots-faulty_count,
                faulty_count=faulty_count,
                non_faulty_algorithm=non_faulty_algorithm,
                faulty_algorithm=faulty_algorithm,
                random_seed=random_seed,
                run_tag=run_tag,
                file_path=f'{experiments_folder}/faulty{faulty_count}/random_seed{random_seed}.argos'
            )
            argos_file_path = experiment.generate_argos_file()
            file_paths.append(argos_file_path)
    meta_data_file_path = f'{experiments_folder}/metadata.json'
    metadata = {
        "run_tag": run_tag,
        "number_of_robots": n_robots,
        "non_faulty_algorithm": non_faulty_algorithm.to_dict(),
        "faulty_algorithm": faulty_algorithm.to_dict()
    }
    with open(meta_data_file_path, 'w') as f:
        json.dump(metadata, f)
    return file_paths

def main():
    num = 16  # set to the number of workers you want (the default is the cpu count of your machine)
    tp = ThreadPool(num)
    n_robots = 20
    build()
    run_tag = "unlimited_visibilty"

    non_faulty_algorithms = [
        AlgoMatching(is_commited=False, name="repeated", repeate_interval=1),
        AlgoMatching(is_commited=True),
        VirtualForces()
    ]
    faulty_algorithms = [
        VirtualForcesWalkAway(),
        Crash(),
        KeepDistance(),
        AlgoMatchingWalkAway(),
    ]
    file_pathes = []
    for nf_algo, f_algo in product(non_faulty_algorithms, faulty_algorithms):
        file_pathes += create_all_files(number_of_test_runs=1, n_robots=n_robots,non_faulty_algorithm=nf_algo, faulty_algorithm=f_algo, run_tag=run_tag)
    
    for tmp_file_path in file_pathes:
        tp.apply_async(work, (tmp_file_path,))

    tp.close()
    tp.join()

if __name__ == "__main__":
    main()