from itertools import product
from typing import List
import xmltodict
import os
import subprocess
from pathlib import Path
from multiprocessing.pool import ThreadPool
from functools import reduce
from algorithms import *
import json
import uuid
from tqdm import tqdm
import time

base_dir = '/home/lior/workspace/robust-matching'
template_file_path = f'{base_dir}/automatic_experiments/templates/template.argos'

def work(tmp_file_path):
    process = subprocess.Popen(['argos3', '-c', tmp_file_path], stdout=subprocess.DEVNULL)
    process.wait()
    process.kill()
    os.remove(tmp_file_path)

def build():
    os.system(f'cd {base_dir}/build && make')

def create_all_files(non_faulty_algorithm:NonFaultyAlgorithm, faulty_algorithm:FaultyAlgorithm, run_tag: str, n_robots: int = 20, number_of_test_runs: int = 50, number_of_faults = range(5,6)):
    file_paths = []
    id = uuid.uuid4()
    experiments_folder = f'{base_dir}/automatic_experiments/results/{run_tag}/{non_faulty_algorithm.name}_{faulty_algorithm.name}'
    for random_seed in range(1,number_of_test_runs+1):
        for faulty_count in number_of_faults:
            experiment = Experiment(
                non_faulty_count=n_robots-faulty_count,
                faulty_count=faulty_count,
                non_faulty_algorithm=non_faulty_algorithm,
                faulty_algorithm=faulty_algorithm,
                random_seed=random_seed,
                run_tag=run_tag,
                visualization=False,
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
    num = 4  # set to the number of workers you want (the default is the cpu count of your machine)
    build()
    # n_robots = 6
    # range = 100
    all_robots = [20]
    # all_range = [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
    # all_range = [0.6,0.7,0.8,0.9,1]
    # all_range = [1,1.5,2]
    all_range = [0.5]
    for n_robots, range in tqdm(product(all_robots,all_range)):
        print(f"{n_robots=} {range=}")
        tp = ThreadPool(num)
        run_tag = f"final/connected/range_{range}_robots_{n_robots}"

        non_faulty_algorithms = [
            AlgoMatching(is_commited=True, range=range),
            AlgoMatching(is_commited=False, name="repeated", repeate_interval=10, range=range),
            VirtualForcesRandom(range=range),
            GreedyMeetingPoints(range=range),
            MeetingPointsEpuck(range=range)
        ]
        # faulty_algorithms = [
        #     VirtualForcesRandomCrash(range=range, start_crash_time=0,end_crash_time=50),
        #     VirtualForcesRandomCrash(range=range, start_crash_time=50,end_crash_time=100),
        #     VirtualForcesRandomCrash(range=range, start_crash_time=100,end_crash_time=150),
        #     VirtualForcesRandomCrash(range=range, start_crash_time=150,end_crash_time=200),
        #     VirtualForcesRandomCrash(range=range, start_crash_time=200,end_crash_time=250),
        #     VirtualForcesRandomCrash(range=range, start_crash_time=250,end_crash_time=300)
        # ]

        # faulty_algorithms = [
        #     AlgoMatchingCrash(range=range, start_crash_time=0,end_crash_time=50),
        #     AlgoMatchingCrash(range=range, start_crash_time=50,end_crash_time=100),
        #     AlgoMatchingCrash(range=range, start_crash_time=100,end_crash_time=150),
        #     AlgoMatchingCrash(range=range, start_crash_time=150,end_crash_time=200),
        #     AlgoMatchingCrash(range=range, start_crash_time=200,end_crash_time=250),
        #     AlgoMatchingCrash(range=range, start_crash_time=250,end_crash_time=300)
        # ]
        faulty_algorithms = [
            AlgoMatchingWalkAway(range=range),
            # Crash(range=range),
            # KeepDistance(range=range),
            VirtualForcesWalkAway(range=range),
        ]
        # non_faulty_algorithms = [
        #     GreedyMeetingPoints(range=range)
        # ]
        file_pathes = []
        for nf_algo, f_algo in product(non_faulty_algorithms, faulty_algorithms):
            file_pathes += create_all_files(number_of_test_runs=50,
                                            n_robots=n_robots,
                                            non_faulty_algorithm=nf_algo,
                                            faulty_algorithm=f_algo,
                                            run_tag=run_tag,
                                            number_of_faults=[1,2,3,4,6,7,8,9,10])
        
        ts_start = time.time()
        # file_pathes = [0,"/home/lior/workspace/robust-matching/automatic_experiments/results/unlimited_visibility/repeated_virtual_forces_walk_away/faulty0/random_seed1.argos",0]
        # ["/home/lior/workspace/robust-matching/automatic_experiments/results/range_100_robots_6/algo_matching_virtual_forces_walk_away/faulty3/random_seed31.argos"]
        for tmp_file_path in file_pathes:
            tp.apply_async(work, (tmp_file_path,))

        tp.close()
        tp.join()

    print(time.time()-ts_start)
if __name__ == "__main__":
    main()