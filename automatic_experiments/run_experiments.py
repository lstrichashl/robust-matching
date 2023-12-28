import xmltodict
import os
import subprocess
from pathlib import Path
from multiprocessing.pool import ThreadPool
from functools import reduce

base_dir = '/Users/lior.strichash/private/robust-matching/automatic_experiments'
template_file_path = f'{base_dir}/template.argos'

def work(tmp_file_path):
    process = subprocess.Popen(['argos3', '-c', tmp_file_path])
    process.wait()
    process.kill()
    os.remove(tmp_file_path)

def build():
    os.system("cd ~/private/robust-matching/build && make")

def create_experiment_file(random_seed, non_faulty_count, faulty_count, algorithm, visualization=False):
    tmp_file_path = f'{base_dir}/results/{algorithm}/faulty{faulty_count}/random_seed{random_seed}.argos'
    log_file = f"{tmp_file_path}.log"
    os.makedirs(os.path.dirname(tmp_file_path), exist_ok=True)
    with open(template_file_path) as fd:
        doc = xmltodict.parse(fd.read())

    doc['argos-configuration']['framework']['experiment']['@random_seed'] = random_seed
    doc['argos-configuration']['arena']['distribute'][0]['entity']['@quantity'] = non_faulty_count
    doc['argos-configuration']['arena']['distribute'][1]['entity']['@quantity'] = faulty_count
    doc['argos-configuration']['visualization'] = doc['argos-configuration']['visualization'] if visualization else {}
    doc['argos-configuration']['loop_functions']['params']['@is_commited'] = "true" if algorithm == "commited" else "false" 
    doc['argos-configuration']['loop_functions']['params']['@log_file_path'] = log_file
    if "repeated" in algorithm:
        doc['argos-configuration']['loop_functions']['params']['@repeat_interval'] = algorithm[8:]
    elif "commited" in algorithm:
        doc['argos-configuration']['loop_functions']['params']['@repeat_interval'] = 100000
    to_save_string = xmltodict.unparse(doc)

    with open(tmp_file_path, 'w') as f:
        f.write(to_save_string)

    return tmp_file_path, log_file

def create_all_files(number_of_test_runs, n_robots,algorithm):
    file_paths = []
    for random_seed in range(number_of_test_runs):
        for faulty_count in range(1,11):
            non_faulty_count = n_robots-faulty_count
            tmp_file_path, _ = create_experiment_file(random_seed, non_faulty_count, faulty_count, algorithm)
            file_paths.append(tmp_file_path)
    return file_paths

def main():
    num = 16  # set to the number of workers you want (it defaults to the cpu count of your machine)
    tp = ThreadPool(num)
    n_robots = 20
    build()
    file_paths = reduce(lambda a,b: a+b, 
                            [create_all_files(number_of_test_runs=50, n_robots=n_robots,algorithm="commited"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated1"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated10"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated20"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated40"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated80"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated160"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated320"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated640"),
                            create_all_files(number_of_test_runs=50, n_robots=n_robots, algorithm="repeated1000")]
                        )
    for tmp_file_path in file_paths:
        tp.apply_async(work, (tmp_file_path,))

    tp.close()
    tp.join()

if __name__ == "__main__":
    main()