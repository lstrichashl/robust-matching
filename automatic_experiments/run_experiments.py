import xmltodict
import os
import subprocess
from pathlib import Path
from multiprocessing.pool import ThreadPool

base_dir = '/Users/lior.strichash/private/robust-matching/automatic_experiments'
template_file_path = f'{base_dir}/template.argos'

def work(tmp_file_path):
    print(tmp_file_path)
    process = subprocess.Popen(['argos3', '-c', tmp_file_path])
    process.wait()
    process.kill()
    os.remove(tmp_file_path)

def build():
    os.system("cd ~/private/robust-matching/build && make")

def create_experiment_file(random_seed, non_faulty_count, faulty_count, visualization=False, is_commited=False):
    tmp_file_path = f'{base_dir}/results/non_faulty{non_faulty_count}_faulty{faulty_count}_isCommited{1 if is_commited else 0}/random_seed{random_seed}.argos'
    os.makedirs(os.path.dirname(tmp_file_path), exist_ok=True)
    with open(template_file_path) as fd:
        doc = xmltodict.parse(fd.read())

    doc['argos-configuration']['framework']['experiment']['@random_seed'] = random_seed
    doc['argos-configuration']['arena']['distribute'][0]['entity']['@quantity'] = non_faulty_count
    doc['argos-configuration']['arena']['distribute'][1]['entity']['@quantity'] = faulty_count
    doc['argos-configuration']['visualization'] = doc['argos-configuration']['visualization'] if visualization else {}
    doc['argos-configuration']['loop_functions']['params']['@is_commited'] = "true" if is_commited else "false" 
    to_save_string = xmltodict.unparse(doc)

    with open(tmp_file_path, 'w') as f:
        f.write(to_save_string)

    return tmp_file_path

def create_all_files(number_of_test_runs, is_commited=False):
    file_paths = []
    for random_seed in range(number_of_test_runs):
        for non_faulty_count in range(10,20):
            faulty_count = 20-non_faulty_count
            tmp_file_path = create_experiment_file(random_seed, non_faulty_count, faulty_count, is_commited=is_commited)
            file_paths.append(tmp_file_path)
    return file_paths

def main():
    num = 16  # set to the number of workers you want (it defaults to the cpu count of your machine)
    tp = ThreadPool(num)
    build()
    file_paths = create_all_files(number_of_test_runs=30, is_commited=True)
    for tmp_file_path in file_paths:
        tp.apply_async(work, (tmp_file_path,))

    tp.close()
    tp.join()

if __name__ == "__main__":
    main()