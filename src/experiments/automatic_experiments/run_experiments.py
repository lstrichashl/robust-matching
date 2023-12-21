import xmltodict
import os
import subprocess
from pathlib import Path

base_dir = '/Users/lior.strichash/private/robust-matching/src/experiments/automatic_experiments'
template_file_path = f'{base_dir}/template.argos'
number_of_test_runs = 3

os.system("cd ~/private/robust-matching/build && make")

for random_seed in range(number_of_test_runs):
    non_faulty_count = 15
    faulty_count = 5
    tmp_file_path = f'{base_dir}/non_faulty{non_faulty_count}_faulty{faulty_count}/random_seed{random_seed}.argos'
    os.makedirs(os.path.dirname(tmp_file_path), exist_ok=True)
    with open(template_file_path) as fd:
        doc = xmltodict.parse(fd.read())

    doc['argos-configuration']['framework']['experiment']['@random_seed'] = random_seed
    doc['argos-configuration']['arena']['distribute'][0]['entity']['@quantity'] = non_faulty_count
    doc['argos-configuration']['arena']['distribute'][1]['entity']['@quantity'] = faulty_count
    to_save_string = xmltodict.unparse(doc)



    with open(tmp_file_path, 'w') as f:
        f.write(to_save_string)
    # os.system(f"argos3 -c {tmp_file_path}")
    process = subprocess.Popen(['argos3', '-c', tmp_file_path])
    process.wait()
    process.kill()
    os.remove(tmp_file_path)