
from expected_number_of_pairs_commited import get_expected_faulty_pairs_in_system_of_n_robots
import pathlib
import json
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt

def open_file(path):
    f = open(path, 'r')
    data2 = json.load(f)
    return data2


def parse(data):
    expariment_logs = []
    robot_types = {}
    for step in data["logs"]:
        matching = step["paring"]
        expariment_logs.append({
            "matching": matching,
            "tick": int(step["tick"]),
        })
    for type in data["robot_types"]:
        if type["type"] not in robot_types:
            robot_types[type["type"]] = []
        robot_types[type["type"]].append(int(type["robot_id"]))
        
    return robot_types, expariment_logs

def filter_faulty_pairs(pairing, robottypes):
    robot_types = {}
    for type in robottypes:
        if type["type"] not in robot_types:
            robot_types[type["type"]] = []
        robot_types[type["type"]].append(int(type["robot_id"]))
    nf_pairing = []
    for pair in pairing:
        if pair[0] in robot_types['non_faulty'] and pair[1] in robot_types['non_faulty']:
            nf_pairing.append(pair)
    return nf_pairing


def stat_experiment(
    file_path
):
    a = open_file(file_path)
    last_pairing = a["logs"][-1]["pairs"]
    nf_pairing = filter_faulty_pairs(last_pairing, a["robot_types"])
    experiment_length = int(a["logs"][-1]["tick"])
    return len(last_pairing), experiment_length


def stat_experiment_set(
    directory_path
):
    files = [f for f in pathlib.Path(directory_path).iterdir() if f.is_file() and ".log" in f.name]
    arr_time_to_pairing = []
    arr_number_of_pairs = []
    for file in files:
        number_of_pairs, time_to_pairing = stat_experiment(file)
        arr_time_to_pairing.append(time_to_pairing)
        arr_number_of_pairs.append(number_of_pairs)
    return arr_time_to_pairing, arr_number_of_pairs

def get_std_mean(array):
    nparray = np.array(array)
    return {"std": np.std(nparray), "mean": np.mean(nparray)}


def stat_all(
    results_path,
    from_cache = True
):
    cache_file = f"{results_path}/cache.json"
    if from_cache and pathlib.Path(cache_file).is_file():
        with open(cache_file, 'r') as f:
            results = json.load(f)
            return results
    experiments = [f for f in pathlib.Path(results_path).iterdir() if f.is_dir()]
    results = []
    for exp in tqdm(experiments):
        time_to_pairing, number_of_pairs = stat_experiment_set(
            directory_path=exp
        )
        results.append({
            "name": exp.name,
            "time_to_pairing": get_std_mean(time_to_pairing),
            "number_of_pairs":  get_std_mean(number_of_pairs),
        })
        # results = sorted(results, key=lambda x: int(x["name"][6:]))
    with open(cache_file, 'w') as f:
        json.dump(results, f)
    return results

def get_number_of_pairs(results):
    stds = {}
    means = {}
    for r in results:
        faulty_count = int(r["name"][6:])
        nf_pairs = r["number_of_pairs"]
        means[faulty_count] = float(nf_pairs["mean"])
        stds[faulty_count] = float(nf_pairs["std"])
    return means, stds

def get_time_to_stable(results):
    stds = {}
    means = {}
    for r in results:
        faulty_count = int(r["name"][6:])
        nf_pairs = r["time_to_pairing"]
        means[faulty_count] = float(nf_pairs["mean"])
        stds[faulty_count] = float(nf_pairs["std"])
    return means, stds

        

def key_to_color(key:str):
    colors = {
        "virtual_forces":"#ff7f0e",
        "algo_matching": "#1f77b4",
        "repeated": "#17b3cf"
    }
    return colors[key]

def plot_data(data, get_metric_func):
    f_range = range(0,10+1)
    number_of_robots = 20
    expected_commited = get_expected_faulty_pairs_in_system_of_n_robots(n = number_of_robots, f_range=f_range)
    best_case = [(number_of_robots-f)/2 for f in f_range]
    worst_case = [number_of_robots/2-f for f in f_range]
    fig, axs = plt.subplots(nrows=2, ncols=2, layout=None)
    axss = axs.flat
    i = 0
    for k, v in data.items():
        for result in v["data"]:
            stats = stat_all(results_path = result["dir"], from_cache=False)
            means,stds = get_metric_func(stats)
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt="o", label=result["label"], capsize=5, color=key_to_color(result["label"]))
        axss[i].plot(f_range, np.array(worst_case), "--", label="worst", color="red", alpha=0.3)
        axss[i].plot(f_range, np.array(best_case), "--", label="best", color="green", alpha=0.3)
        axss[i].plot(f_range, np.array(expected_commited), "--", label="expected", color="gray", alpha=0.3)
        axss[i].grid(linestyle = ':')
        axss[i].legend()
        axss[i].set_title(k)
        i += 1
    plt.show() 

def plot_runtag(run_tag: str):
    dir = f"/Users/lior.strichash/private/robust-matching/automatic_experiments/results/{run_tag}"
    algorithms_directories = [f for f in pathlib.Path(dir).iterdir() if f.is_dir()]
    plots = {}
    for algorithm_directory in algorithms_directories:
        with open(f"{algorithm_directory}/metadata.json") as f:
            metadata = json.load(f)
            if metadata["faulty_algorithm"]['name'] not in plots:
                plots[metadata["faulty_algorithm"]['name']] = {
                    "title": metadata["faulty_algorithm"]['name'],
                    "data": []
                }
            plots[metadata["faulty_algorithm"]['name']]["data"].append({
                "dir": algorithm_directory,
                "label": metadata["non_faulty_algorithm"]['name']
            })
    plot_data(plots, get_number_of_pairs)


if __name__ == "__main__":
    base_dir = "/Users/lior.strichash/private/robust-matching/automatic_experiments/results/virtual_forces_gazi"
    base_dir3 = "/Users/lior.strichash/private/robust-matching/automatic_experiments/results/repeated"

    run_tag = "range05"
    plot_runtag(run_tag)