
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

def stat_experiment(
    file_path
):
    a = open_file(file_path)
    last_pairing = a["logs"][-1]["pairs"]
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

        


def plot_pairs():
    f_range = range(0,10+1)
    number_of_robots = 20
    # expected_commited = get_expected_faulty_pairs_in_system_of_n_robots(n = number_of_robots, f_range=f_range)

    best_case = [(number_of_robots-f)/2 for f in f_range]
    worst_case = [number_of_robots/2-f for f in f_range]

    results_commited = stat_all(
        results_path="/Users/lior.strichash/private/robust-matching/automatic_experiments/results/AlgoMatching"
    )
    results_virtual_forces = stat_all(
        results_path="/Users/lior.strichash/private/robust-matching/automatic_experiments/results/VirtualForces",
        from_cache=False
    )
    means_commited, stds_commited = get_number_of_pairs(results_commited)
    means_forces, stds_forces = get_number_of_pairs(results_virtual_forces)
    plt.plot(f_range, np.array(worst_case), "--", label="matching worst", color="red", alpha=0.3)
    plt.plot(f_range, np.array(best_case), "--", label="best", color="green", alpha=0.3)
    plt.errorbar(list(means_commited.keys()), list(means_commited.values()), list(stds_commited.values()), fmt="o", label="commited", capsize=5)
    plt.errorbar(list(means_forces.keys()), list(means_forces.values()), list(stds_forces.values()), fmt="o", label="Virtual Forces", capsize=5)
    
    plt.grid(linestyle = ':')
    plt.xlabel("f")
    plt.legend()
    plt.xticks(f_range)
    plt.yticks(range(0,11))
    plt.ylabel("number of pairs")
    plt.show() 


def plot_time():
    f_range = range(0,10+1)
    number_of_robots = 20

    results_commited = stat_all(
        results_path="/Users/lior.strichash/private/robust-matching/automatic_experiments/results/AlgoMatching"
    )
    results_virtual_forces = stat_all(
        results_path="/Users/lior.strichash/private/robust-matching/automatic_experiments/results/VirtualForces",
        from_cache=False
    )
    means_commited, stds_commited = get_time_to_stable(results_commited)
    means_forces, stds_forces = get_time_to_stable(results_virtual_forces)
    plt.errorbar(list(means_commited.keys()), list(means_commited.values()), list(stds_commited.values()), fmt="o", label="commited", capsize=5)
    plt.errorbar(list(means_forces.keys()), list(means_forces.values()), list(stds_forces.values()), fmt="o", label="Virtual Forces", capsize=5)
    
    plt.grid(linestyle = ':')
    plt.xlabel("f")
    plt.legend()
    plt.xticks(f_range)
    plt.ylabel("time to be stable")
    plt.show() 


if __name__ == "__main__":
    plot_time()
    # plot_pairs()