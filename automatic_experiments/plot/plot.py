
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

        


def plot_pairs(data):
    f_range = range(0,10+1)
    number_of_robots = 20
    expected_commited = get_expected_faulty_pairs_in_system_of_n_robots(n = number_of_robots, f_range=f_range)
    best_case = [(number_of_robots-f)/2 for f in f_range]
    worst_case = [number_of_robots/2-f for f in f_range]
    fig, axs = plt.subplots(nrows=2, ncols=2, layout=None)
    axss = axs.flat
    for i in range(len(data)):
        for result in data[i]:
            stats = stat_all(results_path = result["dir"])
            means,stds = get_number_of_pairs(stats)
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt="o", label=result["label"], capsize=5)
        axss[i].plot(f_range, np.array(worst_case), "--", label="worst", color="red", alpha=0.3)
        axss[i].plot(f_range, np.array(best_case), "--", label="best", color="green", alpha=0.3)
        axss[i].plot(f_range, np.array(expected_commited), "--", label="expected", color="gray", alpha=0.3)
        axss[i].grid(linestyle = ':')
        # axss[i].xlabel("f")
        axss[i].legend()
        # axss[i].xticks(f_range)
        # axss[i].yticks(range(0,11))
        # axss[i].ylabel("number of pairs")
    plt.show() 


def plot_time(data):
    f_range = range(0,10+1)
    number_of_robots = 20
    fig, axs = plt.subplots(nrows=2, ncols=2, layout=None)
    axss = axs.flat
    for i in range(len(data)):
        for result in data[i]:
            stats = stat_all(results_path = result["dir"])
            means,stds = get_time_to_stable(stats)
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt="o", label=result["label"], capsize=5)
        axss[i].grid(linestyle = ':')
        # plt.xlabel("f")
        axss[i].legend()
        # plt.xticks(f_range)
        # plt.ylabel("time to be stable")
    plt.show() 


if __name__ == "__main__":
    base_dir = "/Users/lior.strichash/private/robust-matching/automatic_experiments/results/virtual_forces_gazi"
    base_dir3 = "/Users/lior.strichash/private/robust-matching/automatic_experiments/results/repeated"
    data = [[{
        "dir": f"{base_dir}/AlgoMatching_KeepDistance",
        "label": "commited KeepDistance"
    },{
        "dir": f"{base_dir3}/AlgoMatching_KeepDistance",
        "label": "repeated KeepDistance"
    },{
        "dir": f"{base_dir}/VirtualForces_KeepDistance",
        "label": "virtual_forces KeepDistance"
    }],[{
        "dir": f"{base_dir}/AlgoMatching_Crash",
        "label": "commited Crash"
    },{
        "dir": f"{base_dir3}/AlgoMatching_Crash",
        "label": "repeated Crash"
    },{
        "dir": f"{base_dir}/VirtualForces_Crash",
        "label": "virtual_forces Crash"
    }],[{
        "dir": f"{base_dir}/AlgoMatching_AlgoMatchingWalkAway",
        "label": "commited AlgoMatchingWalkAway"
    },{
        "dir": f"{base_dir3}/AlgoMatching_AlgoMatchingWalkAway",
        "label": "repeated AlgoMatchingWalkAway"
    }],[{
        "dir": f"{base_dir}/AlgoMatching_VirtualForcesWalkAway",
        "label": "commited VirtualForcesWalkAway"
    },{
        "dir": f"{base_dir3}/AlgoMatching_VirtualForcesWalkAway",
        "label": "repeated VirtualForcesWalkAway"
    },{
        "dir": f"{base_dir}/VirtualForces_VirtualForcesWalkAway",
        "label": "virtual_forces VirtualForcesWalkAway"
    }]]
    # plot_time(data)
    plot_pairs(data)