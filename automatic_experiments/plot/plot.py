import pathlib
import json
import numpy as np
from tqdm import tqdm
from itertools import product

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
    density = 1
    avg_distance = 1
    diameter = 1
    if "init_positions" in a:
        positions = np.array(a["init_positions"])
        diameter = 0
        avg_distance = 0 
        for i,pos1 in enumerate(positions):
            for j,pos2 in enumerate(positions):
                distance = np.linalg.norm(pos1-pos2)
                if distance > diameter:
                    diameter = distance
                avg_distance += distance
        avg_distance /= len(positions)
        density = len(a["robot_types"]) / diameter
    max_pair = -1
    max_i = -1
    distance_traveled = 0
    pairs_times = {}
    times_pairs = {}
    for i, log in enumerate(a["logs"]):
        distance_traveled += log["distance_travel:"]
        num_of_pairs = len(filter_faulty_pairs(log["pairs"], a["robot_types"]))
        if num_of_pairs > max_pair:
            max_pair = num_of_pairs
            pairs_times[num_of_pairs] = int(log["tick"])
        times_pairs[int(log["tick"])] = max_pair
    return pairs_times, times_pairs, diameter


def stat_experiment_set(
    directory_path,
    cuttime = 5000,
    normalize_by_diameter = True
):
    files = [f for f in pathlib.Path(directory_path).iterdir() if f.is_file() and ".log" in f.name]
    arr_time_to_pairing = []
    arr_number_of_pairs = []
    total_pairs_times = {}
    total_times_pairs = {}
    step = int(max(cuttime/30, 10))
    for i, file in enumerate(files):
        pairs_times, times_pairs, diameter = stat_experiment(file)
        mv = list(times_pairs.values())
        # assert all(mv[i+1] - mv[i] > -0.01 for i in range(len(mv)-1)), f"element {i}"
        max_pair = max(pairs_times.keys())
        arr_time_to_pairing.append(pairs_times[max_pair]/diameter if normalize_by_diameter else pairs_times[max_pair])
        arr_number_of_pairs.append(max(times_pairs.values()))
        for pair, time in pairs_times.items():
            if pair not in total_pairs_times:
                total_pairs_times[pair] = []    
            total_pairs_times[pair].append(time)
        for time, pair in times_pairs.items():
            if time < cuttime and time % 10 == 0:
                if time not in total_times_pairs:
                    total_times_pairs[time] = []    
                total_times_pairs[time].append(pair)
        log_step = 10
        experiment_end_time = int(np.ceil((max(times_pairs.keys())+1)/log_step)*log_step)
        for time in range(experiment_end_time, cuttime+log_step, log_step):
            if time not in total_times_pairs:
                total_times_pairs[time] = []    
            total_times_pairs[time].append(max_pair)
    
    a = 1
    new_total_times_pairs = {}
    for pair in total_pairs_times:
        total_pairs_times[pair] = get_std_mean(total_pairs_times[pair])

    for time in total_times_pairs:
        time2 = int(np.ceil(time/step)*step)
        if time2 not in new_total_times_pairs:
            new_total_times_pairs[time2] = []
        new_total_times_pairs[time2].extend(total_times_pairs[time])

    for pair in new_total_times_pairs:
        new_total_times_pairs[pair] = get_std_mean(new_total_times_pairs[pair])

    return arr_time_to_pairing, arr_number_of_pairs, total_pairs_times, new_total_times_pairs

def get_std_mean(array):
    upper_quartile = np.percentile(array, 95)
    lower_quartile = np.percentile(array, 5)
    outliers_filtered = [item for item in array if item >= lower_quartile and item <= upper_quartile ]
    nparray = np.array(outliers_filtered)
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
        time_to_pairing, number_of_pairs, pairs_times, _ = stat_experiment_set(
            directory_path=exp
        )
        results.append({
            "name": exp.name,
            "time_to_pairing": get_std_mean(time_to_pairing),
            "number_of_pairs":  get_std_mean(number_of_pairs),
            "pairs_times": pairs_times
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
        # "virtual_forces_random":"#ff7f0e",
        "Virtual Forces":"#ff7f0e",
        "Committed": "#1f77b4",
        "IP": "#17b3cf"
    }
    return colors.get(key, "#ffffff")

def algo_to_label(algo):
    algo_to_label = {
        "virtual_forces_random": "Virtual Forces",
        "algo_matching": "Committed",
        "keep_distance": "Keep Distance",
        "algo_matching_walk_away": "Matching Walk Away",
        "crash": "Crash",
        "virtual_forces_walk_away": "Virtual Forces Walk Away",
        "repeated": "IP"
    }
    return algo_to_label.get(algo, algo)


if __name__ == "__main__":
    stat_experiment("/Users/lior.strichash/private/robust-matching/automatic_experiments/results/test/algo_matching_algo_matching_walk_away/faulty0/random_seed38.argos.log")