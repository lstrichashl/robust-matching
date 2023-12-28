import json
import math
import pathlib
import csv
from tqdm import tqdm

PAIRING_THRESHOLD =  0.07

def open_file(path):
    f = open(path, 'r')
    data2 = json.load(f)
    return data2

def parse_matching(matching_data):
    return eval(matching_data)

def parse(data):
    expariment_logs = []
    robot_types = {}
    for step in data["logs"]:
        matching = parse_matching(step["matching"])
        expariment_logs.append({
            "matching": matching,
            "cost": float(step["cost"]),
            "tick": int(step["tick"]),
            "nf_matching_cost": float(step["nf_matching_cost"]),
            "nf_half_matching_cost": float(step["nf_half_matching_cost"]),
        })
    for type in data["robot_types"]:
        if type["type"] not in robot_types:
            robot_types[type["type"]] = []
        robot_types[type["type"]].append(int(type["robot_id"]))
        
    return robot_types, expariment_logs

def how_many_times_matching_changed(logs):
    count = 0
    for i in range(1,len(logs)):
        if(logs[i]["matching"] != logs[i-1]["matching"]):
            count += 1
    return count

def get_nf_pairs(matching, robot_types):
    nf_pairs = []
    for pair in matching:
        if pair[0] in robot_types["CRobustMatching"] and pair[1] in robot_types["CRobustMatching"]:
            nf_pairs.append(pair)
    return nf_pairs

def get_time_to_pairing(logs, robot_types):
    for log in logs:
        if log["nf_half_matching_cost"] < PAIRING_THRESHOLD * len(robot_types["CRobustMatching"])/2:
            return log["tick"]
    return -1

def get_minimum_cost_nf_matching(logs, robot_types):
    min_cost = math.inf
    for log in logs:
        if log["nf_matching_cost"] < min_cost:
            min_cost = log["nf_matching_cost"]
    return min_cost

# def get_maximum_nf_pairing(logs, robot_types):
#     max_matching = log["matching"]
#     for log in logs:
#         nf_pairs = get_nf_pairs(log["matching"], robot_types)
#         if len(nf_pairs) > len(max_pair):
#             max_matching = 


def stat_experiment(
    file_path
):
    a = open_file(file_path)
    robot_types, logs = parse(a)
    stablity = how_many_times_matching_changed(logs)
    nf_pairs = get_nf_pairs(logs[-1]["matching"], robot_types)
    time_to_pairing = get_time_to_pairing(logs, robot_types)
    min_cost = get_minimum_cost_nf_matching(logs, robot_types)
    optimality = 0
    return stablity, time_to_pairing, min_cost, len(nf_pairs), optimality

def stat_set(
    directory_path
):
    files = [f for f in pathlib.Path(directory_path).iterdir() if f.is_file()]
    avg_stablity = 0
    avg_time_to_pairing = 0
    avg_min_cost = 0
    avg_number_of_pairs = 0
    avg_optimality = 0
    for file in files:
        stablity, time_to_pairing, min_cost, number_of_pairs, optimality = stat_experiment(file)
        avg_stablity += stablity
        avg_time_to_pairing += time_to_pairing
        avg_min_cost += min_cost
        avg_number_of_pairs += number_of_pairs
        avg_optimality += optimality
    avg_stablity /= len(files)
    avg_time_to_pairing /= len(files)
    avg_min_cost /= len(files)
    avg_number_of_pairs /= len(files)
    avg_optimality /= len(files)
    return avg_stablity, avg_time_to_pairing, avg_min_cost, avg_number_of_pairs, avg_optimality

def stat_all(
    results_path
):
    experiments = [f for f in pathlib.Path(results_path).iterdir() if f.is_dir()]
    results = []
    for exp in tqdm(experiments):
        stablity, time_to_pairing, min_cost, number_of_pairs, optimality = stat_set(
            directory_path=exp
        )
        results.append({
            "name": exp.name,
            "stablity": stablity,
            "time_to_pairing": time_to_pairing,
            "min_cost": min_cost,
            "number_of_pairs": number_of_pairs,
            "optimality": optimality
        })
    return results


def main():
    # avg_stablity, avg_time_to_pairing, avg_min_cost, avg_number_of_pairs, avg_optimality = stat_set(
    #     directory_path="/Users/lior.strichash/private/robust-matching/src/experiments/automatic_experiments/non_faulty15_faulty5"
    # )
    results = stat_all(
        results_path="/Users/lior.strichash/private/robust-matching/src/experiments/automatic_experiments/results"
    )
    r2 = []
    for r in results:
        r2.append([r["name"], r["time_to_pairing"], r["stablity"], r["optimality"], r["number_of_pairs"]])
    header = ["name", "time_to_pairing", "stablity", "optimality", "number of pairs"]
    with open("/Users/lior.strichash/private/robust-matching/src/experiments/automatic_experiments/results/process_experiments.csv", "w") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(r2)
    
if __name__ == "__main__":
    avg_stablity, avg_time_to_pairing, avg_min_cost, avg_number_of_pairs, avg_optimality = stat_set(
        directory_path="/Users/lior.strichash/private/robust-matching/src/experiments/automatic_experiments/results/non_faulty11_faulty0"
    )