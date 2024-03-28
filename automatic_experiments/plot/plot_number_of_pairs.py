from plot import stat_all, algo_to_label, get_number_of_pairs, key_to_color
import matplotlib.pyplot as plt
from itertools import product
from expected_number_of_pairs_commited import get_expected_faulty_pairs_in_system_of_n_robots
import numpy as np
import pathlib
import json

base_dir = "/Users/lior.strichash/private/robust-matching/automatic_experiments/results/virtual_forces_gazi_attraction/connected"

def plot_data(data, number_of_robots):
    max_faulty_num = (int)(number_of_robots/2+1)
    f_range = range(0,max_faulty_num)
    expected_commited = get_expected_faulty_pairs_in_system_of_n_robots(n = number_of_robots, f_range=f_range)
    best_case = [(number_of_robots-f)/2 for f in f_range]
    worst_case = [number_of_robots/2-f for f in f_range]
    fig, axs = plt.subplots(nrows=1, ncols=len(data), layout="constrained", figsize=(16,9))
    axss = axs.flat
    i = 0
    for v in data.values():
        for result in v["data"]:
            stats = stat_all(results_path = result["dir"], from_cache=False)
            means,stds = get_number_of_pairs(stats)
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt="o", label=result["label"], capsize=5, color=key_to_color(result["label"]))
        axss[i].plot(f_range, np.array(worst_case), "--", label="worst", color="red", alpha=0.3)
        axss[i].plot(f_range, np.array(best_case), "--", label="best", color="green", alpha=0.3)
        axss[i].plot(f_range, np.array(expected_commited), "--", label="expected", color="gray", alpha=0.3)
        axss[i].grid(linestyle = ':')
        axss[i].legend()
        axss[i].set_title(v["title"])
        axss[i].set_xlabel("faults")
        axss[i].set_ylabel("number_of_pairs")
        i += 1
    plt.show() 


def plot_compare_faulty_algorithms(number_of_robots: int, range):
    run_tag = f"range_{range}_robots_{number_of_robots}"
    dir = f"{base_dir}/{run_tag}"
    algorithms_directories = [f for f in pathlib.Path(dir).iterdir() if f.is_dir()]
    plots = {}
    for algorithm_directory in algorithms_directories:
        with open(f"{algorithm_directory}/metadata.json") as f:
            metadata = json.load(f)
            if metadata["faulty_algorithm"]['name'] not in plots:
                plots[metadata["faulty_algorithm"]['name']] = {
                    "title": algo_to_label(metadata["faulty_algorithm"]['name']),
                    "data": []
                }
            plots[metadata["faulty_algorithm"]['name']]["data"].append({
                "dir": algorithm_directory,
                "label": algo_to_label(metadata["non_faulty_algorithm"]['name'])
            })
    plot_data(plots, number_of_robots)
    # plt.savefig(f"/Users/lior.strichash/private/thesis/images/experiments/plot_number_of_pairs_compare_faulty_algorithms{range}_robots{number_of_robots}.png")


def plot_compare_range(number_of_robots):
    nf_algorithm = ["virtual_forces_random", "algo_matching"]
    f_algorithm = ["crash"]
    algorithms = [nf+"_"+f  for nf, f in product(nf_algorithm,f_algorithm)]
    ranges = [0.3, 0.5, 100]
    plots = {
        algorithm: {
            "title": algorithm,
            "data": [{
                "dir": f"{base_dir}/range_{range}_robots_{number_of_robots}/{algorithm}",
                "label": range
            } for range in ranges ] 
        }  for algorithm in algorithms
    }
    plot_data(plots, number_of_robots)

if __name__ == "__main__":
    number_of_robots = 20
    vis_range = 0.5
    # for vis_range in [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]:
    plot_compare_faulty_algorithms(number_of_robots, vis_range)
    # plot_compare_range(number_of_robots)