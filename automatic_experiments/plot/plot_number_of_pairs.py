from plot import stat_all, algo_to_label, get_number_of_pairs, key_to_color, stat_experiment_set
import matplotlib.pyplot as plt
from itertools import product
from expected_number_of_pairs_commited import get_expected_faulty_pairs_in_system_of_n_robots
import numpy as np
import pathlib
import json
from scipy import stats

base_dir = "/home/lior/workspace/robust-matching/automatic_experiments/results/fixed_velocity_random_orientation_randomCommitedMovement/connected"

def plot_data(data, number_of_robots):
    max_faulty_num = (int)(number_of_robots/2+1)
    f_range = range(0,max_faulty_num)
    expected_commited = get_expected_faulty_pairs_in_system_of_n_robots(n = number_of_robots, f_range=f_range)
    best_case = [(number_of_robots-f)/2 for f in f_range]
    worst_case = [number_of_robots/2-f for f in f_range]
    i = 0
    for v in data.values():
        for result in v["data"]:
            stats = stat_all(results_path = result["dir"], from_cache=True)
            means,stds = get_number_of_pairs(stats)
            plt.errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt="o", label=result["label"], capsize=5, color=key_to_color(result["label"]))
        plt.plot(f_range, np.array(worst_case), "--", label="worst", color="red", alpha=0.3)
        plt.plot(f_range, np.array(best_case), "--", label="best", color="green", alpha=0.3)
        plt.plot(f_range, np.array(expected_commited), "--", label="expected", color="gray", alpha=0.3)
        plt.grid(linestyle = ':')
        plt.legend()
        # plt.title(v["title"])
        plt.xlabel("faults")
        plt.ylabel("pairs")
        i += 1


def plot_compare_faulty_algorithms(number_of_robots: int, range):
    run_tag = f"range_{range}_robots_{number_of_robots}"
    dir = f"{base_dir}/{run_tag}"
    algorithms_directories = [f for f in pathlib.Path(dir).iterdir() if f.is_dir()]
    plots = {}
    for algorithm_directory in algorithms_directories:
        with open(f"{algorithm_directory}/metadata.json") as f:
            metadata = json.load(f)
            if metadata["faulty_algorithm"]["name"] == 'algo_matching_walk_away' and metadata["non_faulty_algorithm"]["name"] == "virtual_forces_random":
                continue
            if metadata["faulty_algorithm"]['name'] not in plots:
                plots[metadata["faulty_algorithm"]['name']] = {
                    "title": algo_to_label(metadata["faulty_algorithm"]['name']),
                    "data": []
                }
            plots[metadata["faulty_algorithm"]['name']]["data"].append({
                "dir": algorithm_directory,
                "label": algo_to_label(metadata["non_faulty_algorithm"]['name'])
            })
    # fig, axs = plt.subplots(1)
    k = list(plots.keys())[3]
    print(k)
    plot_data({k:plots[k]}, number_of_robots)
    plt.savefig(f"/home/lior/workspace/thesis/images/experiments/plot_compare_faults_range{range}_robots{number_of_robots}_{k}.png", bbox_inches='tight')
    # plt.show() 

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


def plot_t_test(number_of_robots, vis_range, f_count):
    nf_algorithms = ["virtual_forces_random", "algo_matching", "repeated"]
    f_algorithm = "crash"
    run_tag = f"range_{vis_range}_robots_{number_of_robots}"
    number_of_pairs = {}
    t_stat1 = []
    t_stat2 = []
    t_stat3 = []
    max_faulty_num = (int)(number_of_robots/2+1)
    f_range = range(1,max_faulty_num)
    for f_count in f_range:
        for nf in nf_algorithms:
            dir = f"{base_dir}/{run_tag}/{nf}_{f_algorithm}/faulty{f_count}"
            _,number_of_pairs[nf],_,_ = stat_experiment_set(directory_path=dir, cuttime=1500)
        t_stat11,_ = stats.ttest_rel(number_of_pairs["virtual_forces_random"], number_of_pairs["algo_matching"])
        t_stat22,_ = stats.ttest_rel(number_of_pairs["virtual_forces_random"], number_of_pairs["repeated"])
        t_stat33,_ = stats.ttest_rel(number_of_pairs["repeated"], number_of_pairs["algo_matching"])
        t_stat1.append(t_stat11)
        t_stat2.append(t_stat22)
        t_stat3.append(t_stat33)
    print(f"{t_stat1=}, {t_stat2=}, {t_stat3=}")
    plt.plot(f_range, np.array(t_stat1), label="VirtualForces and Committed")
    plt.plot(f_range, np.array(t_stat2), label="VirtualForces and IP")
    plt.plot(f_range, np.array(t_stat3), label="IP and Committed")
    plt.plot(f_range, [0.05]*len(f_range), "--", label="0.05 line")
    plt.plot(f_range, [-0.05]*len(f_range), "--", label="0.05 line")
    plt.xlabel("faults")
    plt.ylabel("t value")
    plt.title("T test of algorithms in crash fault")
    plt.legend()
    plt.show()


def plot_compare_number_of_robots(range, f_percent):
    nf_algorithms = ["virtual_forces_random", "algo_matching"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    numbers_of_robots = [10,20,40,80]
    f_robots = (np.array(numbers_of_robots)*f_percent).astype(int)
    plots = [
        {
            "title": f"{algo_to_label(nf)}, {algo_to_label(f)}",
            "data": [{
                "dir": f"{base_dir}/range_{range}_robots_{number_of_robots}/{nf}_{f}/faulty{f_c}",
                "label": f"robots={number_of_robots}, faulty{f_c}",
                "nf_count": number_of_robots-f_c
            } for number_of_robots, f_c in zip(numbers_of_robots, f_robots) ] 
        } for nf, f in product(nf_algorithms, f_algorithms)
    ]
    fig, axs = plt.subplots(nrows=2, ncols=4, layout="constrained",figsize=(16,9))
    axss = axs.flat

    i = 0
    for plot in plots:
        for result in plot["data"]:
            _,_,_,pairs_times = stat_experiment_set(result["dir"])
            means,stds = {},{}
            for pairs, values in pairs_times.items():
                means[pairs] = values['mean']
                stds[pairs] = values['std']
            k = np.array(list(means.keys()))
            v = np.array(list(means.values()))/(result["nf_count"]/2)
            std = np.array(list(stds.values()))/(result["nf_count"]/2)
            axss[i].errorbar(k, v, std, fmt=".-", label=result["label"], capsize=5)
        axss[i].grid(linestyle = ':')
        axss[i].legend(fontsize=7)
        axss[i].set_title(plot["title"])
        axss[i].set_ylabel("relative pair number")
        axss[i].set_xlabel("time")
        i += 1

    plt.show()
            

if __name__ == "__main__":
    number_of_robots = 20
    vis_range = 0.5
    f_count = 5
    # for vis_range in [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]:
    plot_compare_faulty_algorithms(number_of_robots, vis_range)
    # plot_compare_range(number_of_robots)
    # plot_t_test(number_of_robots, vis_range, f_count)
    # plot_compare_number_of_robots(vis_range, f_percent=0.2)