
import matplotlib.pyplot as plt
from itertools import product
from plot import stat_experiment_set, algo_to_label, get_std_mean
import numpy as np

base_dir = "/Users/lior.strichash/private/robust-matching/automatic_experiments/results/fixed_velocity_random_orientation_randomCommitedMovement/connected"


def plot_time_distributions(data, axss):
    # fig, axs = plt.subplots(nrows=2, ncols=1, sharex=True, sharey=True, layout="constrained", figsize=(3.5,5))
    # fig, axs = plt.subplots(nrows=1, ncols=2, sharex=True, sharey=True, layout="constrained", figsize=(6.5,2.5))
    i = 0
    for v in data:
        for result in v["data"]:
            _,_,pairs_times, times_pairs = stat_experiment_set(result["dir"], cuttime=1500)
            means,stds = {},{}
            for time, values in times_pairs.items():
                means[time] = values['mean']
                stds[time] = values['std']
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt=".-", label=result["label"], capsize=5)
        axss[i].grid(linestyle = ':')
        axss[i].legend(fontsize=7)
        axss[i].set_title(v["title"])
        axss[i].set_xlabel("time")
        axss[i].set_ylabel("pair number")
        i += 1

def plot_time_by_pairs_distributions(data, axss):
    # fig, axs = plt.subplots(nrows=2, ncols=1, sharex=True, sharey=True, layout="constrained", figsize=(3.5,5))
    # fig, axs = plt.subplots(nrows=1, ncols=2, sharex=True, sharey=True, layout="constrained", figsize=(6.5,2.5))
    i = 0
    for v in data:
        for result in v["data"]:
            _,_,pairs_times, times_pairs = stat_experiment_set(result["dir"], cuttime=1500)
            means,stds = {},{}
            for time, values in pairs_times.items():
                means[time] = values['mean']
                stds[time] = values['std']
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt="o", label=result["label"], capsize=5)
        axss[i].grid(linestyle = ':')
        axss[i].legend(fontsize=7)
        axss[i].set_title(v["title"])
        axss[i].set_xlabel("pair")
        axss[i].set_ylabel("time")
        i += 1


def plot_compare_faults(number_of_robots, range):
    nf_algorithm = ["virtual_forces_random", "algo_matching"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    faults = ["faulty1", "faulty3","faulty5", "faulty8"]
    plots = [
        {
            "title": f"{algo_to_label(nf)}, {algo_to_label(f)}",
            "data": [{
                "dir": f"{base_dir}/range_{range}_robots_{number_of_robots}/{nf}_{f}/{fault}",
                "label": fault
            }for fault in faults ] 
        }  for nf, f in product(nf_algorithm, f_algorithms)
    ]
    fig, axs = plt.subplots(nrows=2, ncols=4, sharex=True, sharey=True, layout="constrained",figsize=(16,9))
    axss = axs.flat
    plot_time_distributions(plots, axss)
    plt.show()
    # plt.savefig(f"/Users/lior.strichash/private/thesis/images/experiments/plot_compare_faults_range{range}_robots{number_of_robots}.png")


def plot_compare_range(number_of_robots, f_count):
    nf_algorithm = ["virtual_forces_random", "algo_matching"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    ranges = [0.3, 0.5, 100]
    plots = [
        {
            "title": f"{algo_to_label(nf)}, {algo_to_label(f)}",
            "data": [{
                "dir": f"{base_dir}/range_{range}_robots_{number_of_robots}/{nf}_{f}/faulty{f_count}",
                "label": range
            } for range in ranges ] 
        }  for nf, f in product(nf_algorithm, f_algorithms)
    ]

    fig, axs = plt.subplots(nrows=2, ncols=4, layout="constrained" ,figsize=(16,9))
    axss = axs.flat
    plot_time_distributions(plots, axss)
    # i = 0
    # for v in plots:
    #     for result in v["data"]:
    #         _,_,pairs_times, times_pairs = stat_experiment_set(result["dir"])
    #         means,stds = {},{}
    #         for pairs, values in times_pairs.items():
    #             means[pairs] = values['mean']
    #             stds[pairs] = values['std']
    #         axss[i].errorbar(list(means.values()), list(means.keys()), list(stds.values()), fmt="o", label=result["label"], capsize=5)
    #     axss[i].grid(linestyle = ':')
    #     axss[i].legend(fontsize=7)
    #     axss[i].set_title(v["title"])
    #     axss[i].set_xlabel("time")
    #     axss[i].set_ylabel("pair number")
    #     i += 1
    plt.show()


    plt.savefig(f"/Users/lior.strichash/private/thesis/images/experiments/plot_compare_range_primerange_robots{number_of_robots}_faulty{f_count}.png")

def plot_compare_number_of_robots(range, f_count):
    nf_algorithms = ["virtual_forces_random", "algo_matching"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    numbers_of_robots = [6,10,20,40]
    f_robots = (np.array(numbers_of_robots)*f_count).astype(int)
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
    fig, axs = plt.subplots(nrows=2, ncols=4, sharex=True, sharey=True, layout="constrained",figsize=(16,9))
    axss = axs.flat

    i = 0
    for plot in plots:
        for result in plot["data"]:
            _,_,pairs_times = stat_experiment_set(result["dir"])
            means,stds = {},{}
            for pairs, values in pairs_times.items():
                means[pairs] = values['mean']
                stds[pairs] = values['std']
            k = np.array(list(means.keys()))/max(list(means.keys()))
            v = np.array(list(means.values()))/result["nf_count"]
            std = np.array(list(stds.values()))/result["nf_count"]
            axss[i].errorbar(v, k, std, fmt="o", label=result["label"], capsize=5)
        axss[i].grid(linestyle = ':')
        axss[i].legend(fontsize=7)
        axss[i].set_title(plot["title"])
        axss[i].set_ylabel("relative pair number")
        axss[i].set_xlabel("distance traveled")
        i += 1

    plt.show()


def plot_compare_algoithms(number_of_robots, f_count, range):
    nf_algorithms = ["virtual_forces_random", "algo_matching", "repeated"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    plots = [
        {
            "title": algo_to_label(f_algorithm),
            "data": [{
                "dir": f"{base_dir}/range_{range}_robots_{number_of_robots}/{nf_algorithm}_{f_algorithm}/faulty{f_count}",
                "label": algo_to_label(nf_algorithm)
            } for nf_algorithm in nf_algorithms ] 
        } for f_algorithm in f_algorithms
    ]
    fig, axs = plt.subplots(nrows=2, ncols=2, layout="constrained", figsize=(16,9))
    plot_time_by_pairs_distributions(plots, axs.flat)
    plt.show()
    # plt.savefig(f"/Users/lior.strichash/private/thesis/images/experiments/number_of_pairs_by_time_algorithms_range{vis_range}_robots{number_of_robots}_faulty{f_count}.png")


def plot_compare_algoithms_reverse(number_of_robots, f_count, range):
    nf_algorithms = ["virtual_forces_random", "algo_matching"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    plots = [
        {
            "title": algo_to_label(nf_algorithm),
            "data": [{
                "dir": f"{base_dir}/range_{range}_robots_{number_of_robots}/{nf_algorithm}_{f_algorithm}/faulty{f_count}",
                "label": algo_to_label(f_algorithm)
            } for f_algorithm in f_algorithms ] 
        }   for nf_algorithm in nf_algorithms
    ]
    fig, axs = plt.subplots(nrows=1, ncols=2, sharex=True, sharey=True, layout="constrained")
    plot_time_distributions(plots,axs.flat)
    plt.show()
    # plt.savefig(f"/Users/lior.strichash/private/thesis/images/experiments/pairing_time_algorithms_revese_range{vis_range}_robots{number_of_robots}_faulty{f_count}.png")


def plot_range_on_x(number_of_robots, f_count):
    nf_algorithms = ["virtual_forces_random", "algo_matching", "repeated"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    ranges = [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
    # faults = [1,3,5,8]
    tables = [
        {
            "plots":[{
                "label": algo_to_label(nf),
                "data": [{
                    "dir": f"{base_dir}/range_{visrange}_robots_{number_of_robots}/{nf}_{f}/faulty{f_count}",
                    "x": visrange if visrange != 100 else 1
                } for visrange in ranges]
            } for nf in nf_algorithms],
            "title": algo_to_label(f)
        } for f in f_algorithms
    ]
    fig, axs = plt.subplots(nrows=2, ncols=2, layout="constrained")
    axss = axs.flat
    i = 0
    for table in tables:
        for v in table["plots"]:
            means,stds = {},{}
            for result in v["data"]:
                arr_time_to_pairing,_,_,_ = stat_experiment_set(result["dir"], cuttime=1500)
                number_of_pairs = get_std_mean(arr_time_to_pairing)
                means[result["x"]] = number_of_pairs['mean']
                stds[result["x"]] = number_of_pairs['std']
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), label=v["label"],fmt=".-", capsize=5)
        axss[i].grid(linestyle = ':')
        axss[i].legend(fontsize=7)
        axss[i].set_title(table["title"])
        axss[i].set_xlabel("range")
        axss[i].set_ylabel("time max pairing")
        # axss[i].set_yticks(range(11))
        i += 1


    # plt.show()
    plt.savefig(f"/Users/lior.strichash/private/thesis/images/experiments/pairing_time_range_on_x_robots{number_of_robots}_faulty{f_count}.png")


if __name__ == "__main__":
    # base_dir = "/Users/lior.strichash/
    # private/robust-matching/automatic_experiments/results/virtual_forces_gazi"
    # base_dir3 = "/Users/lior.strichash/private/robust-matching/automatic_experiments/results/repeated"
    number_of_robots = 20
    vis_range = 0.5
    f_count = 5
    run_tag = f"range_{vis_range}_robots_{number_of_robots}"
    # plot_compare_faults(number_of_robots, vis_range)
    # plot_compare_number_of_robots(range=vis_range, f_count=0.2)
    # plot_compare_range(number_of_robots, f_count)
    # plot_range_on_x(number_of_robots,f_count)
    # plot_compare_algoithms_reverse(number_of_robots, f_count=f_count, range=vis_range)
    plot_compare_algoithms(number_of_robots, f_count, vis_range)
    # stat_all(results_path = "/Users/lior.strichash/private/robust-matching/automatic_experiments/results/range_100_robots_20/virtual_forces_random_algo_matching_walk_away", from_cache=False)
    # stat_experiment_set(directory_path="automatic_experiments/results/range_100_robots_6/algo_matching_virtual_forces_walk_away/faulty3")