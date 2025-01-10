from plot import stat_all, algo_to_label, get_number_of_pairs, key_to_color, stat_experiment_set, get_std_mean, get_distance_traveled
import matplotlib.pyplot as plt
from itertools import product
from expected_number_of_pairs_commited import get_expected_faulty_pairs_in_system_of_n_robots
import numpy as np
import pathlib
import json
from scipy import stats

base_dir = "/home/lior/workspace/robust-matching/automatic_experiments/results/final/connected"

def plot_data(data, number_of_robots):
    max_faulty_num = (int)(number_of_robots/2+1)
    f_range = range(0,max_faulty_num)
    expected_commited = get_expected_faulty_pairs_in_system_of_n_robots(n = number_of_robots, f_range=f_range)
    best_case = [(number_of_robots-f)/2 for f in f_range]
    worst_case = [number_of_robots/2-f for f in f_range]
    i = 0
    for v in data.values():
        for result in v["data"]:
            stats = stat_all(results_path = result["dir"], from_cache=False)
            means,stds = get_number_of_pairs(stats)
            plt.errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt="o", label=result["label"], capsize=5, color=key_to_color(result["label"]))
        # plt.plot(f_range, np.array(worst_case), "--", label="worst", color="red", alpha=0.3)
        # plt.plot(f_range, np.array(best_case), "--", label="best", color="green", alpha=0.3)
        # plt.plot(f_range, np.array(expected_commited), "--", label="expected", color="gray", alpha=0.3)
        plt.grid(linestyle = ':')
        plt.legend()
        # plt.title(v["title"])
        plt.xlabel("faults")
        plt.ylabel("pairs")
        plt.yticks(range(0,11))
        plt.xticks(range(0,11))
        i += 1


def plot_compare_faulty_algorithms(number_of_robots: int, vis_range, nf_algorithms, f_algorithm):
    run_tag = f"range_{vis_range}_robots_{number_of_robots}"
    dir = f"{base_dir}/{run_tag}"
    plots = {}
    for nf in nf_algorithms:
        algorithm_directory = dir+"/"+nf+"_"+f_algorithm
        with open(f"{algorithm_directory}/metadata.json") as f:
            metadata = json.load(f)
            if metadata["faulty_algorithm"]['@id'] not in plots:
                plots[metadata["faulty_algorithm"]['@id']] = {
                    "title": algo_to_label(metadata["faulty_algorithm"]['@id']),
                    "data": []
                }
            plots[metadata["faulty_algorithm"]['@id']]["data"].append({
                "dir": algorithm_directory,
                "label": algo_to_label(metadata["non_faulty_algorithm"]['@id'])
            })
    # fig, axs = plt.subplots(1)
    k = list(plots.keys())[0]
    print(k)
    plt.figure()
    plot_data({k:plots[k]}, number_of_robots)
    plt.savefig(f"/home/lior/workspace/thesis/images/experiments/plot_compare_faults_range{vis_range}_robots{number_of_robots}_{k}.png", bbox_inches='tight')
    plt.close()
    # plt.show() 

def plot_compare_range(number_of_robots, f_count, f_algorithm):
    nf_algorithms = ["virtual_forces_random_controller", "algo_matching", "repeated", "meeting_point_epuck_controller", "greedy_meeting_point_controller", "greedy_meeting_point_controller_random"]
    ranges = [0.3, 0.5, 2]
    plots = [
        {
            "data": [{
                "dir": f"{base_dir}/range_{vis_range}_robots_{number_of_robots}/{nf}_{f_algorithm}/faulty{f_count}",
            } for nf in nf_algorithms ]
        } for vis_range in ranges
    ]
    
    plt.figure()
    x = np.arange(len(ranges))
    width = 0.1
    group_offsets = np.arange(0,len(nf_algorithms),width)

    means = [[] for _ in range(len(nf_algorithms))]
    errors = [[] for _ in range(len(nf_algorithms))]
    for i, plot in enumerate(plots):
        for j, result in enumerate(plot["data"]):
            _,npairs,_,_ = stat_experiment_set(result["dir"])
            n = get_std_mean(npairs)
            means[j].append(n['mean']/((number_of_robots-f_count)/2))
            errors[j].append(n['std']/((number_of_robots-f_count)/2))
    for i,(m,e) in enumerate(zip(means,errors)):
        label = algo_to_label(nf_algorithms[i])
        plt.bar(
            x+group_offsets[i],
            m,
            width=width,
            yerr=e,
            linestyle="None",
            label=label,
            color=key_to_color(label)
        )
        # plt.errorbar(k, v, std, fmt=".-", label=result["label"], capsize=5)
    plt.grid(linestyle = ':')
    plt.xticks(x, ranges)
    plt.legend(fontsize=7, loc='upper center', ncols=6)
    plt.yticks(np.arange(0,1.4,.2))
    plt.ylabel("relative pairs")
    plt.xlabel("visibility range")

    plt.show()
    # plt.savefig(f"/home/lior/workspace/thesis/images/experiments/plot_compare_range_robots{number_of_robots}_f_count{f_count}_{f_algorithm}.png",bbox_inches='tight')
    # plt.close()


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


def plot_compare_number_of_robots(vis_range, f_percent, f_algorithm):
    nf_algorithms = ["virtual_forces_random_controller", "algo_matching", "repeated", "meeting_point_epuck_controller", "greedy_meeting_point_controller", "greedy_meeting_point_controller_random"]
    numbers_of_robots = [10,20,40,80]
    f_robots = (np.array(numbers_of_robots)*f_percent).astype(int)
    plots = [
        {
            "data": [{
                "dir": f"{base_dir}/range_{vis_range}_robots_{number_of_robots}/{nf}_{f_algorithm}/faulty{f_c}",
                "label": f"robots={number_of_robots}, faulty{f_c}",
                "nf_count": number_of_robots-f_c
            } for nf in nf_algorithms ]
        } for number_of_robots, f_c in zip(numbers_of_robots, f_robots)
    ]

    plt.figure()
    x = np.arange(len(numbers_of_robots))
    width = 0.1
    group_offsets = np.arange(0,len(nf_algorithms),width)

    means = [[] for _ in range(len(nf_algorithms))]
    errors = [[] for _ in range(len(nf_algorithms))]
    for i, plot in enumerate(plots):
        for j, result in enumerate(plot["data"]):
            _,npairs,_,_ = stat_experiment_set(result["dir"])
            n = get_std_mean(npairs)
            means[j].append(n['mean']/(result["nf_count"]/2))
            errors[j].append(n['std']/(result["nf_count"]/2))
    for i,(m,e) in enumerate(zip(means,errors)):
        label = algo_to_label(nf_algorithms[i])
        plt.bar(
            x+group_offsets[i],
            m,
            width=width,
            yerr=e,
            linestyle="None",
            label=label,
            color=key_to_color(label)
        )
        # plt.errorbar(k, v, std, fmt=".-", label=result["label"], capsize=5)
    plt.grid(linestyle = ':')
    plt.xticks(x, numbers_of_robots)
    plt.legend(fontsize=7, loc='upper center', ncols=6)
    plt.yticks(np.arange(0,1.4,0.2))
    plt.ylabel("relative pair number")
    plt.xlabel("numbers of robots")

    # plt.show()
    plt.savefig(f"/home/lior/workspace/thesis/images/experiments/plot_compare_number_of_robots_visrange{vis_range}_f_percent{f_percent}_{f_algorithm}.png",bbox_inches='tight')
    plt.close()


def plot_convergance_time_of_each_fault(number_of_robots, vis_range, nf_algorithms, f_algorithm):
    plt.figure()
    for nf in nf_algorithms:
        means = []
        errors = []
        faults = range(1,11)
        for i in faults:
            arr_time_to_pairing,_,_,_ = stat_experiment_set(f"automatic_experiments/results/final/connected/range_{vis_range}_robots_{number_of_robots}/{nf}_{f_algorithm}/faulty{i}", cuttime=1500)
            number_of_pairs = get_std_mean(np.array(arr_time_to_pairing))
            means.append(number_of_pairs['mean'])
            errors.append(number_of_pairs['std'])
        label = algo_to_label(nf)
        plt.errorbar(faults, means, errors, fmt="-o", label=label, capsize=5, color=key_to_color(label))
    
    plt.xlabel("faults")
    plt.ylabel("time")
    plt.legend()
    plt.show()
    # plt.savefig(f"/home/lior/workspace/thesis/images/experiments/convergance_time_of_each_fault_robots{number_of_robots}_visrange{vis_range}_{f_algorithm}.png",bbox_inches='tight')
    # plt.close()


def plot_distance_traveled(number_of_robots, vis_range,nf_algorithms, f_algorithm):
    plt.figure()
    for nf in nf_algorithms:
        means = []
        errors = []
        faults = range(1,11)
        for i in faults:
            distances = get_distance_traveled(f"automatic_experiments/results/final/connected/range_{vis_range}_robots_{number_of_robots}/{nf}_{f_algorithm}/faulty{i}", cuttime=1500)
            d = get_std_mean(np.array(distances))
            means.append(d['mean'])
            errors.append(d['std'])
        label = algo_to_label(nf)
        plt.errorbar(faults, means, errors, fmt="-o", label=label, capsize=5, color=key_to_color(label))
    
    plt.xlabel("faults")
    plt.ylabel("distance")
    plt.legend()
    plt.yticks(range(0,10))
    # plt.show()
    plt.savefig(f"/home/lior/workspace/thesis/images/experiments/distance_of_each_fault_robots{number_of_robots}_visrange{vis_range}_{f_algorithm}.png",bbox_inches='tight')
    plt.close()

if __name__ == "__main__":
    number_of_robots = 20
    vis_range = 2
    f_count = 5
    # for vis_range in [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]:
    nf_algorithms = ["virtual_forces_random_controller", "algo_matching", "repeated", "greedy_meeting_point_controller", "meeting_point_epuck_controller", "greedy_meeting_point_controller_random"]
    f_algorithms = ["crash", "keep_distance", "opposite", "virtual_forces_walk_away"]
    f_algorithms = [f_algorithms[0]]
    # nf_algorithms = ["greedy_meeting_point_controller", "greedy_meeting_point_controller_random", "meeting_point_epuck_controller"]
    # nf_algorithms = ["virtual_forces_random_controller", "virtual_forces_bot_controller"]

    # plot_compare_faulty_algorithms(number_of_robots, vis_range=0.5,nf_algorithms=nf_algorithms,f_algorithm="crash")
    # plot_compare_range(number_of_robots=10, f_count=2, f_algorithm="crash")
    # plot_t_test(number_of_robots, vis_range, f_count)
    # plot_compare_number_of_robots(0.5, f_percent=0.5,f_algorithm="crash")


    # plot_convergance_time_of_each_fault(number_of_robots,vis_range=0.5, nf_algorithms=nf_algorithms, f_algorithm="virtual_forces_walk_away")

    plot_distance_traveled(number_of_robots, vis_range=0.5, nf_algorithms=nf_algorithms,f_algorithm="crash")