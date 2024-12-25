
import matplotlib.pyplot as plt
from itertools import product
from plot import stat_experiment_set, algo_to_label, get_std_mean, key_to_color
import numpy as np

base_dir = "/home/lior/workspace/robust-matching/automatic_experiments/results/final/connected"
# base_dir = "/home/lior/workspace/robust-matching/automatic_experiments/results/arena1.5/connected"

def plot_time_distributions(data, axss):
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
            for time, values in times_pairs.items():
                means[time] = values['mean']
                stds[time] = values['std']
            mv = list(means.values())
            # assert all(mv[i+1] - mv[i] > -0.01 for i in range(len(mv)-1)), f"element {i}"
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt=".-", label=result["label"], capsize=5)
        axss[i].grid(linestyle = ':')
        axss[i].legend(fontsize=7)
        axss[i].set_title(v["title"])
        axss[i].set_xlabel("time")
        axss[i].set_ylabel("pairs")
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
    # plt.savefig(f"/home/lior/workspace/thesis/images/experiments/plot_compare_faults_range{range}_robots{number_of_robots}.png")


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


    plt.savefig(f"/home/lior/workspace/thesis/images/experiments/plot_compare_range_primerange_robots{number_of_robots}_faulty{f_count}.png")

def plot_number_of_robots_on_x(range, f_percent):
    nf_algorithms = ["virtual_forces_random", "algo_matching"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    base_dir2 = "/home/lior/workspace/robust-matching/automatic_experiments/results"
    dirs = ["final/connected", "arena3/connected","arena4.5/connected"]
    numbers_of_robots = [10,20,40,80]
    f_robots = (np.array(numbers_of_robots)*f_percent).astype(int)
    tables = [
        {
            "plots":[{
                "label": algo_to_label(nf),
                "data": [{
                    "dir": f"{base_dir2}/{d}/range_{range}_robots_{n_robots}/{nf}_{f}/faulty{faulty_count}",
                    "x": i
                } for i, (n_robots,faulty_count) in enumerate(zip(numbers_of_robots,f_robots))]
            } for nf in nf_algorithms],
            "title": f"{algo_to_label(f)}, {d}"
        } for f,d in product(f_algorithms, dirs)
    ]
    fig, axs = plt.subplots(nrows=4, ncols=3, layout="constrained")
    axss = axs.flat
    i = 0
    for table in tables:
        for v in table["plots"]:
            means,stds = {},{}
            for result in v["data"]:
                arr_time_to_pairing,_,_,_ = stat_experiment_set(result["dir"], cuttime=1500)
                number_of_pairs = get_std_mean(np.array(arr_time_to_pairing))
                means[result["x"]] = number_of_pairs['mean']
                stds[result["x"]] = number_of_pairs['std']
            axss[i].errorbar(list(means.keys()), list(means.values()), list(stds.values()), label=v["label"],fmt=".", capsize=5)
        axss[i].grid(linestyle = ':')
        axss[i].legend(fontsize=7)
        axss[i].set_title(table["title"])
        axss[i].set_xlabel("swarm size")
        axss[i].set_ylabel("time max pairing")
        axss[i].set_xticks([0,1,2,3], numbers_of_robots)
        i += 1


    plt.show()
    # plt.savefig(f"/home/lior/workspace/thesis/images/experiments/pairing_time_range_on_x_robots{number_of_robots}_faulty{f_count}.png")



def plot_compare_algoithms(number_of_robots, f_count, vis_range):
    nf_algorithms = ["virtual_forces_random", "algo_matching", "repeated"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    plots = [
        {
            "title": algo_to_label(f_algorithm),
            "data": [{
                "dir": f"{base_dir}/range_{vis_range}_robots_{number_of_robots}/{nf_algorithm}_{f_algorithm}/faulty{f_count}",
                "label": algo_to_label(nf_algorithm)
            } for nf_algorithm in nf_algorithms if not(nf_algorithm == "virtual_forces_random" and f_algorithm == "algo_matching_walk_away")] 
        } for f_algorithm in f_algorithms
    ]

    plots = [plots[0]]
    print(plots[0]["title"])
    for v in plots:
        for result in v["data"]:
            _,_,pairs_times, times_pairs = stat_experiment_set(result["dir"], cuttime=1500)
            means,stds = {},{}
            for time, values in times_pairs.items():
                means[time] = values['mean']
                stds[time] = values['std']
            # mv = list(means.values())
            # assert all(mv[i+1] - mv[i] > -0.01 for i in range(len(mv)-1)), f"element {i}"
            plt.errorbar(list(means.keys()), list(means.values()), list(stds.values()), fmt=".-", label=result["label"], capsize=5,color=key_to_color(result["label"]))
        plt.grid(linestyle = ':')
        plt.legend(fontsize=7)
        # plt.set_title(v["title"])
        plt.xlabel("time")
        plt.ylabel("pairs")
        plt.yticks(range(0,11))
    plt.show()
    # plt.savefig(f"/home/lior/workspace/thesis/images/experiments/number_of_pairs_by_time_algorithms_range{vis_range}_robots{number_of_robots}_faulty{f_count}_{v['title']}.png",bbox_inches='tight')


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
    # plt.savefig(f"/home/lior/workspace/thesis/images/experiments/pairing_time_algorithms_revese_range{vis_range}_robots{number_of_robots}_faulty{f_count}.png")


def plot_range_on_x(number_of_robots, f_count):
    # base_dir = "/home/lior/workspace/robust-matching/automatic_experiments/results/arena1.5/connected"
    # f_count=4
    nf_algorithms = ["virtual_forces_random", "algo_matching", "repeated"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    ranges = [0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
    # ranges=[1,1.5,2]
    # faults = [1,3,5,8]
    tables = [
        {
            "plots":[{
                "label": algo_to_label(nf),
                "data": [{
                    "dir": f"{base_dir}/range_{visrange}_robots_{number_of_robots}/{nf}_{f}/faulty{f_count}",
                    "x": visrange if visrange != 100 else 1
                } for visrange in ranges]
            } for nf in nf_algorithms  if not(nf == "virtual_forces_random" and f == "algo_matching_walk_away")],
            "title": algo_to_label(f)
        } for f in f_algorithms
    ]
    tables = [tables[0]]
    i = 0

    plt.figure(figsize=(4,3))
    for table in tables:
        for v in table["plots"]:
            means,stds = {},{}
            for result in v["data"]:
                arr_time_to_pairing,_,_,_ = stat_experiment_set(result["dir"], cuttime=1500)
                number_of_pairs = get_std_mean(np.array(arr_time_to_pairing))
                means[result["x"]] = number_of_pairs['mean']
                stds[result["x"]] = number_of_pairs['std']
            plt.errorbar(list(means.keys()), list(means.values()), list(stds.values()), label=v["label"],fmt=".-", capsize=5,color=key_to_color(v["label"]))
        plt.grid(linestyle = ':')
        plt.legend(fontsize=7)
        # plt.title(table["title"])
        plt.xlabel("range")
        plt.ylabel("time max pairing/diameter")
        plt.yticks(range(100,1000,100))
        i += 1

    # plt.show()
    plt.savefig(f"/home/lior/workspace/thesis/images/experiments/pairing_time_range_on_x_robots{number_of_robots}_faulty{f_count}_{table['title']}.png", bbox_inches='tight')

def plot_compare_crash_timing():
    nf_algorithms = ["virtual_forces_random", "algo_matching", "repeated"]
    f_algorithms = ["crash"]
    plots = [
        {
            "title": algo_to_label(f_algorithm),
            "data": [{
                "dir": f"{base_dir}/range_{range}_robots_{number_of_robots}/{nf_algorithm}_{f_algorithm}/faulty{f_count}",
                "label": algo_to_label(nf_algorithm)
            } for nf_algorithm in nf_algorithms ] 
        } for f_algorithm in f_algorithms
    ]
    fig, axs = plt.subplots(nrows=2, ncols=2, layout="constrained", figsize=(10,6))
    plot_time_by_pairs_distributions(plots, axs.flat)
    plt.show()
    # plt.savefig(f"/home/lior/workspace/thesis/images/experiments/number_of_pairs_by_time_algorithms_range{vis_range}_robots{number_of_robots}_faulty{f_count}.png")


def plot_delay_crash(vis_range):
    nf_algorithms = ["virtual_forces_random", "algo_matching", "repeated"]
    delay_ranges = ["0_50", "50_100", "100_150","150_200", "200_250","250_300"]
    f_count = 5
    number_of_robots = 20
    plots = [{
                "label": algo_to_label(nf),
                "algo": nf,
                "data": [{
                    "dir": f"{base_dir}/range_{vis_range}_robots_{number_of_robots}/{nf}_crash_{delay}/faulty{f_count}",
                    "x": delay.replace("_","-")
                } for delay in delay_ranges]
            } for nf in nf_algorithms
            ]
    for plot in plots:
        plot["data"].insert(0,{
            "dir": f"{base_dir}/range_{vis_range}_robots_{number_of_robots}/{plot['algo']}_crash/faulty{f_count}",
            "x": "0"
        })
    i = 0

    plt.figure(figsize=(6,4))
    for v in plots:
        means,stds = {},{}
        for result in v["data"]:
            arr_time_to_pairing,arr_numer_of_pairs,_,_ = stat_experiment_set(result["dir"], cuttime=1500, normalize_by_diameter=True)
            number_of_pairs = get_std_mean(np.array(arr_numer_of_pairs))
            means[result["x"]] = number_of_pairs['mean']
            stds[result["x"]] = number_of_pairs['std']
        plt.errorbar(list(means.keys()), list(means.values()), list(stds.values()), label=v["label"],fmt=".-", capsize=5)
    
    plt.grid(linestyle = ':')
    plt.legend(fontsize=7)
    # plt.title(table["title"])
    plt.xlabel("crash timing")
    plt.ylabel("time max pairing/diameter")
    plt.yticks(range(4,8))
    i += 1

    plt.show()
    # plt.savefig(f"/home/lior/workspace/thesis/images/experiments/crash_delay_numberpairs_robots{number_of_robots}_range{vis_range}_faulty{f_count}.png", bbox_inches='tight')


def plot_faults_on_x(number_of_robots, visrange):
    # base_dir = "/home/lior/workspace/robust-matching/automatic_experiments/results/arena1.5/connected"
    # f_count=4
    nf_algorithms = ["virtual_forces_random", "algo_matching", "repeated"]
    f_algorithms = ["crash", "keep_distance", "algo_matching_walk_away", "virtual_forces_walk_away"]
    # ranges=[1,1.5,2]
    faults = range(0,11,1)
    tables = [
        {
            "plots":[{
                "label": algo_to_label(nf),
                "data": [{
                    "dir": f"{base_dir}/range_{visrange}_robots_{number_of_robots}/{nf}_{f}/faulty{f_count}",
                    "x": f_count
                } for f_count in faults]
            } for nf in nf_algorithms  if not(nf == "virtual_forces_random" and f == "algo_matching_walk_away")],
            "title": algo_to_label(f)
        } for f in f_algorithms
    ]
    tables = [tables[3]]
    i = 0

    plt.figure(figsize=(4,3))
    for table in tables:
        for v in table["plots"]:
            means,stds = {},{}
            for result in v["data"]:
                arr_time_to_pairing,_,_,_ = stat_experiment_set(result["dir"], cuttime=1500)
                number_of_pairs = get_std_mean(np.array(arr_time_to_pairing))
                means[result["x"]] = number_of_pairs['mean']
                stds[result["x"]] = number_of_pairs['std']
            plt.errorbar(list(means.keys()), list(means.values()), list(stds.values()), label=v["label"],fmt=".-", capsize=5,color=key_to_color(v["label"]))
        plt.grid(linestyle = ':')
        plt.legend(fontsize=7)
        # plt.title(table["title"])
        plt.xlabel("faults")
        plt.ylabel("time max pairing/diameter")
        plt.yticks(range(100,1000,100))
        i += 1

    # plt.show()
    plt.savefig(f"/home/lior/workspace/thesis/images/experiments/pairing_time_faults_on_x_robots{number_of_robots}_faulty{f_count}_{table['title']}.png", bbox_inches='tight')




if __name__ == "__main__":
    # base_dir = "/Users/lior.strichash/
    # private/robust-matching/automatic_experiments/results/virtual_forces_gazi"
    # base_dir3 = "/home/lior/workspace/robust-matching/automatic_experiments/results/repeated"
    number_of_robots = 20
    vis_range = 0.5
    f_count = 5
    run_tag = f"range_{vis_range}_robots_{number_of_robots}"
    # plot_compare_faults(number_of_robots, vis_range)
    # plot_number_of_robots_on_x(range=vis_range, f_percent=0.2)
    # plot_compare_number_of_robots(vis_range, f_percent=0.2)
    # plot_compare_range(number_of_robots, f_count)
    # plot_delay_crash(vis_range)
    # plot_compare_algoithms_reverse(number_of_robots, f_count=f_count, range=vis_range)
    # plot_convergance_time(number_of_robots, f_count, vis_range)
    # stat_all(results_path = "/home/lior/workspace/robust-matching/automatic_experiments/results/range_100_robots_20/virtual_forces_random_algo_matching_walk_away", from_cache=False)
    # stat_experiment_set(directory_path="automatic_experiments/results/range_100_robots_6/algo_matching_virtual_forces_walk_away/faulty3")


    plot_compare_algoithms(number_of_robots, f_count, vis_range)
    # plot_range_on_x(number_of_robots,f_count)
    # plot_faults_on_x(number_of_robots, vis_range)