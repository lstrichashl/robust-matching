from plot_pair_time_distribution import *
from plot_number_of_pairs import *

if __name__ == "__main__":
    number_of_robots = 20
    # vis_range = 2
    # f_count = 0
    # run_tag = f"range_{vis_range}_robots_{number_of_robots}"
    f_algorithms = ["crash", "keep_distance", "opposite", "virtual_forces_walk_away"]
    nf_algorithms = ["virtual_forces_random_controller", "algo_matching", "repeated", "greedy_meeting_point_controller", "meeting_point_epuck_controller"]

    # for f_algorithm in f_algorithms:
    #     plot_evolution_over_time(number_of_robots, f_count=5, vis_range=0.5, nf_algorithms=nf_algorithms, f_algorithm=f_algorithm)
    #     plot_time_by_pair(number_of_robots, f_count=5, vis_range=0.5, nf_algorithms=nf_algorithms, f_algorithm=f_algorithm)
    
    plot_number_of_pairs_random_exploration_comparison(number_of_robots=20,f_count=0)

    # for nf,f in product(nf_algorithms, f_algorithms):
    #     if f_algorithms == 'opposite' and nf_algorithms == "virtual_forces_random_controller":
    #         continue
    #     plot_compare_faulty_algorithms(number_of_robots, vis_range,nf,f)