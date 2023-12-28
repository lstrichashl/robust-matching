from run_experiments import create_experiment_file, work, build
from process_logs import stat_experiment


if __name__ == "__main__":
    build()
    file = create_experiment_file( # faulty robot that break two nf_robots
        random_seed=14, #8,
        non_faulty_count=15, #11,
        faulty_count=5, #0,
        visualization=True,
        is_commited=False
    )
    work(file)
    print(stat_experiment(
        file_path=f"{file}.log"
    ))