from run_experiments import create_experiment_file, work, build
from process_logs import stat_experiment


if __name__ == "__main__":
    build()
    file, log_file = create_experiment_file( # faulty robot that break two nf_robots
        random_seed=26,
        non_faulty_count=20,
        faulty_count=0,
        algorithm="virtual_forces",
        length=300,
        visualization=True,
    )
    work(file)
    print(stat_experiment(
        file_path=log_file
    ))