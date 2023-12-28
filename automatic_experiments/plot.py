from math import comb, factorial, pow
import matplotlib.pyplot as plt
import numpy as np
import csv

from process_logs import get_f_faulty_pairs, stat_all

def M(n):
    k = int(n/2)
    return factorial(n) / (pow(2,k)*factorial(k))



def get_probablity_function_of_n_robots_f_faulty(n,f):
    def inner(number_of_nf_pairs):
        x = number_of_nf_pairs
        if x < f/2 or x > f or n < 2*x:
            return 0
        product = 1
        for i in range(f, 2*x): # loop from f to 2x-1
            product *= (n-i)
        return ( comb(f, 2*(f-x)) * M(2*(f-x)) * product * M(n-2*x) ) / M(n)
    return inner

def expected(prob_function, range):
    e = 0
    for x in range:
        e += x * prob_function(x)
    return e

def assert_probablity_sum_is_1(probablity, range):
    s = 0
    for x in range:
        p = probablity(number_of_nf_pairs=x)
        s += p 
        assert p >= 0, f'{p=} p is negative'
    assert abs(s - 1.0) < 0.00001, f'probablitys not equal to 1' # floating point error

def get_expected_faulty_pairs_in_system_of_n_robots(n, f_range):
    es = []
    for f in f_range:
        probablity = get_probablity_function_of_n_robots_f_faulty(n,f)
        assert_probablity_sum_is_1(probablity, range(f+1))
        e = expected(probablity, range(f+1))
        es.append(e)
    return es

def get_f_pairs_from_file(file_path):
    f = open(file_path, 'r')
    reader = csv.reader(f)
    next(reader, None)  # skip the headers
    number_of_faulty_pairs = {}
    for row in reader:
        faulty_count = int(row[0].split('_')[2][6:])
        if(faulty_count != 0):
            nf_pairs = row[4]
            number_of_faulty_pairs[faulty_count] = 10-float(nf_pairs)
    return number_of_faulty_pairs

def plot_f_faulty_robots_in_20_robots_system():
    f_range = range(1,10+1)
    expected1 = get_expected_faulty_pairs_in_system_of_n_robots(n = 20, f_range=f_range)

    results = stat_all(
        results_path="/Users/lior.strichash/private/robust-matching/automatic_experiments/results/repeated1"
    )
    results_commited = stat_all(
        results_path="/Users/lior.strichash/private/robust-matching/automatic_experiments/results/commited"
    )
    means_commited, stds_commited = get_f_faulty_pairs(results_commited)
    means_repeated, stds_repeated = get_f_faulty_pairs(results)
    plt.plot(f_range, np.array(expected1), "--", label="expected", color="gray", alpha=0.3)
    plt.errorbar(list(means_repeated.keys()), list(means_repeated.values()), list(stds_repeated.values()), fmt="o", label="repeated", capsize=5, color="orange")
    plt.errorbar(list(means_commited.keys()), list(means_commited.values()), list(stds_commited.values()), fmt="o", label="commited", capsize=5, color="#1f77b4")
    plt.grid(linestyle = ':')
    plt.xlabel("f")
    plt.legend()
    plt.xticks(f_range)
    plt.ylabel("Number of faulty pairs")
    plt.show() 

if __name__ == "__main__":
    plot_f_faulty_robots_in_20_robots_system()