from math import comb, factorial, pow
import matplotlib.pyplot as plt
import numpy as np
import csv

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
    result = get_f_pairs_from_file("/Users/lior.strichash/private/robust-matching/src/experiments/automatic_experiments/results/process_experiments.csv")
    plt.plot(f_range, np.array(expected1), "--", label="expected")
    plt.plot(result.keys(), result.values(), "o", label="experiments")
    plt.grid(linestyle = ':')
    plt.xlabel("f")
    plt.legend()
    plt.xticks(f_range)
    y_min_value = int(min(expected1+list(result.values())))
    y_max_value = int(max(expected1+list(result.values())))+1
    y_range = range(y_min_value, y_max_value)
    plt.yticks(y_range) 
    plt.ylabel("Number of faulty pairs")
    plt.show() 


if __name__ == "__main__":
    plot_f_faulty_robots_in_20_robots_system()