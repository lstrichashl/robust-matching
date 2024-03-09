from math import comb, factorial, pow

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
        es.append(f_range[-1]-e)
    return es