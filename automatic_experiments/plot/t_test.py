# Import necessary libraries
import numpy as np
from scipy import stats

# Given student scores
student_scores1 = np.array([72, 89, 65, 73, 79, 84, 63, 76, 85, 75])
student_scores2 = np.array([72, 89, 65, 73, 79, 84, 63, 76, 85, 75])


# Hypothesized population mean

# Perform one-sample t-test
t_stat, p_value = stats.ttest_ind(student_scores2, student_scores1)
print("T statistic:", t_stat)
print("P-value:", p_value)

# Setting significance level
alpha = 0.05

# Interpret the results
if p_value < alpha:
    print("Reject the null hypothesis; there is a significant difference between the sample mean and the hypothesized population mean.")
else:
    print("Fail to reject the null hypothesis; there is no significant difference between the sample mean and the hypothesized population mean.")