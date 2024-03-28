import matplotlib.pyplot as plt
import numpy as np

xpoints = np.array(range(0,300))/10

def gazi_repusion(distance):
    b = 10
    c = 30
    return distance * (- b * np.exp(-distance**2/c))

ypoints = [gazi_repusion(x) for x in xpoints]

plt.plot(xpoints, ypoints)
plt.xlabel("Distance")
plt.ylabel("Force")
plt.show()